#include "mode.h"
#include "Plane.h"
#include <math.h>

/*
  TRACKING flight mode
  ====================
  Controls roll and pitch from external tracking errors received via
  the custom TRACKING MAVLink message (ID 229).

  errorx/errory arrive normalised in [-1, 1] and are converted to
  radians by GCS_MAVLink_Plane before being passed here.

  Roll / pitch: raw error fed directly to PID with settle ramp.
    Deadband zeros the error inside ±TRK_DEADBAND_DEG; active roll-rate
    damping holds wings level while in deadband.

  Throttle: Kalman filter on pitch attitude error + PID, with a linear
    settle ramp over TRK_SETTLE_S seconds from mode entry.

  Terminal: activates when (current_alt_msl - TRK_TGT_ALT) <= TRK_TERM_ALT.
*/



// ── _enter ────────────────────────────────────────────────────────────────────
bool ModeTracking::_enter()
{
    _errorx_rad     = 0.0f;
    _errory_rad     = 0.0f;
    _prev_update_ms = AP_HAL::millis();
    _lock_stable_ms = AP_HAL::millis();
    _cruise_throttle    = plane.aparm.throttle_cruise.get();
    _terminal_entry_ms  = 0;

    // Throttle KF — seed at current pitch error so throttle starts smoothly.
    _kf_x[0]        = 0.0f;
    _kf_x[1]        = 0.0f;
    _kf_P[0]        = 1.0f;
    _kf_P[1]        = 0.0f;
    _kf_P[2]        = 0.0f;
    _kf_P[3]        = 1.0f;
    _kf_initialized     = false;

    _close_enough_prev  = false;
    _last_dist_log_ms   = 0;
    plane.g2.tracking_roll_pid.reset_I();
    plane.g2.tracking_roll_pid.reset_filter();
    plane.g2.tracking_pitch_pid.reset_I();
    plane.g2.tracking_pitch_pid.reset_filter();
    plane.g2.tracking_throt_pid.reset_I();
    plane.g2.tracking_throt_pid.reset_filter();

    gcs().send_text(MAV_SEVERITY_INFO, "Tracking: active");
    return true;
}


// ── run ───────────────────────────────────────────────────────────────────────
void ModeTracking::run()
{
    plane.stabilize_roll();
    plane.stabilize_pitch();
    plane.stabilize_yaw();
}


// ── _exit ─────────────────────────────────────────────────────────────────────
void ModeTracking::_exit()
{
    plane.g2.tracking_roll_pid.reset_I();
    plane.g2.tracking_roll_pid.reset_filter();
    plane.g2.tracking_pitch_pid.reset_I();
    plane.g2.tracking_pitch_pid.reset_filter();
    plane.g2.tracking_throt_pid.reset_I();
    plane.g2.tracking_throt_pid.reset_filter();
    _cruise_throttle   = plane.aparm.throttle_cruise.get();
    RC_Channels::clear_overrides();
    gcs().send_text(MAV_SEVERITY_INFO, "Tracking: exit");
}


// ── handle_tracking_error ─────────────────────────────────────────────────────
// Called by GCS_MAVLink_Plane::handle_tracking_message() with values already
// converted from normalised [-1,1] to radians (× TRACKING_MAX_DELTA_RAD).
void ModeTracking::handle_tracking_error(float errorx_rad, float errory_rad)
{
    _errorx_rad = errorx_rad;
    _errory_rad = errory_rad;
}



// ── update ────────────────────────────────────────────────────────────────────
void ModeTracking::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float    dt_s   = constrain_float((now_ms - _prev_update_ms) * 1e-3f,
                                            0.001f, 0.5f);
    _prev_update_ms = now_ms;

    const float deadband_rad = plane.g2.tracking_deadband_deg.get() * (M_PI / 180.0f);

    // Horizontal distance to target using TRK_TGT_LAT / TRK_TGT_LON.
    const Location target_loc(
        (int32_t)(plane.g2.tracking_target_lat.get() * 1.0e7f),
        (int32_t)(plane.g2.tracking_target_lon.get() * 1.0e7f),
        0, Location::AltFrame::ABSOLUTE);
    const float horiz_dist_m = plane.current_loc.get_distance(target_loc);
    const float close_m      = plane.g2.tracking_close_m.get();
    const bool  close_enough = (close_m > 0.0f) && (horiz_dist_m <= close_m);

  
    // errorx > 0 → target right → roll right (positive bank).
    const float ex_raw = fabsf(_errorx_rad) > deadband_rad ? _errorx_rad : 0.0f;
    const float ey_raw = fabsf(_errory_rad) > deadband_rad ? _errory_rad : 0.0f;

    if (now_ms - _last_dist_log_ms >= 1000U) {
        _last_dist_log_ms = now_ms;
        const float nav_pitch_rad_log = plane.nav_pitch_cd * 0.01f * (M_PI / 180.0f);
        const float alt_rel_m = plane.current_loc.alt * 0.01f
                              - plane.g2.tracking_target_alt_msl.get();
        gcs().send_text(MAV_SEVERITY_INFO,
                        "TRK d=%.0fm alt=%.0fm ex=%.2f ey=%.2f",
                        (double)horiz_dist_m, (double)alt_rel_m,
                        (double)ex_raw, (double)ey_raw);
        gcs().send_text(MAV_SEVERITY_INFO,
                        "TRK p_ahrs=%.1f nav=%.1f err=%.1f",
                        (double)degrees(ahrs.get_pitch()),
                        (double)degrees(nav_pitch_rad_log),
                        (double)degrees(nav_pitch_rad_log - ahrs.get_pitch()));
    }

    // Settle ramp — linear 0→1 over TRK_SETTLE_S seconds from mode entry.
    // Applied to roll, pitch, and throttle PID outputs so all axes come up
    // smoothly together instead of snapping to full authority on entry.
    const float settle_s = plane.g2.tracking_settle_s.get();
    const float elapsed  = constrain_float((now_ms - _lock_stable_ms) * 1e-3f,
                                           0.0f, settle_s);
    const float ramp     = (settle_s > 0.0f) ? (elapsed / settle_s) : 1.0f;

    // ── Roll PID ──────────────────────────────────────────────────────────────
    if (is_zero(ex_raw)) {
        plane.g2.tracking_roll_pid.reset_I();
        // Active damping: oppose roll rate to hold wings level.
        const float roll_rate_dps = degrees(ahrs.get_gyro().x);
        const float damp_cd       = -(roll_rate_dps * 50.0f);
        plane.nav_roll_cd = constrain_int32((int32_t)damp_cd,
                                            -plane.roll_limit_cd,
                                             plane.roll_limit_cd);
    } else {
        const float roll_cd = plane.g2.tracking_roll_pid.update_all(
                                  degrees(ex_raw), 0.0f, dt_s) * ramp;
        plane.nav_roll_cd   = constrain_int32((int32_t)roll_cd,
                                              -plane.roll_limit_cd,
                                               plane.roll_limit_cd);
    }

    // ── Pitch PID ─────────────────────────────────────────────────────────────
    // Sign convention (this build): nav_pitch_cd positive = nose DOWN.
    //   -pitch_offset_deg * 100  →  negative nav_pitch_cd (mounting camera downwards so need to pitch up to look at target on ground)
    //   ey_raw < 0 (below) → need nose down → positive PID input so correction < 0
    // When ey = 0 (deadband): correction = 0, plane holds -pitch_offset setpoint.
    const float pitch_offset_deg = plane.g2.tracking_pitch_offset.get();
    float pitch_correction_cd = 0.0f;
    if (is_zero(ey_raw)) {
        plane.g2.tracking_pitch_pid.reset_I();
    } else {
        pitch_correction_cd = plane.g2.tracking_pitch_pid.update_all(
                                  degrees(ey_raw), pitch_offset_deg, dt_s) * ramp;
    }
    plane.nav_pitch_cd = constrain_int32(
                             (int32_t)pitch_correction_cd,
                             (int32_t)(plane.pitch_limit_min * 100),
                             plane.aparm.pitch_limit_max.get() * 100);

    plane.update_load_factor();

    // ── Throttle (Kalman filter on pitch error) ───────────────────────────────
    // State: x = [pitch_err (rad), pitch_err_rate (rad/s)]
    // Model: constant-velocity  F = [[1,dt],[0,1]]
    // Observation: z = pitch_err,  H = [1,0]
    // Process noise Q = diag(1e-4, TRK_KF_Q);  Measurement noise R = TRK_KF_R
    // Throttle input: x[0] + x[1]*TRK_THR_LEAD  (Kalman-predicted pitch error)
    {
        const float cruise        = plane.aparm.throttle_cruise.get();
        const float lead_s        = plane.g2.tracking_throt_lead.get();
        const float q_vel         = plane.g2.tracking_kf_q.get();
        const float r_meas        = MAX(plane.g2.tracking_kf_r.get(), 1e-6f);
        const float nav_pitch_rad = plane.nav_pitch_cd * 0.01f * (M_PI / 180.0f);
        const float pitch_err     = nav_pitch_rad - ahrs.get_pitch() ;

        if (!_kf_initialized) {
            _kf_x[0]        = pitch_err;
            _kf_x[1]        = 0.0f;
            _kf_P[0]        = 1.0f;
            _kf_P[1]        = 0.0f;
            _kf_P[2]        = 0.0f;
            _kf_P[3]        = 1.0f;
            _kf_initialized = true;
        } else {
            // Predict
            const float px0  = _kf_x[0] + _kf_x[1] * dt_s;
            const float px1  = _kf_x[1];
            const float pp00 = _kf_P[0] + dt_s * (_kf_P[2] + _kf_P[1]) + dt_s * dt_s * _kf_P[3] + 1e-4f;
            const float pp01 = _kf_P[1] + dt_s * _kf_P[3];
            const float pp10 = _kf_P[2] + dt_s * _kf_P[3];
            const float pp11 = _kf_P[3] + q_vel;
            // Update
            const float innov = pitch_err - px0;
            const float S_inv = 1.0f / (pp00 + r_meas);
            const float K0    = pp00 * S_inv;
            const float K1    = pp10 * S_inv;
            _kf_x[0] = px0 + K0 * innov;
            _kf_x[1] = px1 + K1 * innov;
            _kf_P[0] = (1.0f - K0) * pp00;
            _kf_P[1] = (1.0f - K0) * pp01;
            _kf_P[2] = pp10 - K1 * pp00;
            _kf_P[3] = pp11 - K1 * pp01;
        }

        const float pitch_err_pred = _kf_x[0] + _kf_x[1] * lead_s;
        const float pid_out        = plane.g2.tracking_throt_pid.update_all(
                                         pitch_err_pred, 0.0f, dt_s) * ramp;

        const float throttle = constrain_float(cruise + pid_out, 4.0*cruise / 5.0f, 7.0*cruise/5.0f);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }

}
