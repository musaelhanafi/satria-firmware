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

  Roll / pitch: 2-state Kalman filters (error, error-rate) run every
    cycle and feed the roll and pitch PIDs.  The KFs start at x=[0,0]
    with zero covariance on mode entry; process noise Q drives the
    covariance away from zero so the Kalman gain rises gradually and
    the filtered error converges from zero over ~1-2 s (tune TRK_KFXY_R).
    There is no timeout fallback — the filters run continuously and the
    last received error is held when no new message arrives.

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

    // Roll / pitch error KFs — seeded at zero with zero covariance so the
    // Kalman gain rises gradually and provides a natural settle ramp.
    _kf_roll_x[0]  = 0.0f; _kf_roll_x[1]  = 0.0f;
    _kf_roll_P[0]  = 0.0f; _kf_roll_P[1]  = 0.0f;
    _kf_roll_P[2]  = 0.0f; _kf_roll_P[3]  = 0.0f;
    _kf_roll_init  = false;
    _kf_pitch_x[0] = 0.0f; _kf_pitch_x[1] = 0.0f;
    _kf_pitch_P[0] = 0.0f; _kf_pitch_P[1] = 0.0f;
    _kf_pitch_P[2] = 0.0f; _kf_pitch_P[3] = 0.0f;
    _kf_pitch_init = false;

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
    ::printf("TRK ex=%.4f ey=%.4f deg (ex=%.2f ey=%.2f)\n",
             errorx_rad, errory_rad,
             degrees(errorx_rad), degrees(errory_rad));
}


// ── _kf2_update ───────────────────────────────────────────────────────────────
// 2-state Kalman filter step shared by the roll and pitch error KFs.
// State: x = [pos, rate]  Model: F = [[1,dt],[0,1]]
// Observation: z mapped to pos, H = [1,0]
// Q = diag(1e-4, q_vel),  R = r_meas
// On first call (_init == false) the state is seeded at [0,0] with P=0.
// Process noise then drives P away from zero so the Kalman gain rises
// gradually — this provides a natural ramp from zero to the true measurement.
static void _kf2_update(bool &init, float x[2], float P[4],
                        float z, float dt_s, float q_vel, float r_meas)
{
    if (!init) {
        x[0] = 0.0f; x[1] = 0.0f;
        P[0] = 0.0f; P[1] = 0.0f;
        P[2] = 0.0f; P[3] = 0.0f;
        init = true;
        return;
    }
    // Predict
    const float px0  = x[0] + x[1] * dt_s;
    const float px1  = x[1];
    const float pp00 = P[0] + dt_s * (P[2] + P[1]) + dt_s * dt_s * P[3] + 1e-4f;
    const float pp01 = P[1] + dt_s * P[3];
    const float pp10 = P[2] + dt_s * P[3];
    const float pp11 = P[3] + q_vel;
    // Update
    const float innov = z - px0;
    const float S_inv = 1.0f / (pp00 + r_meas);
    const float K0    = pp00 * S_inv;
    const float K1    = pp10 * S_inv;
    x[0] = px0 + K0 * innov;
    x[1] = px1 + K1 * innov;
    P[0] = (1.0f - K0) * pp00;
    P[1] = (1.0f - K0) * pp01;
    P[2] = pp10 - K1 * pp00;
    P[3] = pp11 - K1 * pp01;
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

    if (close_enough != _close_enough_prev) {
        gcs().send_text(close_enough ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO,
                        "Tracking: %.0fm %s TRK_CLOSE_M (%.0fm)",
                        (double)horiz_dist_m,
                        close_enough ? "<=" : ">",
                        (double)close_m);
        _close_enough_prev = close_enough;
    }

    if (now_ms - _last_dist_log_ms >= 1000U) {
        _last_dist_log_ms = now_ms;
        ::printf("TRK dist=%.1fm close=%d alt=%.1fm\n",
                 (double)horiz_dist_m, (int)close_enough,
                 (double)(plane.current_loc.alt * 0.01f));
    }

    // Throttle settle ramp — linear from 0 to 1 over TRK_SETTLE_S seconds
    // after mode entry.  Roll/pitch use the KF for their own natural ramp.
    const float settle_s = plane.g2.tracking_settle_s.get();
    const float elapsed  = constrain_float((now_ms - _lock_stable_ms) * 1e-3f,
                                           0.0f, settle_s);
    const float ramp     = (settle_s > 0.0f) ? (elapsed / settle_s) : 1.0f;

    const float kfxy_q = plane.g2.tracking_kfxy_q.get();
    const float kfxy_r = MAX(plane.g2.tracking_kfxy_r.get(), 1e-6f);

    // ── Roll KF + PID ─────────────────────────────────────────────────────────
    // errorx > 0 → target right → roll right (positive bank).
    // KF runs every cycle; measurement = 0 inside deadband so state decays
    // toward zero while wings are level.
    const float ex_raw = fabsf(_errorx_rad) > deadband_rad ? _errorx_rad : 0.0f;

    _kf2_update(_kf_roll_init, _kf_roll_x, _kf_roll_P,
                ex_raw, dt_s, kfxy_q, kfxy_r);

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
                                  degrees(_kf_roll_x[0]), 0.0f, dt_s);
        plane.nav_roll_cd   = constrain_int32((int32_t)roll_cd,
                                              -plane.roll_limit_cd,
                                               plane.roll_limit_cd);
    }

    // ── Pitch KF + PID ────────────────────────────────────────────────────────
    // TRK_PITCH_OFFSET is the setpoint: PID drives errory → pitch_offset.
    const float ey_raw = fabsf(_errory_rad) > deadband_rad ? _errory_rad : 0.0f;

    _kf2_update(_kf_pitch_init, _kf_pitch_x, _kf_pitch_P,
                ey_raw, dt_s, kfxy_q, kfxy_r);

    if (is_zero(ey_raw)) {
        plane.g2.tracking_pitch_pid.reset_I();
    }
    const float pitch_offset_deg = plane.g2.tracking_pitch_offset.get();
    const float pitch_cd = plane.g2.tracking_pitch_pid.update_all(
                               degrees(_kf_pitch_x[0]), pitch_offset_deg, dt_s);
    plane.nav_pitch_cd   = constrain_int32((int32_t)pitch_cd,
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
        const float pitch_err     = ahrs.get_pitch() - nav_pitch_rad;

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

        const float throttle = constrain_float(cruise + pid_out, cruise / 2.0f, cruise);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }

}
