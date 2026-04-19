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

  Roll control  (PID on errorx → nav_roll_cd):
    Tunable via parameters TRK_ROLL_P / _I / _D / _IMAX
    Default: P=200 cd/deg, I=10, D=5, imax=3000 cd

  Pitch control (PID on errory → nav_pitch_cd):
    Tunable via parameters TRK_PTCH_P / _I / _D / _IMAX
    Default: P=100 cd/deg, I=500, D=0, imax=3000 cd

  Throttle: constant TRIM_THROTTLE percent.
  Timeout:  if no message within TRACKING_TIMEOUT_MS, both PIDs are
            reset and roll/pitch commands are zeroed (hold level until
            signal returns).

  Terminal: activates when (current_alt_msl - TRK_TGT_ALT) <= TRK_TERM_ALT.
            TRK_TGT_ALT / TRK_TGT_LAT / TRK_TGT_LON store the target's MSL
            altitude (m) and position (decimal degrees).
*/



// ── _enter ────────────────────────────────────────────────────────────────────
bool ModeTracking::_enter()
{
    _errorx_rad     = 0.0f;
    _errory_rad     = 0.0f;
    _last_msg_ms    = 0;
    _prev_update_ms = AP_HAL::millis();
    _was_timed_out      = false;
    _lock_stable_ms     = 0;   // 0 = not yet acquired
    _cruise_throttle    = plane.aparm.throttle_cruise.get();
    _terminal_entry_ms  = 0;   // 0 = not yet in terminal
    _kf_x[0]        = 0.0f;
    _kf_x[1]        = 0.0f;
    _kf_P[0]        = 1.0f;  // P00
    _kf_P[1]        = 0.0f;  // P01
    _kf_P[2]        = 0.0f;  // P10
    _kf_P[3]        = 1.0f;  // P11
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
    plane.g2.tracking_throt_pid.reset_I();
    plane.g2.tracking_throt_pid.reset_filter();
    RC_Channels::clear_overrides();
    gcs().send_text(MAV_SEVERITY_INFO, "Tracking: exit");
}


// ── handle_tracking_error ─────────────────────────────────────────────────────
// Called by GCS_MAVLink_Plane::handle_tracking_message() with values already
// converted from normalised [-1,1] to radians (× TRACKING_MAX_DELTA_RAD).
void ModeTracking::handle_tracking_error(float errorx_rad, float errory_rad)
{
    _errorx_rad  = errorx_rad;
    _errory_rad  = errory_rad;
    _last_msg_ms = AP_HAL::millis();
    ::printf("TRK ex=%.4f ey=%.4f deg (ex=%.2f ey=%.2f)\n",
             errorx_rad, errory_rad,
             degrees(errorx_rad), degrees(errory_rad));
}


// ── update ────────────────────────────────────────────────────────────────────
void ModeTracking::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float    dt_s   = constrain_float((now_ms - _prev_update_ms) * 1e-3f,
                                            0.001f, 0.5f);
    _prev_update_ms = now_ms;
    float ey=0.0f;
    float ey_raw=0.0f;

    const uint32_t timeout_ms = (uint32_t)plane.g2.tracking_timeout_ms.get();
    const bool timed_out = (_last_msg_ms == 0) ||
                           (now_ms - _last_msg_ms > timeout_ms);

    // Deadband is needed in both branches.
    const float deadband_rad = plane.g2.tracking_deadband_deg.get() * (M_PI / 180.0f);

    // Horizontal distance to target using TRK_TGT_LAT / TRK_TGT_LON.
    // Mirrors seekerctrl.py _dist_to_target_m() (haversine via Location::get_distance).
    const Location target_loc(
        (int32_t)(plane.g2.tracking_target_lat.get() * 1.0e7f),
        (int32_t)(plane.g2.tracking_target_lon.get() * 1.0e7f),
        0, Location::AltFrame::ABSOLUTE);
    const float horiz_dist_m = plane.current_loc.get_distance(target_loc);
    const float close_m      = plane.g2.tracking_close_m.get();
    const bool  close_enough = (close_m > 0.0f) && (horiz_dist_m <= close_m);

    // GCS alert on close_enough edge (entry and exit).
    if (close_enough != _close_enough_prev) {
        gcs().send_text(close_enough ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO,
                        "Tracking: %.0fm %s TRK_CLOSE_M (%.0fm)",
                        (double)horiz_dist_m,
                        close_enough ? "<=" : ">",
                        (double)close_m);
        _close_enough_prev = close_enough;
    }

    // Periodic 1 Hz console log: distance, terminal and close flags, altitude.
    if (now_ms - _last_dist_log_ms >= 1000U) {
        _last_dist_log_ms = now_ms;
        ::printf("TRK dist=%.1fm close=%d alt=%.1fm\n",
                 (double)horiz_dist_m, (int)close_enough,
                 (double)(plane.current_loc.alt * 0.01f));
    }

    // Settle ramp — computed here so throttle can also use it.
    // During timeout _lock_stable_ms is cleared, so ramp stays 0 until re-acquisition.
    const float settle_s = plane.g2.tracking_settle_s.get();
    float ramp = 1.0f;
    if (!timed_out && settle_s > 0.0f && _lock_stable_ms > 0) {
        const float elapsed = constrain_float((now_ms - _lock_stable_ms) * 1e-3f,
                                              0.0f, settle_s);
        ramp = elapsed / settle_s;
    } else if (timed_out) {
        ramp = 0.0f;
    }

    if (timed_out) {
        if (!_was_timed_out) {
            // Flush integrators on first timeout cycle to prevent wind-up.
            plane.g2.tracking_roll_pid.reset_I();
            plane.g2.tracking_roll_pid.reset_filter();
            plane.g2.tracking_pitch_pid.reset_I();
            plane.g2.tracking_pitch_pid.reset_filter();
        }

        // No signal — hold wings level.
        plane.nav_roll_cd  = 0;
        plane.nav_pitch_cd = 0;

        _was_timed_out  = true;
        _lock_stable_ms = 0;   // force re-settle when signal returns
    } else {
        if (_was_timed_out) {
            // Signal just returned — start settle timer now.
            _lock_stable_ms = now_ms;
        } else if (_lock_stable_ms == 0) {
            // First acquisition after mode entry.
            _lock_stable_ms = now_ms;
        }
        _was_timed_out = false;

        // ── Roll PID ─────────────────────────────────────────────────────────
        // errorx > 0 → target is to the right → roll right (positive bank).
        // Reset I when inside deadband to prevent integrator wind-up.
        const float ex = fabsf(_errorx_rad) > deadband_rad ? _errorx_rad : 0.0f;
        if (is_zero(ex)) {
            plane.g2.tracking_roll_pid.reset_I();
            // In deadband: use gyro roll rate to actively damp any rolling
            // motion and drive toward wings-level.  Each deg/s of roll rate
            // commands an opposing 50 cdeg (0.5 deg) of bank target so the
            // attitude controller fights the rotation rather than coasting.
            const float roll_rate_dps = degrees(ahrs.get_gyro().x);
            const float damp_cd       = -(roll_rate_dps * 50.0f) * ramp;
            plane.nav_roll_cd = constrain_int32((int32_t)damp_cd,
                                                -plane.roll_limit_cd,
                                                 plane.roll_limit_cd);
        } else {
            const float roll_cd = plane.g2.tracking_roll_pid.update_all(degrees(ex), 0.0f, dt_s)
                                  * ramp;
            plane.nav_roll_cd   = constrain_int32((int32_t)roll_cd,
                                                  -plane.roll_limit_cd,
                                                   plane.roll_limit_cd);
        }

        // ── Yaw / roll mix (vtail and conventional plane only) ───────────────
        // Elevon aircraft have no separate rudder — skip yaw command entirely.
        // For vtail/plane use calc_nav_yaw_coordinated() so the attitude
        // controller adds the correct slip correction for the current bank.
        {
            const bool is_elevon = SRV_Channels::function_assigned(SRV_Channel::k_elevon_left) ||
                                   SRV_Channels::function_assigned(SRV_Channel::k_elevon_right);
            if (!is_elevon) {
                plane.calc_nav_yaw_coordinated();
            }
        }

        // ── Pitch PID ────────────────────────────────────────────────────────
        // errory already has TRK_PITCH_OFFSET subtracted on the GCS side.
        ey_raw = fabsf(_errory_rad) > deadband_rad ? _errory_rad : 0.0f;
        ey     = ey_raw;
        if (is_zero(ey_raw)) {
            plane.g2.tracking_pitch_pid.reset_I();
        }
        const float pitch_cd = plane.g2.tracking_pitch_pid.update_all(degrees(ey), 0.0f, dt_s)
                               * ramp;
        plane.nav_pitch_cd   = constrain_int32((int32_t)pitch_cd,
                                               (int32_t)(plane.pitch_limit_min * 100),
                                               plane.aparm.pitch_limit_max.get() * 100);
    }

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

        if (timed_out) {
            // Reset filter on signal loss so stale rate estimate doesn't linger.
            _kf_x[0]        = pitch_err;
            _kf_x[1]        = 0.0f;
            _kf_P[0]        = 1.0f;
            _kf_P[1]        = 0.0f;
            _kf_P[2]        = 0.0f;
            _kf_P[3]        = 1.0f;
            _kf_initialized = false;
        } else if (!_kf_initialized) {
            // First measurement — seed position; rate starts at zero.
            _kf_x[0]        = pitch_err;
            _kf_x[1]        = 0.0f;
            _kf_P[0]        = 1.0f;
            _kf_P[1]        = 0.0f;
            _kf_P[2]        = 0.0f;
            _kf_P[3]        = 1.0f;
            _kf_initialized = true;
        } else {
            // ── Predict ────────────────────────────────────────────────────
            // x_pred = F * x
            const float px0 = _kf_x[0] + _kf_x[1] * dt_s;
            const float px1 = _kf_x[1];

            // P_pred = F*P*F' + Q   (Q = diag(1e-4, q_vel))
            const float pp00 = _kf_P[0] + dt_s * (_kf_P[2] + _kf_P[1]) + dt_s * dt_s * _kf_P[3] + 1e-4f;
            const float pp01 = _kf_P[1] + dt_s * _kf_P[3];
            const float pp10 = _kf_P[2] + dt_s * _kf_P[3];
            const float pp11 = _kf_P[3] + q_vel;

            // ── Update ─────────────────────────────────────────────────────
            // Innovation: y = z - H*x_pred  (H = [1,0])
            const float innov = pitch_err - px0;
            // Innovation covariance: S = H*P*H' + R = P[0][0] + R
            const float S_inv = 1.0f / (pp00 + r_meas);
            // Kalman gain: K = P*H'/S  →  K = [pp00, pp10] / S
            const float K0 = pp00 * S_inv;
            const float K1 = pp10 * S_inv;

            // State update
            _kf_x[0] = px0 + K0 * innov;
            _kf_x[1] = px1 + K1 * innov;

            // Covariance update: P = (I - K*H)*P_pred
            _kf_P[0] = (1.0f - K0) * pp00;
            _kf_P[1] = (1.0f - K0) * pp01;
            _kf_P[2] = pp10 - K1 * pp00;
            _kf_P[3] = pp11 - K1 * pp01;
        }

        // Predicted pitch error at look-ahead horizon
        const float pitch_err_pred = _kf_x[0] + _kf_x[1] * lead_s;
        const float pid_out        = plane.g2.tracking_throt_pid.update_all(
                                         pitch_err_pred, 0.0f, dt_s) * ramp;

        const float throttle = constrain_float(cruise + pid_out, cruise / 2.0f, cruise);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }

}
