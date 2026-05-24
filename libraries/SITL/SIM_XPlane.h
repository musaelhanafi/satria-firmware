/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connection for ardupilot version of Xplane
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_XPLANE_ENABLED

#include <AP_HAL/utility/Socket.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include "SIM_Aircraft.h"
#include <AP_JSON/AP_JSON.h>

namespace SITL {

/*
  a Xplane simulator
 */
class XPlane : public Aircraft {
public:
    XPlane(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW XPlane(frame_str);
    }

private:

    bool receive_data(void);
    void send_dref(const char *name, float value);
    void request_drefs(void);
    void request_dref(const char *name, uint8_t code, uint32_t rate_hz);
    void send_drefs(const struct sitl_input &input);
    void handle_rref(const uint8_t *p, uint32_t len);
    void select_data(void);
    void deselect_code(uint8_t code);
    int8_t find_data_index(uint8_t id);

    // return true if at least X
    bool is_xplane12(void) const {
        return xplane_version / 10000 >= 12;
    }
    
    const char *xplane_ip = "127.0.0.1";
    uint16_t xplane_port = 49000;
    uint16_t bind_port = 49001;
    char xplane_ip_buf[32] {};
    bool elevons = false;
    // udp socket, input and output
    SocketAPM socket_in{true};
    SocketAPM socket_out{true};

    uint64_t time_base_us;
    uint32_t last_data_time_ms;
    Vector3d position_zero;
    Vector3f accel_earth;
    bool connected = false;
    uint32_t xplane_frame_time;
    uint64_t seen_mask;
    uint8_t _recv_buf[1500];
    uint32_t last_dref_ms;
    uint32_t last_dsel_ms;

    struct {
        uint32_t last_report_ms;
        uint32_t data_count;
        uint32_t frame_count;
    } report;

    enum class DRefType {
        ANGLE = 0,
        RANGE = 1,
        FIXED = 2,
        ELEVON_AILERON  = 3,  // recovers aileron  from two elevon channels
        ELEVON_ELEVATOR = 4,  // recovers elevator from two elevon channels
        VTAIL_ELEVATOR  = 5,  // recovers elevator from two vtail channels (right+left)/2
        VTAIL_RUDDER    = 6,  // recovers rudder    from two vtail channels (left-right)/2
        RUNNING         = 7,  // outputs range when armed and PWM > min, 0 otherwise (for ENGN_running)
    };

    struct DRef {
        struct DRef *next;
        char *name;
        DRefType type;
        uint8_t channel;   // primary channel (elevon_right for elevon types)
        uint8_t channel2;  // secondary channel (elevon_left for elevon types)
        float range;
        float fixed_value;
        bool invert;            // negate value before sending (e.g. right aileron from single channel)
        float last_sent = NAN; // last value sent — skip if change < deadband
        struct DRef *pair = nullptr;      // sibling sent atomically in same round-robin slot (e.g. aileron pair)
        bool is_pair_secondary = false;   // skip in main round-robin; sent by primary's slot
    };

    // list of DRefs;
    struct DRef *drefs;
    struct DRef *dref_cursor = nullptr;  // round-robin pointer for bandwidth-limited send
    uint32_t dref_fixed_count = 0;       // counter for periodic FIXED DREF resend
    uint32_t dref_debug;

    enum class JoyType {
        AXIS = 0,
        BUTTON = 1,
    };

    // list of joystick inputs
    struct JoyInput {
        struct JoyInput *next;
        uint8_t axis;
        uint8_t channel;
        JoyType type;
        float input_min, input_max;
        uint32_t mask;
    };
    struct JoyInput *joyinputs;

    char *map_filename;
    struct stat map_st;

    bool load_dref_map(const char *map_json);
    void add_dref(const char *name, DRefType type, const AP_JSON::value &dref);
    void add_joyinput(const char *name, JoyType type, const AP_JSON::value &d);
    void handle_setting(const AP_JSON::value &d);
    void check_reload_dref(void);

    bool last_armed = false;    // tracks arm state for DREF deadband reset

    uint32_t xplane_version;

    // RREF-based gyro (Prad/Qrad/Rrad) — matches PX4xplane convention exactly.
    // Populated by handle_rref(); used by AngularVelocities case when valid.
    Vector3f rref_gyro;
    uint8_t  rref_gyro_mask  = 0;   // bit 0=P, 1=Q, 2=R; valid when ==7
    bool     rref_gyro_valid = false;

    // Gyro bias compensation. X-Plane reports nonzero Prad/Qrad/Rrad for an
    // aircraft that is visually stationary (model physics quirk; observed
    // ~-16 to +11 deg/s on the Z axis at SITL boot). We mimic real-drone
    // gyro calibration: accumulate the first N samples per axis at boot,
    // compute the mean as the bias, then subtract it from every subsequent
    // sample before publishing into the EKF / rate controller.
    //
    // Variance gate: if the std-dev of a calibration window exceeds
    // GYRO_BIAS_STDDEV_LIMIT, the aircraft was clearly moving — discard
    // the window and restart accumulation. Keeps retrying until we catch
    // a truly-stationary window. Prevents the failure mode where the SITL
    // is restarted while a previous crash is still tumbling the airframe.
    static constexpr uint16_t GYRO_BIAS_SAMPLES = 200;       // ~2.5 s at 80 Hz RREF
    static constexpr float    GYRO_BIAS_STDDEV_LIMIT = 0.05f; // rad/s ≈ 2.9°/s
    Vector3f rref_gyro_bias;
    Vector3f rref_gyro_bias_sum;
    Vector3f rref_gyro_bias_sum_sq;
    uint16_t rref_gyro_bias_count_x = 0;
    uint16_t rref_gyro_bias_count_y = 0;
    uint16_t rref_gyro_bias_count_z = 0;
    bool     rref_gyro_bias_locked_x = false;
    bool     rref_gyro_bias_locked_y = false;
    bool     rref_gyro_bias_locked_z = false;
    bool     rref_gyro_bias_reported = false;
    uint32_t rref_gyro_bias_rejects  = 0;   // count of discarded windows

    // Accelerometer magnitude calibration. Mirrors the px4xplane plugin's
    // AccelCalibration: X-Plane aircraft models can report |accel| ≠ 1g when
    // stationary. Fix is MAGNITUDE SCALING — compute scale = g / measured_|a|
    // while stationary, then multiply every subsequent accel sample.
    // Magnitude-scale preserves direction so works at all attitudes (offset
    // subtraction would only be correct at the calibration attitude).
    static constexpr uint16_t ACCEL_CAL_SAMPLES        = 200;
    static constexpr uint16_t ACCEL_CAL_WAIT_SAMPLES   = 50;
    static constexpr float    ACCEL_CAL_STATIONARY_VEL = 0.5f;
    float    accel_scale_factor = 1.0f;
    float    accel_cal_sum_mag  = 0.f;
    uint16_t accel_cal_count    = 0;
    uint16_t accel_cal_stationary_count = 0;
    bool     accel_calibrated   = false;

    // IIR low-pass filter on IMU samples. X-Plane physics produces high-freq
    // gyro/accel jitter (we measured σ≈17°/s on stationary models) that gets
    // republished raw without smoothing → EKF instability → crashes on
    // takeoff. The px4xplane plugin uses a Kalman tracker for this; we use
    // a simpler 1st-order IIR. α=0.8 → ~11 ms lag at 80 Hz RREF, snappy
    // enough to preserve fast attitude dynamics while killing jitter.
    static constexpr float SENSOR_LPF_ALPHA = 0.8f;
    Vector3f rref_gyro_filt;
    Vector3f rref_accel_filt;
    bool     sensor_filt_initialized = false;

    // RREF-based accel (g_axil/g_side/g_nrml) — matches PX4xplane convention exactly.
    // Populated by handle_rref(); used by Gload case when valid.
    Vector3f rref_accel;
    uint8_t  rref_accel_mask  = 0;  // bit 0=axil, 1=side, 2=nrml; valid when ==7
    bool     rref_accel_valid = false;
};


} // namespace SITL


#endif  // AP_SIM_XPLANE_ENABLED
