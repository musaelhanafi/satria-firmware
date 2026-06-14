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
  simulator connector for XPlane
*/

#include "SIM_config.h"

#if AP_SIM_XPLANE_ENABLED

#include "SIM_XPlane.h"
#include "SITL.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>   // clock_gettime(CLOCK_MONOTONIC) for wall-clock pacing in failed:

#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

// ignore cast errors in this case to keep complexity down
#pragma GCC diagnostic ignored "-Wcast-align"

extern const AP_HAL::HAL& hal;

#ifndef XPLANE_JSON
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define XPLANE_JSON "xplane_quad.json"
#elif APM_BUILD_TYPE(APM_BUILD_Heli)
#define XPLANE_JSON "xplane_heli.json"
#else
#define XPLANE_JSON "xplane_plane.json"
#endif
#endif // XPLANE_JSON
#define XPLANE_JSON_ELEVON   "xplane_elevon.json"
#define XPLANE_JSON_VTAIL    "xplane_vtail.json"
#define XPLANE_JSON_VTAILVTOL "xplane_vtail_vtol.json"
#define XPLANE_JSON_QUAD_JOY "xplane_quad_joy.json"

// DATA@ frame types. Thanks to TauLabs xplanesimulator.h
// (which strangely enough acknowledges APM as a source!)
enum {
    FramRate            = 0,
    Times               = 1,
    SimStats            = 2,
    Speed               = 3,
    Gload               = 4,
    AtmosphereWeather   = 5,
    AtmosphereAircraft  = 6,
    SystemPressures     = 7,
    Joystick1           = 8,
    Joystick2           = 9,
    ArtStab             = 10,
    FlightCon           = 11,
    WingSweep           = 12,
    Trim                = 13,
    Brakes              = 14,
    AngularMoments      = 15,
    AngularVelocities   = 16,
    PitchRollHeading    = 17,
    AoA                 = 18,
    MagCompass          = 19,
    LatLonAlt           = 20,
    LocVelDistTraveled  = 21,
    ThrottleCommand     = 25,
    CarbHeat            = 30,
    EngineRPM           = 37,
    PropRPM             = 38,
    PropPitch           = 39,
    Generator           = 58,
    JoystickRaw         = 136,
};

enum RREF {
    RREF_VERSION = 1,
    RREF_PRAD    = 2,   // sim/flightmodel/position/Prad    — roll  rate (rad/s)
    RREF_QRAD    = 3,   // sim/flightmodel/position/Qrad    — pitch rate (rad/s)
    RREF_RRAD    = 4,   // sim/flightmodel/position/Rrad    — yaw   rate (rad/s)
    RREF_GAXIL   = 5,   // sim/flightmodel/forces/g_axil    — axial G (forward, Gs)
    RREF_GSIDE   = 6,   // sim/flightmodel/forces/g_side    — lateral G (right, Gs)
    RREF_GNRML   = 7,   // sim/flightmodel/forces/g_nrml    — normal G (up, Gs)
    // Position / velocity / attitude / airspeed via RREF — matches PX4's
    // SimulatorXPlane, which sources ALL state over RREF. Replaces the fragile
    // DATA@/DSEL rows (slot-offset ambiguity + dependence on X-Plane's per-user
    // "Data Output" config) that were destabilising ArduCopter's EKF.
    RREF_LX      = 8,   // sim/flightmodel/position/local_x   (OpenGL east,  m)
    RREF_LY      = 9,   // sim/flightmodel/position/local_y   (OpenGL up,    m)
    RREF_LZ      = 10,  // sim/flightmodel/position/local_z   (OpenGL south, m)
    RREF_VX      = 11,  // sim/flightmodel/position/local_vx  (east,  m/s)
    RREF_VY      = 12,  // sim/flightmodel/position/local_vy  (up,    m/s)
    RREF_VZ      = 13,  // sim/flightmodel/position/local_vz  (south, m/s)
    RREF_LAT     = 14,  // sim/flightmodel/position/latitude  (deg)
    RREF_LON     = 15,  // sim/flightmodel/position/longitude (deg)
    RREF_ELEV    = 16,  // sim/flightmodel/position/elevation (m MSL)
    RREF_THETA   = 17,  // sim/flightmodel/position/theta     (pitch, deg)
    RREF_PHI     = 18,  // sim/flightmodel/position/phi       (roll,  deg)
    RREF_PSI     = 19,  // sim/flightmodel/position/psi       (true heading, deg)
    RREF_TAS     = 20,  // sim/flightmodel/position/true_airspeed (m/s)
};

static const uint8_t required_data[] {
        Times, LatLonAlt, Speed, PitchRollHeading,
        LocVelDistTraveled, AngularVelocities, Gload,
        Trim,
        PropPitch, EngineRPM, PropRPM,
        JoystickRaw };

using namespace SITL;

XPlane::XPlane(const char *frame_str) :
    Aircraft(frame_str)
{
    use_time_sync = false;
#if defined(AP_SIM_XPLANE_ELEVON)
    elevons = true;
#else
    if (strstr(frame_str, "-elevon")) {
        elevons = true;
    }
#endif
    const char *colon = strchr(frame_str, ':');
    if (colon) {
        const char *ip_start = colon + 1;
        const char *colon2 = strchr(ip_start, ':');
        if (colon2) {
            size_t ip_len = colon2 - ip_start;
            if (ip_len >= sizeof(xplane_ip_buf)) {
                ip_len = sizeof(xplane_ip_buf) - 1;
            }
            memcpy(xplane_ip_buf, ip_start, ip_len);
            xplane_ip_buf[ip_len] = '\0';
            xplane_ip = xplane_ip_buf;
            bind_port = atoi(colon2 + 1);
        } else {
            xplane_ip = ip_start;
        }
    }

    // SIM_XP_BIND_PORT param overrides the port from the frame string (or
    // the compiled-in default of 49001) when non-zero.
    {
        auto *_sitl = AP::sitl();
        if (_sitl != nullptr && _sitl->xplane_bind_port > 0) {
            bind_port = uint16_t(_sitl->xplane_bind_port.get());
        }
    }

    socket_in.bind("0.0.0.0", bind_port);
    // Connect socket_out now so select_data() can send DSEL before the first
    // DATA packet arrives (xplane_ip is known from the frame string).
    socket_out.connect(xplane_ip, xplane_port);
    printf("Waiting for XPlane data on UDP port %u and sending to port %u\n",
           (unsigned)bind_port, (unsigned)xplane_port);

    // XPlane sensor data is not good enough for EKF. Use fake EKF by default
    AP_Param::set_default_by_name("AHRS_EKF_TYPE", 10);
    AP_Param::set_default_by_name("GPS1_TYPE", 100);
    AP_Param::set_default_by_name("INS_GYR_CAL", 0);

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // default flaps to channel 5
    AP_Param::set_default_by_name("SERVO5_FUNCTION", 3);
    AP_Param::set_default_by_name("SERVO5_MIN", 1000);
    AP_Param::set_default_by_name("SERVO5_MAX", 2000);
#endif

    const bool vtail = strstr(frame_str, "-vtail") != nullptr;
    const bool vtol  = strstr(frame_str, "-vtol")  != nullptr;
    const bool joy   = strstr(frame_str, "-joy")   != nullptr;

    const char *xplane_json;
    if (vtail && vtol) {
        xplane_json = XPLANE_JSON_VTAILVTOL;
    } else if (vtail) {
        xplane_json = XPLANE_JSON_VTAIL;
    } else if (elevons) {
        xplane_json = XPLANE_JSON_ELEVON;
    } else if (joy) {
        xplane_json = XPLANE_JSON_QUAD_JOY;
    } else {
        xplane_json = XPLANE_JSON;
    }

    if (!load_dref_map(xplane_json)) {
        AP_HAL::panic("%s failed to load", xplane_json);
    }
}

/*
  add one DRef to list
 */
void XPlane::add_dref(const char *name, DRefType type, const AP_JSON::value &dref)
{
    struct DRef *d = NEW_NOTHROW struct DRef;
    if (d == nullptr) {
        AP_HAL::panic("out of memory for DRef %s", name);
    }
    d->name = strdup(name);
    d->type = type;
    if (d->name == nullptr) {
        AP_HAL::panic("out of memory for DRef %s", name);
    }
    if (d->type == DRefType::FIXED) {
        d->fixed_value = dref.get("value").get<double>();
    } else {
        d->range = dref.get("range").get<double>();
        d->invert = dref.contains("invert") && dref.get("invert").get<bool>();
        const bool is_quad_type = (d->type == DRefType::QUAD_ROLL     ||
                                   d->type == DRefType::QUAD_PITCH    ||
                                   d->type == DRefType::QUAD_THROTTLE ||
                                   d->type == DRefType::QUAD_YAW);
        if (!is_quad_type) {
            d->channel = dref.get("channel").get<double>();
            if (d->type == DRefType::ELEVON_AILERON || d->type == DRefType::ELEVON_ELEVATOR ||
                d->type == DRefType::VTAIL_ELEVATOR  || d->type == DRefType::VTAIL_RUDDER) {
                d->channel2 = dref.get("channel2").get<double>();
            }
        }
    }
    // add to linked list
    d->next = drefs;
    drefs = d;
}

/*
  add one joystick axis to list
 */
void XPlane::add_joyinput(const char *label, JoyType type, const AP_JSON::value &d)
{
    if (strncmp(label, "axis", 4) == 0) {
        struct JoyInput *j = NEW_NOTHROW struct JoyInput;
        if (j == nullptr) {
            AP_HAL::panic("out of memory for JoyInput %s", label);
        }
        j->axis = atoi(label+4);
        j->type = JoyType::AXIS;
        j->channel = d.get("channel").get<double>();
        j->input_min = d.get("input_min").get<double>();
        j->input_max = d.get("input_max").get<double>();
        j->next = joyinputs;
        joyinputs = j;
    }
    if (strncmp(label, "button", 6) == 0) {
        struct JoyInput *j = NEW_NOTHROW struct JoyInput;
        if (j == nullptr) {
            AP_HAL::panic("out of memory for JoyInput %s", label);
        }
        j->type = JoyType::BUTTON;
        j->channel = d.get("channel").get<double>();
        j->mask = d.get("mask").get<double>();
        j->next = joyinputs;
        joyinputs = j;
    }
}

/*
  handle a setting
 */
void XPlane::handle_setting(const AP_JSON::value &d)
{
    if (d.contains("debug")) {
        dref_debug = d.get("debug").get<double>();
    }
}


/*
  load mapping of channels to datarefs from a json file
 */
bool XPlane::load_dref_map(const char *map_json)
{
    char *fname = nullptr;
    if (AP::FS().stat(map_json, &map_st) == 0) {
        fname = strdup(map_json);
    } else {
        IGNORE_RETURN(asprintf(&fname, "@ROMFS/models/%s", map_json));
        if (AP::FS().stat(fname, &map_st) != 0) {
            return false;
        }
    }
    if (fname == nullptr) {
        return false;
    }
    AP_JSON::value *obj = AP_JSON::load_json(fname);
    if (obj == nullptr) {
        free((void*)fname);
        return false;
    }

    free(map_filename);
    map_filename = fname;

    // free old drefs
    while (drefs) {
        auto *d = drefs->next;
        free(drefs->name);
        delete drefs;
        drefs = d;
    }

    // free old joystick
    while (joyinputs) {
        auto *j = joyinputs->next;
        delete joyinputs;
        joyinputs = j;
    }
    
    uint32_t count = 0;
    // obtain a const reference to the map, and print the contents
    const AP_JSON::value::object& o = obj->get<AP_JSON::value::object>();
    for (AP_JSON::value::object::const_iterator i = o.begin();
         i != o.end();
         ++i) {
        const char *label = i->first.c_str();
        const auto &d = i->second;
        if (strchr(label, '/') != nullptr) {
            const auto str = d.get("type").to_str();
            const char *type_s = str.c_str();
            if (strcmp(type_s, "angle") == 0) {
                add_dref(label, DRefType::ANGLE, d);
            } else if (strcmp(type_s, "range") == 0) {
                add_dref(label, DRefType::RANGE, d);
            } else if (strcmp(type_s, "fixed") == 0) {
                add_dref(label, DRefType::FIXED, d);
            } else if (strcmp(type_s, "elevon_aileron") == 0) {
                add_dref(label, DRefType::ELEVON_AILERON, d);
            } else if (strcmp(type_s, "elevon_elevator") == 0) {
                add_dref(label, DRefType::ELEVON_ELEVATOR, d);
            } else if (strcmp(type_s, "vtail_elevator") == 0) {
                add_dref(label, DRefType::VTAIL_ELEVATOR, d);
            } else if (strcmp(type_s, "vtail_rudder") == 0) {
                add_dref(label, DRefType::VTAIL_RUDDER, d);
            } else if (strcmp(type_s, "running") == 0) {
                add_dref(label, DRefType::RUNNING, d);
            } else if (strcmp(type_s, "quad_roll") == 0) {
                add_dref(label, DRefType::QUAD_ROLL, d);
            } else if (strcmp(type_s, "quad_pitch") == 0) {
                add_dref(label, DRefType::QUAD_PITCH, d);
            } else if (strcmp(type_s, "quad_throttle") == 0) {
                add_dref(label, DRefType::QUAD_THROTTLE, d);
            } else if (strcmp(type_s, "quad_yaw") == 0) {
                add_dref(label, DRefType::QUAD_YAW, d);
            } else {
                ::printf("Invalid dref type %s for %s in %s", type_s, label, map_filename);
            }
        } else if (strcmp(label, "settings") == 0) {
            handle_setting(d);
        } else if (strncmp(label, "axis", 4) == 0) {
            add_joyinput(label, JoyType::AXIS, d);
        } else if (strncmp(label, "button", 6) == 0) {
            add_joyinput(label, JoyType::BUTTON, d);
        } else {
            ::printf("Invalid json type %s in %s", label, map_json);
            continue;
        }
        count++;
    }
    delete obj;

    // Link ANGLE DREFs that share the same channel as atomic pairs (e.g. wing1r + wing1l).
    // Two ANGLE DREFs on the same channel with opposite invert flags must be sent together so
    // that X-Plane sees both aileron surfaces update in the same simulation frame.
    // The first one encountered becomes the primary; the second becomes the secondary and is
    // skipped in the main round-robin — it is sent by the primary's slot instead.
    for (auto *d = drefs; d; d = d->next) {
        if (d->type != DRefType::ANGLE || d->is_pair_secondary || d->pair != nullptr) {
            continue;
        }
        for (auto *d2 = d->next; d2; d2 = d2->next) {
            if (d2->type == DRefType::ANGLE &&
                d2->channel == d->channel &&
                d2->invert != d->invert &&
                d2->pair == nullptr) {
                d->pair = d2;
                d2->pair = d;
                d2->is_pair_secondary = true;
                break;
            }
        }
    }
    // reset round-robin cursor whenever the map is reloaded
    dref_cursor = nullptr;

    ::printf("Loaded %u DRefs from %s\n", unsigned(count), map_filename);
    return true;
}

/*
  load mapping of channels to datarefs from a json file
 */
void XPlane::check_reload_dref(void)
{
    if (!hal.util->get_soft_armed()) {
        struct stat st;
        if (AP::FS().stat(map_filename, &st) == 0 && st.st_mtime != map_st.st_mtime) {
            load_dref_map(map_filename);
        }
    }
}

int8_t XPlane::find_data_index(uint8_t code)
{
    for (uint8_t i = 0; i<ARRAY_SIZE(required_data); i++) {
        if (required_data[i] == code) {
            return i;
        }
    }
    return -1;
}

/*
 change what data is requested from XPlane. This saves the user from
 having to setup the data screen correctly
 */
void XPlane::select_data(void)
{
    const uint64_t all_mask = (1U<<ARRAY_SIZE(required_data))-1;
    if ((seen_mask & all_mask) == all_mask) {
        // got it all
        return;
    }
    // Throttle to 1 Hz — avoid flooding X-Plane with repeated DSEL packets
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_dsel_ms < 1000) {
        return;
    }
    last_dsel_ms = now_ms;

    struct PACKED {
        uint8_t  marker[5] { 'D', 'S', 'E', 'L', '0' };
        uint32_t data[ARRAY_SIZE(required_data)] {};
    } dsel;
    uint8_t count = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(required_data); i++) {
        if (seen_mask & (1U<<i)) {
            // got this one
            continue;
        }
        dsel.data[count++] = required_data[i];
    }
    if (count != 0) {
        socket_out.send(&dsel, sizeof(dsel));
        printf("Selecting %u data types (waiting for X-Plane DATA stream)\n", (unsigned)count);
    }
}

void XPlane::deselect_code(uint8_t code)
{
    struct PACKED {
        uint8_t  marker[5] { 'U', 'S', 'E', 'L', '0' };
        uint32_t data[8] {};
    } usel;
    usel.data[0] = code;
    socket_out.send(&usel, sizeof(usel));
    printf("De-selecting code %u\n", code);
}

/*
  receive data from X-Plane via UDP
  return true if we get a gyro frame
*/
bool XPlane::receive_data(void)
{
    uint8_t *pkt = _recv_buf;
    uint8_t *p = &pkt[5];
    const uint8_t pkt_len = 36;
    Location loc {};
    Vector3d pos;
    uint32_t wait_time_ms = 1;
    uint32_t now = AP_HAL::millis();
    bool ret = false;

    // if we are about to get another frame from X-Plane then wait longer
    if (xplane_frame_time > wait_time_ms &&
        now+1 >= last_data_time_ms + xplane_frame_time) {
        wait_time_ms = 10;
    }
    ssize_t len = socket_in.recv(pkt, sizeof(_recv_buf), wait_time_ms);
    
    if (len < 5) {
        // bad packet
        goto failed;
    }

    if (memcmp(pkt, "RREF", 4) == 0) {
        handle_rref(pkt, len);
        return false;
    }

    if (memcmp(pkt, "DATA", 4) != 0) {
        // not a data packet we understand
        ::printf("PACKET: %4.4s\n", (const char *)pkt);
        goto failed;
    }
    len -= 5;

    if (len < pkt_len) {
        // bad packet
        goto failed;
    }

    
    if (!connected) {
        // we now know the IP X-Plane is using
        uint16_t port;
        socket_in.last_recv_address(xplane_ip, port);
        socket_out.connect(xplane_ip, xplane_port);
        connected = true;
        printf("Connected to %s:%u\n", xplane_ip, (unsigned)xplane_port);
    }
    
    while (len >= pkt_len) {
        // p is at offset 5 from the buffer (after "DATA\0" header).
        // On Cortex-M4, VLDR requires 4-byte alignment; casting uint8_t* to float*
        // at an unaligned address causes a HardFault.  Use memcpy to copy the
        // 36-byte row into an aligned local array before accessing as floats.
        float data[9];
        memcpy(data, p, 36);
        uint8_t code = p[0];  // byte access — always safe
        int8_t idx = find_data_index(code);
        if (idx == -1) {
            deselect_code(code);
            len -= pkt_len;
            p += pkt_len;
            continue;
        }
        seen_mask |= (1U<<idx);

        switch (code) {
        case Times: {
            uint64_t tus = data[3] * 1.0e6f;
            if (tus + time_base_us <= time_now_us) {
                uint64_t tdiff = time_now_us - (tus + time_base_us);
                if (tdiff > 1e6f) {
                    printf("X-Plane time reset %lu\n", (unsigned long)tdiff);
                }
                time_base_us = time_now_us - tus;
            }
            uint64_t tnew = time_base_us + tus;
            //uint64_t dt = tnew - time_now_us;
            //printf("dt %u\n", (unsigned)dt);
            time_now_us = tnew;
            break;
        }
            
        case LatLonAlt: {
            // sanity-check: reject NaN/Inf and out-of-range lat/lon that
            // X-Plane can send during a simulation reset, which would later
            // cause a SIGFPE in the navigation math
            if (!isfinite(data[1]) || !isfinite(data[2]) || !isfinite(data[3]) ||
                fabsf(data[1]) > 90.0f || fabsf(data[2]) > 180.0f) {
                printf("X-Plane bad LatLonAlt lat=%.2f lon=%.2f alt=%.2f — skipping\n",
                       data[1], data[2], data[3]);
                goto failed;
            }
            loc.lat = data[1] * 1e7;
            loc.lng = data[2] * 1e7;
            loc.alt = data[3] * FEET_TO_METERS * 100.0f;
            const float altitude_above_ground = data[4] * FEET_TO_METERS;
            ground_level = loc.alt * 0.01f - altitude_above_ground;
            break;
        }

        case Speed:
            airspeed = data[2] * KNOTS_TO_METERS_PER_SECOND;
            airspeed_pitot = airspeed;
            break;

        case AoA:
            // ignored
            break;

        case PitchRollHeading: {
            float roll, pitch, yaw;
            pitch = radians(data[1]);
            roll = radians(data[2]);
            yaw = radians(data[3]);
            dcm.from_euler(roll, pitch, yaw);
            break;
        }

        case AtmosphereWeather:
            // ignored
            break;

        case LocVelDistTraveled:
            pos.y = data[1];
            pos.z = -data[2];
            pos.x = -data[3];
            velocity_ef.y = data[4];
            velocity_ef.z = -data[5];
            velocity_ef.x = -data[6];
            break;

        case AngularVelocities:
            if (rref_gyro_valid) {
                // Use DREF Prad/Qrad/Rrad — same source PX4xplane uses.
                // rref_gyro is already bias-compensated in handle_rref().
                Vector3f gyro_raw = rref_gyro;
                // IIR low-pass to smooth X-Plane physics jitter that would
                // otherwise destabilize the EKF / rate controller.
                if (!sensor_filt_initialized) {
                    rref_gyro_filt = gyro_raw;
                } else {
                    rref_gyro_filt = gyro_raw * SENSOR_LPF_ALPHA
                                   + rref_gyro_filt * (1.0f - SENSOR_LPF_ALPHA);
                }
                gyro = rref_gyro_filt;
            } else if (is_xplane12()) {
                gyro.x = radians(data[1]);
                gyro.y = radians(data[2]);
                gyro.z = radians(data[3]);
            } else {
                // xplane 11
                gyro.x = data[2];
                gyro.y = data[1];
                gyro.z = data[3];
            }
            // Hold gyro at zero pre-arm. ArduCopter's boot gyro cal needs
            // consecutive 250 ms windowed averages to agree within 0.1°/s
            // ([AP_InertialSensor.cpp GYRO_INIT_MAX_DIFF_DPS=0.1f]). Even
            // after the bridge's per-axis bias lock, the IIR-filtered
            // residual jitter from X-Plane physics is ~0.2–0.3°/s RMS —
            // comfortably above the cal threshold, so AC's cal still fails
            // ("Gyros not calibrated", diff > 0.1°/s). The vehicle is
            // physically stationary from boot until arm, so the *true* gyro
            // is zero throughout; publishing exactly zero lets AC's cal
            // converge trivially with offset = 0. The moment we arm, the
            // already-settled IIR (which kept tracking real gyro internally)
            // flows live to the rate controller — no transient.
            if (!hal.util->get_soft_armed()) {
                gyro.zero();
            }
            // we only count gyro data towards data counts
            ret = true;
            break;

        case Gload:
            if (rref_accel_valid) {
                // Use g_axil/g_side/g_nrml DREFs — same sources PX4xplane uses.
                // Sign convention: specific force = -(gravity component along axis).
                // g_axil>0 when nose-down → accel_body.x should be negative → negate.
                // g_nrml>0 when lift up   → accel_body.z should be -g at rest → negate.
                Vector3f accel_raw;
                accel_raw.x = -rref_accel.x * GRAVITY_MSS;
                accel_raw.y =  rref_accel.y * GRAVITY_MSS;
                accel_raw.z = -rref_accel.z * GRAVITY_MSS;

                // IIR low-pass (same α as gyro).
                if (!sensor_filt_initialized) {
                    rref_accel_filt = accel_raw;
                    sensor_filt_initialized = true;
                } else {
                    rref_accel_filt = accel_raw * SENSOR_LPF_ALPHA
                                    + rref_accel_filt * (1.0f - SENSOR_LPF_ALPHA);
                }

                // Accel magnitude scaling (ported from PX4 SimulatorXPlane,
                // publish_imu / AccelCalibration). The X-Plane quad reports
                // |a| ≈ 7.74 m/s² stationary, not 9.81 — a ~21% deficit. This
                // is a SCALE error, not a bias: it grows with true acceleration
                // and exceeds EKF3's accel-bias clamp (~1 m/s²), so EKF3 cannot
                // absorb it. Fed raw it corrupts the gravity/tilt and vertical
                // velocity estimates → vehicle flips on takeoff (AngErr≈180).
                // PX4 avoids this by normalising |a| to 1 g; do the same here.
                // Measure the stationary magnitude once while disarmed (vehicle
                // is on the ground = stationary), then rescale every sample.
                if (!accel_calibrated && !hal.util->get_soft_armed()) {
                    if (accel_cal_stationary_count < ACCEL_CAL_WAIT_SAMPLES) {
                        accel_cal_stationary_count++;   // let the IIR settle first
                    } else {
                        accel_cal_sum_mag += rref_accel_filt.length();
                        accel_cal_count++;
                        if (accel_cal_count >= ACCEL_CAL_SAMPLES) {
                            const float measured = accel_cal_sum_mag / accel_cal_count;
                            if (measured > 0.1f) {
                                accel_scale_factor = GRAVITY_MSS / measured;
                                accel_calibrated = true;
                                printf("X-Plane accel calibrated: |g|=%.4f m/s2 scale=%.4f (%.2f%%)\n",
                                       measured, accel_scale_factor,
                                       (accel_scale_factor - 1.0f) * 100.0f);
                            }
                        }
                    }
                }

                // Normalise magnitude to 1 g (scale=1.0 until cal completes).
                accel_body = rref_accel_filt * accel_scale_factor;
            } else {
                // Fallback (DATA@ Gload group, no RREF yet): same FRD mapping
                // as the RREF path above so the EKF sees the same gravity
                // vector on either path.
                //   data[5] = g_axil  (longitudinal, +nose-fwd)
                //   data[6] = g_side  (lateral,      +right wing)
                //   data[7] = g_nrml  (normal,       +canopy-up)
                // Sign convention (specific force = -gravity in body frame):
                //   g_axil>0 (nose-down) → accel.x should be negative → negate
                //   g_side>0 (slip-right) → accel.y stays positive
                //   g_nrml>0 (lift up)    → accel.z should be -g at rest → negate
                // Previously this branch had the components permuted
                // (.z = -d[5], .x = d[6], .y = d[7]) which puts gravity on
                // body-Y instead of body-Z on a level airframe and breaks the
                // EKF attitude solution on HIL when RREF accel hasn't arrived.
                accel_body.x = -data[5] * GRAVITY_MSS;
                accel_body.y =  data[6] * GRAVITY_MSS;
                accel_body.z = -data[7] * GRAVITY_MSS;
            }
            // Hold accel at the stationary specific-force vector pre-arm:
            //   FRD body, level → accel = (0, 0, −g).
            // Same rationale as the gyro gate: ArduCopter's gyro cal aborts a
            // 250 ms window when |Δaccel| > 0.2 m/s², and X-Plane's raw accel
            // jitter (plus SITL noise injection) easily breaches that
            // threshold. Pre-arm we freeze accel exactly — every cal window
            // sees |Δaccel| ≈ 0 → cal completes on the first iteration.
            // Once armed, the bridge's filtered/calibrated accel flows live
            // for use by the EKF + rate controller.
            if (!hal.util->get_soft_armed()) {
                accel_body.x = 0.f;
                accel_body.y = 0.f;
                accel_body.z = -GRAVITY_MSS;
            }
            break;

        case PropPitch: {
            break;
        }

        case EngineRPM:
            rpm[0] = data[1];
            motor_mask |= 1;
            break;

        case PropRPM:
            rpm[1] = data[1];
            motor_mask |= 2;
            break;
            
        case JoystickRaw: {
            for (auto *j = joyinputs; j; j=j->next) {
                switch (j->type) {
                case JoyType::AXIS: {
                    if (j->axis >= 1 && j->axis <= 6) {
                        float v = (data[j->axis] - j->input_min) / (j->input_max - j->input_min);
                        rcin[j->channel-1] = v;
                        rcin_chan_count = MAX(rcin_chan_count, j->channel);
                    }
                    break;
                }
                case JoyType::BUTTON: {
                    uint32_t m = uint32_t(data[7]) & j->mask;
                    float v = 0;
                    if (m == 0) {
                        v = 0;
                    } else if (1U<<(__builtin_ffs(j->mask)-1) != m) {
                        v = 0.5;
                    } else {
                        v = 1;
                    }
                    rcin[j->channel-1] = v;
                    rcin_chan_count = MAX(rcin_chan_count, j->channel);
                    break;
                }
                }
            }
        }
        }
        len -= pkt_len;
        p += pkt_len;
    }

    // update data selection
    select_data();

    // Prefer RREF-sourced state over the DATA@ rows. RREF is self-subscribed and
    // immune to X-Plane "Data Output" slot/row ambiguity, so once each group is
    // valid it is authoritative; the DATA@ rows remain only as a startup fallback
    // (before RREF replies arrive) and to drive the frame/time tick. This is the
    // PX4 SimulatorXPlane model: all FDM state comes from RREF.
    if (rref_pos_valid) { pos = rref_pos_ned; }
    if (rref_vel_valid) { velocity_ef = rref_vel_ned; }
    if (rref_att_valid) { dcm.from_euler(rref_roll_rad, rref_pitch_rad, rref_yaw_rad); }
    if (rref_geo_valid) {
        loc.lat = rref_lat_deg * 1e7;
        loc.lng = rref_lon_deg * 1e7;
        loc.alt = rref_elev_m * 100.0f;
    }
    if (rref_airspeed_valid) {
        airspeed = rref_airspeed_mps;
        airspeed_pitot = airspeed;
    }

    position = pos + position_zero;
    position.xy() += origin.get_distance_NE_double(home);
    update_position();
    time_advance();

    accel_earth = dcm * accel_body;
    accel_earth.z += GRAVITY_MSS;
    
    // the position may slowly deviate due to float accuracy and longitude scaling
    if (loc.get_distance(location) > 4 || abs(loc.alt - location.alt)*0.01f > 2.0f) {
        const float reset_dist = loc.get_distance(location);
        // guard against X-Plane sending garbage position during sim reset
        if (reset_dist > 1e6f) {
            printf("X-Plane home reset rejected: dist=%.1f m — likely sim reset garbage\n", reset_dist);
            goto failed;
        }
        printf("X-Plane home reset dist=%f alt=%.1f/%.1f\n",
               reset_dist, loc.alt*0.01f, location.alt*0.01f);
        // reset home location
        position_zero = {-pos.x, -pos.y, -pos.z};
        home.lat = loc.lat;
        home.lng = loc.lng;
        home.alt = loc.alt;
        origin = home;
        position.x = 0;
        position.y = 0;
        position.z = 0;
        update_position();
        time_advance();
    }

    update_mag_field_bf();

    if (now > last_data_time_ms && now - last_data_time_ms < 100) {
        xplane_frame_time = now - last_data_time_ms;
    }
    last_data_time_ms = AP_HAL::millis();

    if (ret) {
        report.data_count++;
        report.frame_count++;
    }
    
    return ret;
        
failed:
    // True-disconnect cap. Wall-pacing of time advance is now handled in
    // update() via wall_paced_time_advance(), which runs every tick regardless
    // of which packet path receive_data took. This block stays just to do the
    // dead-reckoning sensor extrapolation when no fresh packet arrived.
    if (AP_HAL::millis() - last_data_time_ms > 30000) {
        return false;
    }
    extrapolate_sensors(0.001f);
    return false;
}

/*
  Advance simulated time by ≤1 ms wall-clock-paced, applying the latest
  RREF-sourced state. Called from update() every tick so the simulated clock
  tracks real time even when the bridge only receives RREF packets (which
  return early from receive_data without touching time) or none at all. This
  un-bottlenecks any SITL sensor that gates on AP_HAL::millis() — most
  importantly the simulated GPS, whose is_healthy() check fails when the
  observed average inter-arrival exceeds 215 ms.
*/
void XPlane::wall_paced_time_advance(void)
{
    // Real wall-clock reference for pacing. On SITL, AP_HAL::micros64() returns
    // *simulated* time — using it here would make the pacing circular — so read
    // the host's monotonic clock directly. On a real board (e.g. fmuv3-hil)
    // AP_HAL::micros64() *is* the hardware monotonic clock, and ChibiOS's
    // <time.h> does not define CLOCK_MONOTONIC.
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    const uint64_t wall_now_us = uint64_t(tp.tv_sec) * 1000000ULL
                               + uint64_t(tp.tv_nsec) / 1000ULL;
#else
    const uint64_t wall_now_us = AP_HAL::micros64();
#endif
    if (last_wall_extrap_us != 0 && wall_now_us - last_wall_extrap_us < 1000) {
        // less than 1 ms of real time since the last advance — wait
        return;
    }
    last_wall_extrap_us = wall_now_us;

    // --- Apply RREF gyro to FDM `gyro` (mirrors AngularVelocities switch case) ---
    // Previously the FDM gyro was only updated on DATA@ AngularVelocities row
    // arrival (~2 Hz from X-Plane's DSEL), even though RREF Prad/Qrad/Rrad
    // arrives at 100 Hz. ArduCopter's 200 Hz rate controller reading 2 Hz gyro
    // is the prime suspect for the yaw runaway: by the time the controller
    // sees real angular velocity, it has already commanded several rounds of
    // correction based on stale data → positive feedback. Updating gyro every
    // wall-ms closes that loop.
    if (rref_gyro_valid) {
        Vector3f gyro_raw = rref_gyro;   // bias-compensated in handle_rref()
        if (!sensor_filt_initialized) {
            rref_gyro_filt = gyro_raw;
        } else {
            rref_gyro_filt = gyro_raw * SENSOR_LPF_ALPHA
                           + rref_gyro_filt * (1.0f - SENSOR_LPF_ALPHA);
        }
        gyro = rref_gyro_filt;
    }
    if (!hal.util->get_soft_armed()) {
        // pre-arm gyro hold (matches the switch case)
        gyro.zero();
    }

    // --- Apply RREF accel to FDM `accel_body` (mirrors Gload switch case) ---
    // Same staleness problem as gyro. Also runs the PX4-ported stationary
    // magnitude calibration that normalises |a| ≈ 7.74 → 9.81 m/s².
    if (rref_accel_valid) {
        Vector3f accel_raw;
        accel_raw.x = -rref_accel.x * GRAVITY_MSS;
        accel_raw.y =  rref_accel.y * GRAVITY_MSS;
        accel_raw.z = -rref_accel.z * GRAVITY_MSS;
        if (!sensor_filt_initialized) {
            rref_accel_filt = accel_raw;
            sensor_filt_initialized = true;
        } else {
            rref_accel_filt = accel_raw * SENSOR_LPF_ALPHA
                            + rref_accel_filt * (1.0f - SENSOR_LPF_ALPHA);
        }
        if (!accel_calibrated && !hal.util->get_soft_armed()) {
            if (accel_cal_stationary_count < ACCEL_CAL_WAIT_SAMPLES) {
                accel_cal_stationary_count++;
            } else {
                accel_cal_sum_mag += rref_accel_filt.length();
                accel_cal_count++;
                if (accel_cal_count >= ACCEL_CAL_SAMPLES) {
                    const float measured = accel_cal_sum_mag / accel_cal_count;
                    if (measured > 0.1f) {
                        accel_scale_factor = GRAVITY_MSS / measured;
                        accel_calibrated = true;
                        printf("X-Plane accel calibrated: |g|=%.4f m/s2 scale=%.4f (%.2f%%)\n",
                               measured, accel_scale_factor,
                               (accel_scale_factor - 1.0f) * 100.0f);
                    }
                }
            }
        }
        accel_body = rref_accel_filt * accel_scale_factor;
    }
    if (!hal.util->get_soft_armed()) {
        // pre-arm accel hold (matches the switch case)
        accel_body.x = 0.f;
        accel_body.y = 0.f;
        accel_body.z = -GRAVITY_MSS;
    }

    // --- Apply RREF state. `pos` is a local in receive_data() so we go
    // straight to the base class `position`, matching the math in
    // receive_data's finalisation: position = pos + position_zero, then add
    // home NE offset.
    if (rref_pos_valid) {
        position = rref_pos_ned + position_zero;
        position.xy() += origin.get_distance_NE_double(home);
    }
    if (rref_vel_valid) { velocity_ef = rref_vel_ned; }
    if (rref_att_valid) {
        dcm.from_euler(rref_roll_rad, rref_pitch_rad, rref_yaw_rad);
    }
    if (rref_airspeed_valid) {
        airspeed = rref_airspeed_mps;
        airspeed_pitot = airspeed;
    }

    // Advance time by exactly 1 ms wall-clock. Repeated calls from update()
    // keep simulated time aligned with real time at 1 kHz resolution.
    time_now_us += 1000;

    update_position();
    time_advance();
    accel_earth = dcm * accel_body;
    accel_earth.z += GRAVITY_MSS;
    update_mag_field_bf();
}

/*
  receive RREF replies
*/
void XPlane::handle_rref(const uint8_t *pkt, uint32_t len)
{
    // X-Plane batches one or more (code, float) entries after the 5-byte
    // "RREF\0" header. Iterate over ALL of them — reading only the first
    // entry per packet leaves later DREFs (e.g. g_side / g_nrml) frozen
    // at their boot values whenever X-Plane batches, which corrupts the
    // gravity vector and breaks the EKF attitude solution. PPP/serial
    // links from fmuv3-HIL tend to batch much more than SITL localhost,
    // which is why the same firmware was correct on SITL but broken on HIL.
    // Use memcpy to avoid unaligned float/uint32 access on Cortex-M4 (HardFault).
    size_t off = 5;
    while (off + 8 <= len) {
        uint32_t ref_code;
        float    ref_value_f;
        memcpy(&ref_code,    pkt + off,     4);
        memcpy(&ref_value_f, pkt + off + 4, 4);
        off += 8;
        switch (ref_code) {
    case RREF_VERSION:
        if (xplane_version == 0) {
            ::printf("XPlane version %.0f\n", ref_value_f);
        }
        xplane_version = uint32_t(ref_value_f);
        break;
    case RREF_PRAD:
        if (!rref_gyro_bias_locked_x) {
            rref_gyro_bias_sum.x    += ref_value_f;
            rref_gyro_bias_sum_sq.x += ref_value_f * ref_value_f;
            rref_gyro_bias_count_x++;
            if (rref_gyro_bias_count_x >= GYRO_BIAS_SAMPLES) {
                const float n    = rref_gyro_bias_count_x;
                const float mean = rref_gyro_bias_sum.x / n;
                const float var  = (rref_gyro_bias_sum_sq.x / n) - (mean * mean);
                if (var > GYRO_BIAS_STDDEV_LIMIT * GYRO_BIAS_STDDEV_LIMIT) {
                    // Window had too much variation — vehicle wasn't stationary.
                    // Discard and start over.
                    rref_gyro_bias_sum.x    = 0;
                    rref_gyro_bias_sum_sq.x = 0;
                    rref_gyro_bias_count_x  = 0;
                    rref_gyro_bias_rejects++;
                } else {
                    rref_gyro_bias.x = mean;
                    rref_gyro_bias_locked_x = true;
                }
            }
        }
        rref_gyro.x = ref_value_f - rref_gyro_bias.x;
        rref_gyro_mask |= 1;
        if (rref_gyro_mask == 7) { rref_gyro_valid = true; }
        break;
    case RREF_QRAD:
        if (!rref_gyro_bias_locked_y) {
            rref_gyro_bias_sum.y    += ref_value_f;
            rref_gyro_bias_sum_sq.y += ref_value_f * ref_value_f;
            rref_gyro_bias_count_y++;
            if (rref_gyro_bias_count_y >= GYRO_BIAS_SAMPLES) {
                const float n    = rref_gyro_bias_count_y;
                const float mean = rref_gyro_bias_sum.y / n;
                const float var  = (rref_gyro_bias_sum_sq.y / n) - (mean * mean);
                if (var > GYRO_BIAS_STDDEV_LIMIT * GYRO_BIAS_STDDEV_LIMIT) {
                    rref_gyro_bias_sum.y    = 0;
                    rref_gyro_bias_sum_sq.y = 0;
                    rref_gyro_bias_count_y  = 0;
                    rref_gyro_bias_rejects++;
                } else {
                    rref_gyro_bias.y = mean;
                    rref_gyro_bias_locked_y = true;
                }
            }
        }
        rref_gyro.y = ref_value_f - rref_gyro_bias.y;
        rref_gyro_mask |= 2;
        if (rref_gyro_mask == 7) { rref_gyro_valid = true; }
        break;
    case RREF_RRAD:
        if (!rref_gyro_bias_locked_z) {
            rref_gyro_bias_sum.z    += ref_value_f;
            rref_gyro_bias_sum_sq.z += ref_value_f * ref_value_f;
            rref_gyro_bias_count_z++;
            if (rref_gyro_bias_count_z >= GYRO_BIAS_SAMPLES) {
                const float n    = rref_gyro_bias_count_z;
                const float mean = rref_gyro_bias_sum.z / n;
                const float var  = (rref_gyro_bias_sum_sq.z / n) - (mean * mean);
                if (var > GYRO_BIAS_STDDEV_LIMIT * GYRO_BIAS_STDDEV_LIMIT) {
                    rref_gyro_bias_sum.z    = 0;
                    rref_gyro_bias_sum_sq.z = 0;
                    rref_gyro_bias_count_z  = 0;
                    rref_gyro_bias_rejects++;
                } else {
                    rref_gyro_bias.z = mean;
                    rref_gyro_bias_locked_z = true;
                }
            }
        }
        rref_gyro.z = ref_value_f - rref_gyro_bias.z;
        rref_gyro_mask |= 4;
        if (rref_gyro_mask == 7) { rref_gyro_valid = true; }
        // Print the locked bias once, after all three axes finish calibrating.
        if (!rref_gyro_bias_reported && rref_gyro_bias_locked_x
            && rref_gyro_bias_locked_y && rref_gyro_bias_locked_z) {
            ::printf("XPlane gyro bias locked (%u samples/axis, %u windows rejected): "
                     "X=%.4f Y=%.4f Z=%.4f rad/s\n",
                     (unsigned)GYRO_BIAS_SAMPLES,
                     (unsigned)rref_gyro_bias_rejects,
                     (double)rref_gyro_bias.x,
                     (double)rref_gyro_bias.y,
                     (double)rref_gyro_bias.z);
            rref_gyro_bias_reported = true;
        }
        break;
    case RREF_GAXIL:
        rref_accel.x = ref_value_f;
        rref_accel_mask |= 1;
        if (rref_accel_mask == 7) { rref_accel_valid = true; }
        break;
    case RREF_GSIDE:
        rref_accel.y = ref_value_f;
        rref_accel_mask |= 2;
        if (rref_accel_mask == 7) { rref_accel_valid = true; }
        break;
    case RREF_GNRML:
        rref_accel.z = ref_value_f;
        rref_accel_mask |= 4;
        if (rref_accel_mask == 7) { rref_accel_valid = true; }
        break;

    // --- Position (X-Plane OpenGL local frame: x=east, y=up, z=south) → NED ---
    case RREF_LX: rref_pos_ned.y =  ref_value_f; rref_pos_mask |= 1; if (rref_pos_mask == 7) { rref_pos_valid = true; } break;
    case RREF_LY: rref_pos_ned.z = -ref_value_f; rref_pos_mask |= 2; if (rref_pos_mask == 7) { rref_pos_valid = true; } break;
    case RREF_LZ: rref_pos_ned.x = -ref_value_f; rref_pos_mask |= 4; if (rref_pos_mask == 7) { rref_pos_valid = true; } break;

    // --- Velocity (same frame mapping; matches PX4: vel_n=-vz, vel_e=vx, vel_d=-vy) ---
    case RREF_VX: rref_vel_ned.y =  ref_value_f; rref_vel_mask |= 1; if (rref_vel_mask == 7) { rref_vel_valid = true; } break;
    case RREF_VY: rref_vel_ned.z = -ref_value_f; rref_vel_mask |= 2; if (rref_vel_mask == 7) { rref_vel_valid = true; } break;
    case RREF_VZ: rref_vel_ned.x = -ref_value_f; rref_vel_mask |= 4; if (rref_vel_mask == 7) { rref_vel_valid = true; } break;

    // --- Geodetic position (used for the home-reset reference) ---
    case RREF_LAT:  rref_lat_deg = ref_value_f; rref_geo_mask |= 1; if (rref_geo_mask == 7) { rref_geo_valid = true; } break;
    case RREF_LON:  rref_lon_deg = ref_value_f; rref_geo_mask |= 2; if (rref_geo_mask == 7) { rref_geo_valid = true; } break;
    case RREF_ELEV: rref_elev_m  = ref_value_f; rref_geo_mask |= 4; if (rref_geo_mask == 7) { rref_geo_valid = true; } break;

    // --- Attitude (X-Plane theta=pitch, phi=roll, psi=true heading, degrees) ---
    case RREF_THETA: rref_pitch_rad = radians(ref_value_f); rref_att_mask |= 1; if (rref_att_mask == 7) { rref_att_valid = true; } break;
    case RREF_PHI:   rref_roll_rad  = radians(ref_value_f); rref_att_mask |= 2; if (rref_att_mask == 7) { rref_att_valid = true; } break;
    case RREF_PSI:   rref_yaw_rad   = radians(ref_value_f); rref_att_mask |= 4; if (rref_att_mask == 7) { rref_att_valid = true; } break;

    case RREF_TAS:   rref_airspeed_mps = ref_value_f; rref_airspeed_valid = true; break;
        }   // switch (ref_code)
    }       // while (off + 8 <= len)
}


/*
  send DRef data to X-Plane via UDP.

  Each DREF packet is 509 bytes (X-Plane protocol, cannot be shortened).
  On a PPP link at 115200 baud (~10 KB/s effective), sending all DREFs
  every cycle would overflow the link.  Instead we send at most ONE DREF
  per call, round-robin through the list.  At 25 Hz that is ~12.7 KB/s
  outbound — tight but feasible at 115200, comfortable at 921600.

  FIXED DREFs (override flags) are re-sent once per second so that a
  single dropped UDP packet cannot permanently disable them.
*/
#define DREF_DEADBAND 0.005f
#define DREF_FIXED_RESEND_CYCLES 25   // resend FIXED DREFs every N calls (~1 s at 25 Hz)

void XPlane::send_drefs(const struct sitl_input &input)
{
    // On joystick release request (e.g. TRACKING mode exit): send 0 to all
    // non-FIXED DREFs immediately so X-Plane neutralises yoke and throttle.
    // On arm transition, reset deadband so all DREFs are re-sent immediately.
    const bool armed = hal.util->get_soft_armed();
    if (armed != last_armed) {
        for (auto *d = drefs; d; d=d->next) {
            d->last_sent = NAN;
        }
        last_armed = armed;
    }

    const bool resend_fixed = (++dref_fixed_count >= DREF_FIXED_RESEND_CYCLES);
    if (resend_fixed) {
        dref_fixed_count = 0;
        // Priority pass: send each FIXED DREF immediately so override flags
        // are never starved by roll/pitch winning the round-robin every cycle.
        for (auto *d = drefs; d; d = d->next) {
            if (d->type == DRefType::FIXED) {
                d->last_sent = NAN;   // force resend
                send_dref(d->name, d->fixed_value);
            }
        }
        return;
    }

    // Round-robin: start from where we left off last call and scan
    // the full list once, sending the FIRST DREF that needs an update.

    // Per-channel PWM → normalised value helpers.
    // Uses SERVO{n}_TRIM as centre and SERVO{n}_MIN/MAX as endpoints so that
    // non-standard trims and asymmetric throws are handled correctly.
    // Falls back to 1000/1500/2000 if SRV_Channels isn't available yet.
    auto servo_trim = [](uint8_t ch_idx) -> float {
        const SRV_Channel *ch = SRV_Channels::srv_channel(ch_idx - 1);
        return ch ? (float)ch->get_trim() : 1500.0f;
    };
    // Half-range toward maximum (positive side).
    auto servo_half_up = [](uint8_t ch_idx) -> float {
        const SRV_Channel *ch = SRV_Channels::srv_channel(ch_idx - 1);
        if (!ch) return 500.0f;
        const float h = (float)ch->get_output_max() - (float)ch->get_trim();
        return h > 1.0f ? h : 500.0f;
    };
    // Half-range toward minimum (negative side).
    auto servo_half_dn = [](uint8_t ch_idx) -> float {
        const SRV_Channel *ch = SRV_Channels::srv_channel(ch_idx - 1);
        if (!ch) return 500.0f;
        const float h = (float)ch->get_trim() - (float)ch->get_output_min();
        return h > 1.0f ? h : 500.0f;
    };

    // Compute value for one DRef and return it (does not send).
    auto compute_dref = [&](const DRef *d) -> float {
        float v = 0.0f;
        switch (d->type) {
        case DRefType::ANGLE: {
            const float pwm  = input.servos[d->channel-1];
            const float trim = servo_trim(d->channel);
            const float half = (pwm >= trim) ? servo_half_up(d->channel)
                                             : servo_half_dn(d->channel);
            v = d->range * (pwm - trim) / half;
            v = constrain_float(v, -d->range, d->range);
            break;
        }
        case DRefType::RANGE: {
            const SRV_Channel *ch = SRV_Channels::srv_channel(d->channel - 1);
            const float mn   = ch ? (float)ch->get_output_min() : 1000.0f;
            const float mx   = ch ? (float)ch->get_output_max() : 2000.0f;
            const float span = mx - mn;
            if (!hal.util->get_soft_armed()) {
                v = 0.0f;
            } else {
                v = d->range * (input.servos[d->channel-1] - mn) / span;
                v = constrain_float(v, 0.0f, d->range);
            }
            break;
        }
        case DRefType::ELEVON_AILERON: {
            const float ch1   = input.servos[d->channel-1];
            const float ch2   = input.servos[d->channel2-1];
            const float denom = servo_half_up(d->channel) + servo_half_up(d->channel2);
            v = d->range * (ch1 - ch2) / denom;
            v = constrain_float(v, -d->range, d->range);
            break;
        }
        case DRefType::ELEVON_ELEVATOR: {
            const float ch1      = input.servos[d->channel-1];
            const float ch2      = input.servos[d->channel2-1];
            const float sum_trim = servo_trim(d->channel) + servo_trim(d->channel2);
            const float denom    = servo_half_up(d->channel) + servo_half_up(d->channel2);
            v = -d->range * (ch1 + ch2 - sum_trim) / denom;
            v = constrain_float(v, -d->range, d->range);
            break;
        }
        case DRefType::VTAIL_ELEVATOR: {
            const float ch1      = input.servos[d->channel-1];
            const float ch2      = input.servos[d->channel2-1];
            const float sum_trim = servo_trim(d->channel) + servo_trim(d->channel2);
            const float denom    = servo_half_up(d->channel) + servo_half_up(d->channel2);
            v = -d->range * (ch1 + ch2 - sum_trim) / denom;
            v = constrain_float(v, -d->range, d->range);
            break;
        }
        case DRefType::VTAIL_RUDDER: {
            const float ch1   = input.servos[d->channel-1];
            const float ch2   = input.servos[d->channel2-1];
            const float denom = servo_half_up(d->channel) + servo_half_up(d->channel2);
            v = d->range * (ch2 - ch1) / denom;
            v = constrain_float(v, -d->range, d->range);
            break;
        }
        case DRefType::RUNNING: {
            const SRV_Channel *ch = SRV_Channels::srv_channel(d->channel - 1);
            const float mn = ch ? (float)ch->get_output_min() : 1000.0f;
            v = (hal.util->get_soft_armed() && input.servos[d->channel-1] > mn) ? d->range : 0.0f;
            break;
        }
        case DRefType::QUAD_ROLL:
        case DRefType::QUAD_PITCH:
        case DRefType::QUAD_THROTTLE:
        case DRefType::QUAD_YAW: {
            // Inverse ArduCopter X-frame mixer (AP_MotorsMatrix, FRAME_CLASS=1 FRAME_TYPE=1):
            //   CH1=front-right/CCW  CH2=back-left/CCW  CH3=front-left/CW  CH4=back-right/CW
            // Forward mixer:  M1=T-R+P+Y  M2=T+R-P+Y  M3=T+R+P-Y  M4=T-R-P-Y
            // Inverse:  T=(n1+n2+n3+n4)/4  R=(-n1+n2+n3-n4)/4
            //           P=(-n1+n2-n3+n4)/4 (pilot +nose-up)  Y=(n1+n2-n3-n4)/4
            float n[4];
            for (int i = 0; i < 4; i++) {
                const SRV_Channel *ch = SRV_Channels::srv_channel(i);
                const float mn   = ch ? (float)ch->get_output_min() : 1000.0f;
                const float mx   = ch ? (float)ch->get_output_max() : 2000.0f;
                const float span = mx - mn;
                n[i] = hal.util->get_soft_armed() ? (input.servos[i] - mn) / span : 0.0f;
                n[i] = constrain_float(n[i], 0.0f, 1.0f);
            }
            const float T =  (n[0] + n[1] + n[2] + n[3]) * 0.25f;
            const float R = (-n[0] + n[1] + n[2] - n[3]) * 0.25f;  // [-0.5, +0.5]
            const float P = (-n[0] + n[1] - n[2] + n[3]) * 0.25f;  // pilot +nose-up
            const float Y =  (n[0] + n[1] - n[2] - n[3]) * 0.25f;
            float raw;
            switch (d->type) {
            case DRefType::QUAD_ROLL:     raw = R * 2.0f; break;  // scale to [-1, +1]
            case DRefType::QUAD_PITCH:    raw = P * 2.0f; break;
            case DRefType::QUAD_YAW:      raw = Y * 2.0f; break;
            case DRefType::QUAD_THROTTLE: raw = T;         break;  // already [0, 1]
            default:                      raw = 0.0f;      break;
            }
            if (d->invert) { raw = -raw; }
            const float lo = (d->type == DRefType::QUAD_THROTTLE) ? 0.0f : -d->range;
            v = constrain_float(raw * d->range, lo, d->range);
            break;
        }
        default:
            break;
        }
        if (d->invert) {
            v = -v;
        }
        return v;
    };

    // Send one DRef if its value has changed beyond the deadband.  Returns true if sent.
    auto try_send = [&](DRef *d) -> bool {
        const float v = compute_dref(d);
        if (!isnan(d->last_sent) && fabsf(v - d->last_sent) < DREF_DEADBAND) {
            return false;
        }
        d->last_sent = v;
        send_dref(d->name, v);
        return true;
    };

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // SITL (x86/UDP): no PPP bandwidth limit — send every primary DREF that changed.
    // A quad needs all 4 motor DREFs fresh every scheduler tick (100 Hz) for stable
    // attitude control. Round-robin at 25 Hz is not sufficient.
    for (auto *d = drefs; d; d = d->next) {
        if (d->type == DRefType::FIXED || d->is_pair_secondary) {
            continue;
        }
        try_send(d);
        if (d->pair != nullptr) {
            const float v = compute_dref(d->pair);
            d->pair->last_sent = v;
            send_dref(d->pair->name, v);
        }
    }
#else
    // Hardware (ChibiOS/PPP): round-robin one primary per call.
    // Each DREF packet is 509 bytes; at 115200 baud (~10 KB/s) only one fits per tick.
    auto next_primary = [&](DRef *start) -> DRef* {
        DRef *d = start ? start->next : drefs;
        if (d == nullptr) d = drefs;
        DRef *begin = d;
        while (d != nullptr) {
            if (d->type != DRefType::FIXED && !d->is_pair_secondary) {
                return d;
            }
            d = d->next ? d->next : drefs;
            if (d == begin) break;
        }
        return nullptr;
    };

    if (dref_cursor == nullptr) {
        dref_cursor = next_primary(nullptr);
    }
    if (dref_cursor == nullptr) {
        return;
    }

    try_send(dref_cursor);
    if (dref_cursor->pair != nullptr) {
        const float v = compute_dref(dref_cursor->pair);
        dref_cursor->pair->last_sent = v;
        send_dref(dref_cursor->pair->name, v);
    }

    dref_cursor = next_primary(dref_cursor);
#endif
}


/*
  send DREF to X-Plane via UDP
*/
void XPlane::send_dref(const char *name, float value)
{
    if (!connected) {
        // socket_out.connect() has not been called yet — X-Plane hasn't sent
        // its first DATA@ packet so we don't know its address.  Drop silently.
        return;
    }
    static struct PACKED {
        uint8_t  marker[5];
        float value;
        char name[500];
    } d;
    memcpy(d.marker, "DREF\0", 5);
    d.value = value;
    memset(d.name, 0, sizeof(d.name));
    strncpy(d.name, name, sizeof(d.name) - 1);
    socket_out.send(&d, sizeof(d));
    if (dref_debug > 0) {
        ::printf("-> %s : %.3f\n", name, value);
    }
}

/*
  request a dref
*/
void XPlane::request_dref(const char *name, uint8_t code, uint32_t rate)
{
    static struct PACKED {
        uint8_t  marker[5];
        uint32_t rate_hz;
        uint32_t code;
        char name[400];
    } d;
    memcpy(d.marker, "RREF\0", 5);
    d.rate_hz = rate;
    d.code = code; // given back in responses
    memset(d.name, 0, sizeof(d.name));
    strncpy(d.name, name, sizeof(d.name) - 1);
    socket_in.sendto(&d, sizeof(d), xplane_ip, xplane_port);
}

void XPlane::request_drefs(void)
{
    request_dref("sim/version/xplane_internal_version", RREF_VERSION, 1);
    // Body angular rates — same DREFs PX4xplane reads; bypasses DATA@ row-16 ambiguity
    request_dref("sim/flightmodel/position/Prad",  RREF_PRAD,  100);
    request_dref("sim/flightmodel/position/Qrad",  RREF_QRAD,  100);
    request_dref("sim/flightmodel/position/Rrad",  RREF_RRAD,  100);
    // Body accelerations — same DREFs PX4xplane reads; bypasses DATA@ row-4 ambiguity
    request_dref("sim/flightmodel/forces/g_axil",  RREF_GAXIL, 100);
    request_dref("sim/flightmodel/forces/g_side",  RREF_GSIDE, 100);
    request_dref("sim/flightmodel/forces/g_nrml",  RREF_GNRML, 100);
    // Position / velocity / attitude / airspeed via RREF — replaces the DATA@
    // rows (group 20/21/17/3) so the FDM state no longer depends on X-Plane's
    // "Data Output" config or its slot ordering. Matches PX4's SimulatorXPlane.
    request_dref("sim/flightmodel/position/local_x",  RREF_LX, 100);
    request_dref("sim/flightmodel/position/local_y",  RREF_LY, 100);
    request_dref("sim/flightmodel/position/local_z",  RREF_LZ, 100);
    request_dref("sim/flightmodel/position/local_vx", RREF_VX, 100);
    request_dref("sim/flightmodel/position/local_vy", RREF_VY, 100);
    request_dref("sim/flightmodel/position/local_vz", RREF_VZ, 100);
    request_dref("sim/flightmodel/position/latitude",  RREF_LAT,  50);
    request_dref("sim/flightmodel/position/longitude", RREF_LON,  50);
    request_dref("sim/flightmodel/position/elevation", RREF_ELEV, 50);
    request_dref("sim/flightmodel/position/theta", RREF_THETA, 100);
    request_dref("sim/flightmodel/position/phi",   RREF_PHI,   100);
    request_dref("sim/flightmodel/position/psi",   RREF_PSI,   100);
    request_dref("sim/flightmodel/position/true_airspeed", RREF_TAS, 50);
}


/*
  update the XPlane simulation by one time step
 */
void XPlane::update(const struct sitl_input &input)
{
    if (receive_data()) {
        // DREF send cadence is bandwidth-bound. Local UDP (SITL) handles 100 Hz
        // easily (tighter loop = faster actuator response); a PPP serial link
        // (~10 KB/s) needs the 25 Hz round-robin cadence — at 100 Hz even one
        // 509-byte DREF per tick (~51 KB/s) overflows the link and starves
        // X-Plane's reply packets (sending but not replying).
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        const uint32_t dref_interval_ms = 10;   // 100 Hz — local loopback
#else
        const uint32_t dref_interval_ms = 40;   // 25 Hz — PPP bandwidth limit
#endif
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_dref_ms >= dref_interval_ms) {
            last_dref_ms = now_ms;
            send_drefs(input);
        }
    } else {
        // No DATA yet — keep requesting data rows from X-Plane via DSEL.
        // select_data() uses socket_out which is connected in the constructor,
        // so this works even before the first DATA packet (connected=false).
        select_data();
    }

    uint32_t now = AP_HAL::millis();
    if (report.last_report_ms == 0) {
        report.last_report_ms = now;
        request_drefs();
    }
    if (now - report.last_report_ms > 5000) {
        float dt = (now - report.last_report_ms) * 1.0e-3f;
        printf("Data rate: %.1f FPS  Frame rate: %.1f FPS\n",
               report.data_count/dt, report.frame_count/dt);
        report.last_report_ms = now;
        report.data_count = 0;
        report.frame_count = 0;
        request_drefs();
    }
    check_reload_dref();

    // Keep simulated time advancing in lockstep with wall-clock (1 ms steps)
    // regardless of which packet receive_data() handled. RREF-only ticks and
    // failed-path ticks no longer freeze AP_HAL::millis(), so the SITL GPS
    // (and any other sensor gated on simulated time) keeps emitting at its
    // configured rate even when X-Plane's DATA@ stream is sparse.
    wall_paced_time_advance();
}

#endif  // AP_SIM_XPLANE_ENABLED
