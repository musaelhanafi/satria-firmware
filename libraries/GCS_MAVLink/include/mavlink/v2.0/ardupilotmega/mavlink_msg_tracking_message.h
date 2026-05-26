#pragma once
// MESSAGE TRACKING_MESSAGE PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_TRACKING_MESSAGE 11045


typedef struct __mavlink_tracking_message_t {
 uint64_t time_usec; /*< [us] Timestamp (monotonic microseconds)*/
 float errorx; /*<  Normalised horizontal tracking error [-1, 1]*/
 float errory; /*<  Normalised vertical tracking error [-1, 1]*/
} mavlink_tracking_message_t;

#define MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN 16
#define MAVLINK_MSG_ID_TRACKING_MESSAGE_MIN_LEN 16
#define MAVLINK_MSG_ID_11045_LEN 16
#define MAVLINK_MSG_ID_11045_MIN_LEN 16

#define MAVLINK_MSG_ID_TRACKING_MESSAGE_CRC 3
#define MAVLINK_MSG_ID_11045_CRC 3



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TRACKING_MESSAGE { \
    11045, \
    "TRACKING_MESSAGE", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tracking_message_t, time_usec) }, \
         { "errorx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_tracking_message_t, errorx) }, \
         { "errory", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_tracking_message_t, errory) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TRACKING_MESSAGE { \
    "TRACKING_MESSAGE", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tracking_message_t, time_usec) }, \
         { "errorx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_tracking_message_t, errorx) }, \
         { "errory", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_tracking_message_t, errory) }, \
         } \
}
#endif

/**
 * @brief Pack a tracking_message message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (monotonic microseconds)
 * @param errorx  Normalised horizontal tracking error [-1, 1]
 * @param errory  Normalised vertical tracking error [-1, 1]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tracking_message_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float errorx, float errory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, errorx);
    _mav_put_float(buf, 12, errory);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN);
#else
    mavlink_tracking_message_t packet;
    packet.time_usec = time_usec;
    packet.errorx = errorx;
    packet.errory = errory;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRACKING_MESSAGE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRACKING_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_CRC);
}

/**
 * @brief Pack a tracking_message message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (monotonic microseconds)
 * @param errorx  Normalised horizontal tracking error [-1, 1]
 * @param errory  Normalised vertical tracking error [-1, 1]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tracking_message_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, float errorx, float errory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, errorx);
    _mav_put_float(buf, 12, errory);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN);
#else
    mavlink_tracking_message_t packet;
    packet.time_usec = time_usec;
    packet.errorx = errorx;
    packet.errory = errory;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRACKING_MESSAGE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TRACKING_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TRACKING_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN);
#endif
}

/**
 * @brief Pack a tracking_message message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (monotonic microseconds)
 * @param errorx  Normalised horizontal tracking error [-1, 1]
 * @param errory  Normalised vertical tracking error [-1, 1]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tracking_message_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float errorx,float errory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, errorx);
    _mav_put_float(buf, 12, errory);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN);
#else
    mavlink_tracking_message_t packet;
    packet.time_usec = time_usec;
    packet.errorx = errorx;
    packet.errory = errory;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRACKING_MESSAGE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRACKING_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_CRC);
}

/**
 * @brief Encode a tracking_message struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tracking_message C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tracking_message_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tracking_message_t* tracking_message)
{
    return mavlink_msg_tracking_message_pack(system_id, component_id, msg, tracking_message->time_usec, tracking_message->errorx, tracking_message->errory);
}

/**
 * @brief Encode a tracking_message struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tracking_message C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tracking_message_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tracking_message_t* tracking_message)
{
    return mavlink_msg_tracking_message_pack_chan(system_id, component_id, chan, msg, tracking_message->time_usec, tracking_message->errorx, tracking_message->errory);
}

/**
 * @brief Encode a tracking_message struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tracking_message C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tracking_message_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tracking_message_t* tracking_message)
{
    return mavlink_msg_tracking_message_pack_status(system_id, component_id, _status, msg,  tracking_message->time_usec, tracking_message->errorx, tracking_message->errory);
}

/**
 * @brief Send a tracking_message message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (monotonic microseconds)
 * @param errorx  Normalised horizontal tracking error [-1, 1]
 * @param errory  Normalised vertical tracking error [-1, 1]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tracking_message_send(mavlink_channel_t chan, uint64_t time_usec, float errorx, float errory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, errorx);
    _mav_put_float(buf, 12, errory);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_MESSAGE, buf, MAVLINK_MSG_ID_TRACKING_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_CRC);
#else
    mavlink_tracking_message_t packet;
    packet.time_usec = time_usec;
    packet.errorx = errorx;
    packet.errory = errory;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_MESSAGE, (const char *)&packet, MAVLINK_MSG_ID_TRACKING_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_CRC);
#endif
}

/**
 * @brief Send a tracking_message message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tracking_message_send_struct(mavlink_channel_t chan, const mavlink_tracking_message_t* tracking_message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tracking_message_send(chan, tracking_message->time_usec, tracking_message->errorx, tracking_message->errory);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_MESSAGE, (const char *)tracking_message, MAVLINK_MSG_ID_TRACKING_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_CRC);
#endif
}

#if MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tracking_message_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float errorx, float errory)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, errorx);
    _mav_put_float(buf, 12, errory);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_MESSAGE, buf, MAVLINK_MSG_ID_TRACKING_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_CRC);
#else
    mavlink_tracking_message_t *packet = (mavlink_tracking_message_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->errorx = errorx;
    packet->errory = errory;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKING_MESSAGE, (const char *)packet, MAVLINK_MSG_ID_TRACKING_MESSAGE_MIN_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN, MAVLINK_MSG_ID_TRACKING_MESSAGE_CRC);
#endif
}
#endif

#endif

// MESSAGE TRACKING_MESSAGE UNPACKING


/**
 * @brief Get field time_usec from tracking_message message
 *
 * @return [us] Timestamp (monotonic microseconds)
 */
static inline uint64_t mavlink_msg_tracking_message_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field errorx from tracking_message message
 *
 * @return  Normalised horizontal tracking error [-1, 1]
 */
static inline float mavlink_msg_tracking_message_get_errorx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field errory from tracking_message message
 *
 * @return  Normalised vertical tracking error [-1, 1]
 */
static inline float mavlink_msg_tracking_message_get_errory(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a tracking_message message into a struct
 *
 * @param msg The message to decode
 * @param tracking_message C-struct to decode the message contents into
 */
static inline void mavlink_msg_tracking_message_decode(const mavlink_message_t* msg, mavlink_tracking_message_t* tracking_message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tracking_message->time_usec = mavlink_msg_tracking_message_get_time_usec(msg);
    tracking_message->errorx = mavlink_msg_tracking_message_get_errorx(msg);
    tracking_message->errory = mavlink_msg_tracking_message_get_errory(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN? msg->len : MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN;
        memset(tracking_message, 0, MAVLINK_MSG_ID_TRACKING_MESSAGE_LEN);
    memcpy(tracking_message, _MAV_PAYLOAD(msg), len);
#endif
}
