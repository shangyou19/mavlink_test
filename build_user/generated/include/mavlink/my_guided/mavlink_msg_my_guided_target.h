#pragma once
// MESSAGE MY_GUIDED_TARGET PACKING

#define MAVLINK_MSG_ID_MY_GUIDED_TARGET 512


typedef struct __mavlink_my_guided_target_t {
 float lat; /*<  latitude (degrees)*/
 float lon; /*<  longitude (degrees)*/
 float alt; /*<  altitude (meters AMSL)*/
 float yaw; /*<  optional yaw (degrees)*/
} mavlink_my_guided_target_t;

#define MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN 16
#define MAVLINK_MSG_ID_MY_GUIDED_TARGET_MIN_LEN 16
#define MAVLINK_MSG_ID_512_LEN 16
#define MAVLINK_MSG_ID_512_MIN_LEN 16

#define MAVLINK_MSG_ID_MY_GUIDED_TARGET_CRC 189
#define MAVLINK_MSG_ID_512_CRC 189



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MY_GUIDED_TARGET { \
    512, \
    "MY_GUIDED_TARGET", \
    4, \
    {  { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_my_guided_target_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_my_guided_target_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_my_guided_target_t, alt) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_my_guided_target_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MY_GUIDED_TARGET { \
    "MY_GUIDED_TARGET", \
    4, \
    {  { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_my_guided_target_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_my_guided_target_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_my_guided_target_t, alt) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_my_guided_target_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a my_guided_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat  latitude (degrees)
 * @param lon  longitude (degrees)
 * @param alt  altitude (meters AMSL)
 * @param yaw  optional yaw (degrees)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_my_guided_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float lat, float lon, float alt, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN];
    _mav_put_float(buf, 0, lat);
    _mav_put_float(buf, 4, lon);
    _mav_put_float(buf, 8, alt);
    _mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN);
#else
    mavlink_my_guided_target_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MY_GUIDED_TARGET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MY_GUIDED_TARGET_MIN_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_CRC);
}

/**
 * @brief Pack a my_guided_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat  latitude (degrees)
 * @param lon  longitude (degrees)
 * @param alt  altitude (meters AMSL)
 * @param yaw  optional yaw (degrees)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_my_guided_target_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float lat, float lon, float alt, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN];
    _mav_put_float(buf, 0, lat);
    _mav_put_float(buf, 4, lon);
    _mav_put_float(buf, 8, alt);
    _mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN);
#else
    mavlink_my_guided_target_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MY_GUIDED_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MY_GUIDED_TARGET_MIN_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MY_GUIDED_TARGET_MIN_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN);
#endif
}

/**
 * @brief Pack a my_guided_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat  latitude (degrees)
 * @param lon  longitude (degrees)
 * @param alt  altitude (meters AMSL)
 * @param yaw  optional yaw (degrees)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_my_guided_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float lat,float lon,float alt,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN];
    _mav_put_float(buf, 0, lat);
    _mav_put_float(buf, 4, lon);
    _mav_put_float(buf, 8, alt);
    _mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN);
#else
    mavlink_my_guided_target_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MY_GUIDED_TARGET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MY_GUIDED_TARGET_MIN_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_CRC);
}

/**
 * @brief Encode a my_guided_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param my_guided_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_my_guided_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_my_guided_target_t* my_guided_target)
{
    return mavlink_msg_my_guided_target_pack(system_id, component_id, msg, my_guided_target->lat, my_guided_target->lon, my_guided_target->alt, my_guided_target->yaw);
}

/**
 * @brief Encode a my_guided_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param my_guided_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_my_guided_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_my_guided_target_t* my_guided_target)
{
    return mavlink_msg_my_guided_target_pack_chan(system_id, component_id, chan, msg, my_guided_target->lat, my_guided_target->lon, my_guided_target->alt, my_guided_target->yaw);
}

/**
 * @brief Encode a my_guided_target struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param my_guided_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_my_guided_target_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_my_guided_target_t* my_guided_target)
{
    return mavlink_msg_my_guided_target_pack_status(system_id, component_id, _status, msg,  my_guided_target->lat, my_guided_target->lon, my_guided_target->alt, my_guided_target->yaw);
}

/**
 * @brief Send a my_guided_target message
 * @param chan MAVLink channel to send the message
 *
 * @param lat  latitude (degrees)
 * @param lon  longitude (degrees)
 * @param alt  altitude (meters AMSL)
 * @param yaw  optional yaw (degrees)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_my_guided_target_send(mavlink_channel_t chan, float lat, float lon, float alt, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN];
    _mav_put_float(buf, 0, lat);
    _mav_put_float(buf, 4, lon);
    _mav_put_float(buf, 8, alt);
    _mav_put_float(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MY_GUIDED_TARGET, buf, MAVLINK_MSG_ID_MY_GUIDED_TARGET_MIN_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_CRC);
#else
    mavlink_my_guided_target_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MY_GUIDED_TARGET, (const char *)&packet, MAVLINK_MSG_ID_MY_GUIDED_TARGET_MIN_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_CRC);
#endif
}

/**
 * @brief Send a my_guided_target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_my_guided_target_send_struct(mavlink_channel_t chan, const mavlink_my_guided_target_t* my_guided_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_my_guided_target_send(chan, my_guided_target->lat, my_guided_target->lon, my_guided_target->alt, my_guided_target->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MY_GUIDED_TARGET, (const char *)my_guided_target, MAVLINK_MSG_ID_MY_GUIDED_TARGET_MIN_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_CRC);
#endif
}

#if MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_my_guided_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float lat, float lon, float alt, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, lat);
    _mav_put_float(buf, 4, lon);
    _mav_put_float(buf, 8, alt);
    _mav_put_float(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MY_GUIDED_TARGET, buf, MAVLINK_MSG_ID_MY_GUIDED_TARGET_MIN_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_CRC);
#else
    mavlink_my_guided_target_t *packet = (mavlink_my_guided_target_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MY_GUIDED_TARGET, (const char *)packet, MAVLINK_MSG_ID_MY_GUIDED_TARGET_MIN_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN, MAVLINK_MSG_ID_MY_GUIDED_TARGET_CRC);
#endif
}
#endif

#endif

// MESSAGE MY_GUIDED_TARGET UNPACKING


/**
 * @brief Get field lat from my_guided_target message
 *
 * @return  latitude (degrees)
 */
static inline float mavlink_msg_my_guided_target_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field lon from my_guided_target message
 *
 * @return  longitude (degrees)
 */
static inline float mavlink_msg_my_guided_target_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field alt from my_guided_target message
 *
 * @return  altitude (meters AMSL)
 */
static inline float mavlink_msg_my_guided_target_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from my_guided_target message
 *
 * @return  optional yaw (degrees)
 */
static inline float mavlink_msg_my_guided_target_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a my_guided_target message into a struct
 *
 * @param msg The message to decode
 * @param my_guided_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_my_guided_target_decode(const mavlink_message_t* msg, mavlink_my_guided_target_t* my_guided_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    my_guided_target->lat = mavlink_msg_my_guided_target_get_lat(msg);
    my_guided_target->lon = mavlink_msg_my_guided_target_get_lon(msg);
    my_guided_target->alt = mavlink_msg_my_guided_target_get_alt(msg);
    my_guided_target->yaw = mavlink_msg_my_guided_target_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN? msg->len : MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN;
        memset(my_guided_target, 0, MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN);
    memcpy(my_guided_target, _MAV_PAYLOAD(msg), len);
#endif
}
