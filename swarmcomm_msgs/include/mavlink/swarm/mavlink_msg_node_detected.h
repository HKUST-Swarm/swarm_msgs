#pragma once
// MESSAGE NODE_DETECTED PACKING

#define MAVLINK_MSG_ID_NODE_DETECTED 203


typedef struct __mavlink_node_detected_t {
 int64_t id; /*<  Message ID detection*/
 int32_t lps_time; /*< [ms] LPS_TIME*/
 int32_t target_id; /*<  Target ID of drone*/
 float rel_x; /*< [m] Relative X Position*/
 float rel_y; /*< [m] Relative Y Position*/
 float rel_z; /*< [m] Relative Z Position*/
 float rel_yaw; /*< [deg] Yaw of Detector */
 float cov_x; /*< [m] Covariance of Relative X Position*/
 float cov_y; /*< [m] Covariance of Relative Y Position*/
 float cov_z; /*< [m] Covariance of Relative Z Position*/
 float cov_yaw; /*< [deg] Covariance of Yaw of Detector*/
} mavlink_node_detected_t;

#define MAVLINK_MSG_ID_NODE_DETECTED_LEN 48
#define MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN 48
#define MAVLINK_MSG_ID_203_LEN 48
#define MAVLINK_MSG_ID_203_MIN_LEN 48

#define MAVLINK_MSG_ID_NODE_DETECTED_CRC 80
#define MAVLINK_MSG_ID_203_CRC 80



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NODE_DETECTED { \
    203, \
    "NODE_DETECTED", \
    11, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_node_detected_t, lps_time) }, \
         { "id", NULL, MAVLINK_TYPE_INT64_T, 0, 0, offsetof(mavlink_node_detected_t, id) }, \
         { "target_id", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_node_detected_t, target_id) }, \
         { "rel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_node_detected_t, rel_x) }, \
         { "rel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_node_detected_t, rel_y) }, \
         { "rel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_node_detected_t, rel_z) }, \
         { "rel_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_node_detected_t, rel_yaw) }, \
         { "cov_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_node_detected_t, cov_x) }, \
         { "cov_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_node_detected_t, cov_y) }, \
         { "cov_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_node_detected_t, cov_z) }, \
         { "cov_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_node_detected_t, cov_yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NODE_DETECTED { \
    "NODE_DETECTED", \
    11, \
    {  { "lps_time", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_node_detected_t, lps_time) }, \
         { "id", NULL, MAVLINK_TYPE_INT64_T, 0, 0, offsetof(mavlink_node_detected_t, id) }, \
         { "target_id", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_node_detected_t, target_id) }, \
         { "rel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_node_detected_t, rel_x) }, \
         { "rel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_node_detected_t, rel_y) }, \
         { "rel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_node_detected_t, rel_z) }, \
         { "rel_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_node_detected_t, rel_yaw) }, \
         { "cov_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_node_detected_t, cov_x) }, \
         { "cov_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_node_detected_t, cov_y) }, \
         { "cov_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_node_detected_t, cov_z) }, \
         { "cov_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_node_detected_t, cov_yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a node_detected message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lps_time [ms] LPS_TIME
 * @param id  Message ID detection
 * @param target_id  Target ID of drone
 * @param rel_x [m] Relative X Position
 * @param rel_y [m] Relative Y Position
 * @param rel_z [m] Relative Z Position
 * @param rel_yaw [deg] Yaw of Detector 
 * @param cov_x [m] Covariance of Relative X Position
 * @param cov_y [m] Covariance of Relative Y Position
 * @param cov_z [m] Covariance of Relative Z Position
 * @param cov_yaw [deg] Covariance of Yaw of Detector
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_detected_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lps_time, int64_t id, int32_t target_id, float rel_x, float rel_y, float rel_z, float rel_yaw, float cov_x, float cov_y, float cov_z, float cov_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_DETECTED_LEN];
    _mav_put_int64_t(buf, 0, id);
    _mav_put_int32_t(buf, 8, lps_time);
    _mav_put_int32_t(buf, 12, target_id);
    _mav_put_float(buf, 16, rel_x);
    _mav_put_float(buf, 20, rel_y);
    _mav_put_float(buf, 24, rel_z);
    _mav_put_float(buf, 28, rel_yaw);
    _mav_put_float(buf, 32, cov_x);
    _mav_put_float(buf, 36, cov_y);
    _mav_put_float(buf, 40, cov_z);
    _mav_put_float(buf, 44, cov_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_DETECTED_LEN);
#else
    mavlink_node_detected_t packet;
    packet.id = id;
    packet.lps_time = lps_time;
    packet.target_id = target_id;
    packet.rel_x = rel_x;
    packet.rel_y = rel_y;
    packet.rel_z = rel_z;
    packet.rel_yaw = rel_yaw;
    packet.cov_x = cov_x;
    packet.cov_y = cov_y;
    packet.cov_z = cov_z;
    packet.cov_yaw = cov_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_DETECTED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NODE_DETECTED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
}

/**
 * @brief Pack a node_detected message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lps_time [ms] LPS_TIME
 * @param id  Message ID detection
 * @param target_id  Target ID of drone
 * @param rel_x [m] Relative X Position
 * @param rel_y [m] Relative Y Position
 * @param rel_z [m] Relative Z Position
 * @param rel_yaw [deg] Yaw of Detector 
 * @param cov_x [m] Covariance of Relative X Position
 * @param cov_y [m] Covariance of Relative Y Position
 * @param cov_z [m] Covariance of Relative Z Position
 * @param cov_yaw [deg] Covariance of Yaw of Detector
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_detected_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lps_time,int64_t id,int32_t target_id,float rel_x,float rel_y,float rel_z,float rel_yaw,float cov_x,float cov_y,float cov_z,float cov_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_DETECTED_LEN];
    _mav_put_int64_t(buf, 0, id);
    _mav_put_int32_t(buf, 8, lps_time);
    _mav_put_int32_t(buf, 12, target_id);
    _mav_put_float(buf, 16, rel_x);
    _mav_put_float(buf, 20, rel_y);
    _mav_put_float(buf, 24, rel_z);
    _mav_put_float(buf, 28, rel_yaw);
    _mav_put_float(buf, 32, cov_x);
    _mav_put_float(buf, 36, cov_y);
    _mav_put_float(buf, 40, cov_z);
    _mav_put_float(buf, 44, cov_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_DETECTED_LEN);
#else
    mavlink_node_detected_t packet;
    packet.id = id;
    packet.lps_time = lps_time;
    packet.target_id = target_id;
    packet.rel_x = rel_x;
    packet.rel_y = rel_y;
    packet.rel_z = rel_z;
    packet.rel_yaw = rel_yaw;
    packet.cov_x = cov_x;
    packet.cov_y = cov_y;
    packet.cov_z = cov_z;
    packet.cov_yaw = cov_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_DETECTED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NODE_DETECTED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
}

/**
 * @brief Encode a node_detected struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param node_detected C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_detected_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_node_detected_t* node_detected)
{
    return mavlink_msg_node_detected_pack(system_id, component_id, msg, node_detected->lps_time, node_detected->id, node_detected->target_id, node_detected->rel_x, node_detected->rel_y, node_detected->rel_z, node_detected->rel_yaw, node_detected->cov_x, node_detected->cov_y, node_detected->cov_z, node_detected->cov_yaw);
}

/**
 * @brief Encode a node_detected struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param node_detected C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_detected_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_node_detected_t* node_detected)
{
    return mavlink_msg_node_detected_pack_chan(system_id, component_id, chan, msg, node_detected->lps_time, node_detected->id, node_detected->target_id, node_detected->rel_x, node_detected->rel_y, node_detected->rel_z, node_detected->rel_yaw, node_detected->cov_x, node_detected->cov_y, node_detected->cov_z, node_detected->cov_yaw);
}

/**
 * @brief Send a node_detected message
 * @param chan MAVLink channel to send the message
 *
 * @param lps_time [ms] LPS_TIME
 * @param id  Message ID detection
 * @param target_id  Target ID of drone
 * @param rel_x [m] Relative X Position
 * @param rel_y [m] Relative Y Position
 * @param rel_z [m] Relative Z Position
 * @param rel_yaw [deg] Yaw of Detector 
 * @param cov_x [m] Covariance of Relative X Position
 * @param cov_y [m] Covariance of Relative Y Position
 * @param cov_z [m] Covariance of Relative Z Position
 * @param cov_yaw [deg] Covariance of Yaw of Detector
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_node_detected_send(mavlink_channel_t chan, int32_t lps_time, int64_t id, int32_t target_id, float rel_x, float rel_y, float rel_z, float rel_yaw, float cov_x, float cov_y, float cov_z, float cov_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NODE_DETECTED_LEN];
    _mav_put_int64_t(buf, 0, id);
    _mav_put_int32_t(buf, 8, lps_time);
    _mav_put_int32_t(buf, 12, target_id);
    _mav_put_float(buf, 16, rel_x);
    _mav_put_float(buf, 20, rel_y);
    _mav_put_float(buf, 24, rel_z);
    _mav_put_float(buf, 28, rel_yaw);
    _mav_put_float(buf, 32, cov_x);
    _mav_put_float(buf, 36, cov_y);
    _mav_put_float(buf, 40, cov_z);
    _mav_put_float(buf, 44, cov_yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED, buf, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
#else
    mavlink_node_detected_t packet;
    packet.id = id;
    packet.lps_time = lps_time;
    packet.target_id = target_id;
    packet.rel_x = rel_x;
    packet.rel_y = rel_y;
    packet.rel_z = rel_z;
    packet.rel_yaw = rel_yaw;
    packet.cov_x = cov_x;
    packet.cov_y = cov_y;
    packet.cov_z = cov_z;
    packet.cov_yaw = cov_yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED, (const char *)&packet, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
#endif
}

/**
 * @brief Send a node_detected message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_node_detected_send_struct(mavlink_channel_t chan, const mavlink_node_detected_t* node_detected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_node_detected_send(chan, node_detected->lps_time, node_detected->id, node_detected->target_id, node_detected->rel_x, node_detected->rel_y, node_detected->rel_z, node_detected->rel_yaw, node_detected->cov_x, node_detected->cov_y, node_detected->cov_z, node_detected->cov_yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED, (const char *)node_detected, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
#endif
}

#if MAVLINK_MSG_ID_NODE_DETECTED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_node_detected_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lps_time, int64_t id, int32_t target_id, float rel_x, float rel_y, float rel_z, float rel_yaw, float cov_x, float cov_y, float cov_z, float cov_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int64_t(buf, 0, id);
    _mav_put_int32_t(buf, 8, lps_time);
    _mav_put_int32_t(buf, 12, target_id);
    _mav_put_float(buf, 16, rel_x);
    _mav_put_float(buf, 20, rel_y);
    _mav_put_float(buf, 24, rel_z);
    _mav_put_float(buf, 28, rel_yaw);
    _mav_put_float(buf, 32, cov_x);
    _mav_put_float(buf, 36, cov_y);
    _mav_put_float(buf, 40, cov_z);
    _mav_put_float(buf, 44, cov_yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED, buf, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
#else
    mavlink_node_detected_t *packet = (mavlink_node_detected_t *)msgbuf;
    packet->id = id;
    packet->lps_time = lps_time;
    packet->target_id = target_id;
    packet->rel_x = rel_x;
    packet->rel_y = rel_y;
    packet->rel_z = rel_z;
    packet->rel_yaw = rel_yaw;
    packet->cov_x = cov_x;
    packet->cov_y = cov_y;
    packet->cov_z = cov_z;
    packet->cov_yaw = cov_yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_DETECTED, (const char *)packet, MAVLINK_MSG_ID_NODE_DETECTED_MIN_LEN, MAVLINK_MSG_ID_NODE_DETECTED_LEN, MAVLINK_MSG_ID_NODE_DETECTED_CRC);
#endif
}
#endif

#endif

// MESSAGE NODE_DETECTED UNPACKING


/**
 * @brief Get field lps_time from node_detected message
 *
 * @return [ms] LPS_TIME
 */
static inline int32_t mavlink_msg_node_detected_get_lps_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field id from node_detected message
 *
 * @return  Message ID detection
 */
static inline int64_t mavlink_msg_node_detected_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field target_id from node_detected message
 *
 * @return  Target ID of drone
 */
static inline int32_t mavlink_msg_node_detected_get_target_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field rel_x from node_detected message
 *
 * @return [m] Relative X Position
 */
static inline float mavlink_msg_node_detected_get_rel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rel_y from node_detected message
 *
 * @return [m] Relative Y Position
 */
static inline float mavlink_msg_node_detected_get_rel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field rel_z from node_detected message
 *
 * @return [m] Relative Z Position
 */
static inline float mavlink_msg_node_detected_get_rel_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field rel_yaw from node_detected message
 *
 * @return [deg] Yaw of Detector 
 */
static inline float mavlink_msg_node_detected_get_rel_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field cov_x from node_detected message
 *
 * @return [m] Covariance of Relative X Position
 */
static inline float mavlink_msg_node_detected_get_cov_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field cov_y from node_detected message
 *
 * @return [m] Covariance of Relative Y Position
 */
static inline float mavlink_msg_node_detected_get_cov_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field cov_z from node_detected message
 *
 * @return [m] Covariance of Relative Z Position
 */
static inline float mavlink_msg_node_detected_get_cov_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field cov_yaw from node_detected message
 *
 * @return [deg] Covariance of Yaw of Detector
 */
static inline float mavlink_msg_node_detected_get_cov_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a node_detected message into a struct
 *
 * @param msg The message to decode
 * @param node_detected C-struct to decode the message contents into
 */
static inline void mavlink_msg_node_detected_decode(const mavlink_message_t* msg, mavlink_node_detected_t* node_detected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    node_detected->id = mavlink_msg_node_detected_get_id(msg);
    node_detected->lps_time = mavlink_msg_node_detected_get_lps_time(msg);
    node_detected->target_id = mavlink_msg_node_detected_get_target_id(msg);
    node_detected->rel_x = mavlink_msg_node_detected_get_rel_x(msg);
    node_detected->rel_y = mavlink_msg_node_detected_get_rel_y(msg);
    node_detected->rel_z = mavlink_msg_node_detected_get_rel_z(msg);
    node_detected->rel_yaw = mavlink_msg_node_detected_get_rel_yaw(msg);
    node_detected->cov_x = mavlink_msg_node_detected_get_cov_x(msg);
    node_detected->cov_y = mavlink_msg_node_detected_get_cov_y(msg);
    node_detected->cov_z = mavlink_msg_node_detected_get_cov_z(msg);
    node_detected->cov_yaw = mavlink_msg_node_detected_get_cov_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NODE_DETECTED_LEN? msg->len : MAVLINK_MSG_ID_NODE_DETECTED_LEN;
        memset(node_detected, 0, MAVLINK_MSG_ID_NODE_DETECTED_LEN);
    memcpy(node_detected, _MAV_PAYLOAD(msg), len);
#endif
}
