#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

// Minimal MAVLink v2 implementation supporting a subset of common messages
// needed by this project. This avoids requiring the external pymavlink generator.

#define MAVLINK_STX 0xFD
#define MAVLINK_MAX_PAYLOAD_LEN 255

// Enums/defines (subset)
enum MAV_MODE_FLAG : uint8_t {
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1 << 7,
    MAV_MODE_FLAG_TEST_ENABLED        = 1 << 5,
    MAV_MODE_FLAG_AUTO_ENABLED        = 1 << 4,
    MAV_MODE_FLAG_GUIDED_ENABLED      = 1 << 3,
    MAV_MODE_FLAG_STABILIZE_ENABLED   = 1 << 2,
    MAV_MODE_FLAG_HIL_ENABLED         = 1 << 1,
    MAV_MODE_FLAG_SAFETY_ARMED        = 1 << 0,
};

enum MAV_STATE : uint8_t {
    MAV_STATE_UNINIT = 0,
    MAV_STATE_BOOT = 1,
    MAV_STATE_CALIBRATING = 2,
    MAV_STATE_STANDBY = 3,
    MAV_STATE_ACTIVE = 4,
    MAV_STATE_CRITICAL = 5,
    MAV_STATE_EMERGENCY = 6,
    MAV_STATE_POWEROFF = 7,
    MAV_STATE_FLIGHT_TERMINATION = 8,
};

enum MAV_TYPE : uint8_t {
    MAV_TYPE_GENERIC = 0,
    MAV_TYPE_FIXED_WING = 1,
    MAV_TYPE_QUADROTOR = 2,
    MAV_TYPE_GCS = 6,
};

#define MAV_COMP_ID_MISSIONPLANNER 190
#define MAVLINK_COMM_0 0

enum MAV_AUTOPILOT : uint8_t {
    MAV_AUTOPILOT_GENERIC = 0,
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
};

enum MAV_RESULT : uint8_t {
    MAV_RESULT_ACCEPTED = 0,
    MAV_RESULT_TEMPORARILY_REJECTED = 1,
    MAV_RESULT_DENIED = 2,
    MAV_RESULT_UNSUPPORTED = 3,
    MAV_RESULT_FAILED = 4,
};

enum MAV_FRAME : uint8_t {
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
};

enum MAV_CMD : uint16_t {
    MAV_CMD_COMPONENT_ARM_DISARM = 400,
    MAV_CMD_DO_SET_MODE = 176,
    MAV_CMD_NAV_TAKEOFF = 22,
    MAV_CMD_SET_MESSAGE_INTERVAL = 511,
};

#define MAV_PARAM_TYPE_INT32 6

// Estimator status flags
#define ESTIMATOR_ATTITUDE 1
#define ESTIMATOR_VELOCITY_HORIZ 2
#define ESTIMATOR_VELOCITY_VERT 4
#define ESTIMATOR_POS_HORIZ_ABS 16
#define ESTIMATOR_POS_VERT_ABS 32

typedef struct __mavlink_message {
    uint8_t magic; // should be MAVLINK_STX
    uint8_t len;
    uint8_t incompat_flags;
    uint8_t compat_flags;
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint32_t msgid;
    uint8_t payload[MAVLINK_MAX_PAYLOAD_LEN];
    uint16_t checksum;
} mavlink_message_t;

typedef struct __mavlink_status {
    uint8_t msg_received;
    uint8_t parse_error;
    uint8_t parse_state;
    uint8_t index;
    uint8_t payload_len;
    uint32_t msgid;
    uint8_t buffer[MAVLINK_MAX_PAYLOAD_LEN + 10];
    uint8_t sysid;
    uint8_t compid;
} mavlink_status_t;

static inline uint16_t mavlink_crc_accumulate(uint8_t data, uint16_t crc)
{
    uint8_t tmp = data ^ (uint8_t)(crc & 0xff);
    tmp ^= (tmp << 4);
    return (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

static inline uint16_t mavlink_crc_calculate(const uint8_t *buffer, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc = mavlink_crc_accumulate(buffer[i], crc);
    }
    return crc;
}

// message ids
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_SET_MODE 11
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ 20
#define MAVLINK_MSG_ID_PARAM_SET 23
#define MAVLINK_MSG_ID_PARAM_VALUE 22
#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAVLINK_MSG_ID_COMMAND_ACK 77
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT 86
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
#define MAVLINK_MSG_ID_STATUSTEXT 253
#define MAVLINK_MSG_ID_ESTIMATOR_STATUS 230
#define MAVLINK_MSG_ID_MY_GUIDED_TARGET 512

// payload lengths
#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_SET_MODE_LEN 6
#define MAVLINK_MSG_ID_PARAM_SET_LEN 23
#define MAVLINK_MSG_ID_PARAM_VALUE_LEN 25
#define MAVLINK_MSG_ID_COMMAND_LONG_LEN 33
#define MAVLINK_MSG_ID_COMMAND_ACK_LEN 10
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN 53
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 28
#define MAVLINK_MSG_ID_STATUSTEXT_LEN 54
#define MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN 42
#define MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN 16

// CRC extras (computed from dialect XML)
#define MAVLINK_MSG_HEARTBEAT_CRC 182
#define MAVLINK_MSG_SET_MODE_CRC 177
#define MAVLINK_MSG_PARAM_SET_CRC 211
#define MAVLINK_MSG_PARAM_VALUE_CRC 74
#define MAVLINK_MSG_COMMAND_LONG_CRC 154
#define MAVLINK_MSG_COMMAND_ACK_CRC 92
#define MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_CRC 167
#define MAVLINK_MSG_GLOBAL_POSITION_INT_CRC 23
#define MAVLINK_MSG_STATUSTEXT_CRC 10
#define MAVLINK_MSG_ESTIMATOR_STATUS_CRC 122
#define MAVLINK_MSG_MY_GUIDED_TARGET_CRC 221

// message payload structures
typedef struct __mavlink_heartbeat_t {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} mavlink_heartbeat_t;

typedef struct __mavlink_set_mode_t {
    uint32_t custom_mode;
    uint8_t target_system;
    uint8_t base_mode;
} mavlink_set_mode_t;

typedef struct __mavlink_command_long_t {
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
    float param6;
    float param7;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t confirmation;
} mavlink_command_long_t;

typedef struct __mavlink_command_ack_t {
    uint16_t command;
    uint8_t result;
    uint8_t progress;
    int32_t result_param2;
    uint8_t target_system;
    uint8_t target_component;
} mavlink_command_ack_t;

typedef struct __mavlink_global_position_int_t {
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t hdg;
} mavlink_global_position_int_t;

typedef struct __mavlink_estimator_status_t {
    uint64_t time_usec;
    uint16_t flags;
    float vel_ratio;
    float pos_horiz_ratio;
    float pos_vert_ratio;
    float mag_ratio;
    float hagl_ratio;
    float tas_ratio;
    float pos_horiz_accuracy;
    float pos_vert_accuracy;
} mavlink_estimator_status_t;

typedef struct __mavlink_statustext_t {
    uint8_t severity;
    char text[50];
    uint16_t id;
    uint8_t chunk_seq;
} mavlink_statustext_t;

typedef struct __mavlink_param_set_t {
    float param_value;
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
    uint8_t param_type;
} mavlink_param_set_t;

typedef struct __mavlink_param_value_t {
    float param_value;
    uint16_t param_count;
    uint16_t param_index;
    char param_id[16];
    uint8_t param_type;
} mavlink_param_value_t;

typedef struct __mavlink_set_position_target_global_int_t {
    uint32_t time_boot_ms;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t coordinate_frame;
    uint16_t type_mask;
    int32_t lat_int;
    int32_t lon_int;
    float alt;
    float vx;
    float vy;
    float vz;
    float afx;
    float afy;
    float afz;
    float yaw;
    float yaw_rate;
} mavlink_set_position_target_global_int_t;

typedef struct __mavlink_my_guided_target_t {
    float lat;
    float lon;
    float alt;
    float yaw;
} mavlink_my_guided_target_t;

static inline uint8_t mavlink_get_crc_extra(uint32_t msgid) {
    switch(msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: return MAVLINK_MSG_HEARTBEAT_CRC;
        case MAVLINK_MSG_ID_SET_MODE: return MAVLINK_MSG_SET_MODE_CRC;
        case MAVLINK_MSG_ID_COMMAND_LONG: return MAVLINK_MSG_COMMAND_LONG_CRC;
        case MAVLINK_MSG_ID_COMMAND_ACK: return MAVLINK_MSG_COMMAND_ACK_CRC;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return MAVLINK_MSG_GLOBAL_POSITION_INT_CRC;
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS: return MAVLINK_MSG_ESTIMATOR_STATUS_CRC;
        case MAVLINK_MSG_ID_STATUSTEXT: return MAVLINK_MSG_STATUSTEXT_CRC;
        case MAVLINK_MSG_ID_PARAM_SET: return MAVLINK_MSG_PARAM_SET_CRC;
        case MAVLINK_MSG_ID_PARAM_VALUE: return MAVLINK_MSG_PARAM_VALUE_CRC;
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: return MAVLINK_MSG_SET_POSITION_TARGET_GLOBAL_INT_CRC;
        case MAVLINK_MSG_ID_MY_GUIDED_TARGET: return MAVLINK_MSG_MY_GUIDED_TARGET_CRC;
        default: return 0;
    }
}

static inline uint8_t mavlink_get_payload_len(uint32_t msgid) {
    switch(msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: return MAVLINK_MSG_ID_HEARTBEAT_LEN;
        case MAVLINK_MSG_ID_SET_MODE: return MAVLINK_MSG_ID_SET_MODE_LEN;
        case MAVLINK_MSG_ID_COMMAND_LONG: return MAVLINK_MSG_ID_COMMAND_LONG_LEN;
        case MAVLINK_MSG_ID_COMMAND_ACK: return MAVLINK_MSG_ID_COMMAND_ACK_LEN;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN;
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS: return MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN;
        case MAVLINK_MSG_ID_STATUSTEXT: return MAVLINK_MSG_ID_STATUSTEXT_LEN;
        case MAVLINK_MSG_ID_PARAM_SET: return MAVLINK_MSG_ID_PARAM_SET_LEN;
        case MAVLINK_MSG_ID_PARAM_VALUE: return MAVLINK_MSG_ID_PARAM_VALUE_LEN;
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: return MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN;
        case MAVLINK_MSG_ID_MY_GUIDED_TARGET: return MAVLINK_MSG_ID_MY_GUIDED_TARGET_LEN;
        default: return 0;
    }
}

static inline void mavlink_message_init(mavlink_message_t* msg, uint8_t sysid, uint8_t compid, uint32_t msgid, uint8_t payload_len) {
    msg->magic = MAVLINK_STX;
    msg->len = payload_len;
    msg->incompat_flags = 0;
    msg->compat_flags = 0;
    msg->seq = 0;
    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = msgid;
}

static inline uint16_t mavlink_msg_to_send_buffer(uint8_t *buffer, const mavlink_message_t *msg)
{
    uint8_t payload_len = msg->len;
    buffer[0] = MAVLINK_STX;
    buffer[1] = payload_len;
    buffer[2] = msg->incompat_flags;
    buffer[3] = msg->compat_flags;
    static uint8_t seq = 0;
    buffer[4] = seq++;
    buffer[5] = msg->sysid;
    buffer[6] = msg->compid;
    buffer[7] = msg->msgid & 0xFF;
    buffer[8] = (msg->msgid >> 8) & 0xFF;
    buffer[9] = (msg->msgid >> 16) & 0xFF;
    std::memcpy(&buffer[10], msg->payload, payload_len);
    uint16_t crc = mavlink_crc_calculate(&buffer[1], (size_t)payload_len + 9);
    crc = mavlink_crc_accumulate(mavlink_get_crc_extra(msg->msgid), crc);
    buffer[10 + payload_len] = (uint8_t)(crc & 0xFF);
    buffer[11 + payload_len] = (uint8_t)(crc >> 8);
    return (uint16_t)(payload_len + 12);
}

// Packing helpers
static inline void mavlink_msg_heartbeat_encode(uint8_t sysid, uint8_t compid, mavlink_message_t* msg, const mavlink_heartbeat_t* hb) {
    mavlink_message_init(msg, sysid, compid, MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_HEARTBEAT_LEN);
    uint8_t *p = msg->payload;
    std::memcpy(p, &hb->custom_mode, 4); p += 4;
    *p++ = hb->type; *p++ = hb->autopilot; *p++ = hb->base_mode; *p++ = hb->system_status; *p++ = hb->mavlink_version;
}

static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* hb) {
    const uint8_t *p = msg->payload;
    std::memcpy(&hb->custom_mode, p, 4); p += 4;
    hb->type = *p++;
    hb->autopilot = *p++;
    hb->base_mode = *p++;
    hb->system_status = *p++;
    hb->mavlink_version = *p;
}

static inline void mavlink_msg_set_mode_pack(uint8_t sysid, uint8_t compid, mavlink_message_t* msg, uint8_t target_system, uint8_t base_mode, uint32_t custom_mode) {
    mavlink_message_init(msg, sysid, compid, MAVLINK_MSG_ID_SET_MODE, MAVLINK_MSG_ID_SET_MODE_LEN);
    uint8_t *p = msg->payload;
    std::memcpy(p, &custom_mode, 4); p += 4;
    *p++ = target_system;
    *p++ = base_mode;
}

static inline void mavlink_msg_command_long_pack(uint8_t sysid, uint8_t compid, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float p1, float p2, float p3, float p4, float p5, float p6, float p7) {
    mavlink_message_init(msg, sysid, compid, MAVLINK_MSG_ID_COMMAND_LONG, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
    uint8_t *p = msg->payload;
    std::memcpy(p, &p1, 4); p+=4; std::memcpy(p,&p2,4); p+=4; std::memcpy(p,&p3,4); p+=4; std::memcpy(p,&p4,4); p+=4;
    std::memcpy(p,&p5,4); p+=4; std::memcpy(p,&p6,4); p+=4; std::memcpy(p,&p7,4); p+=4;
    std::memcpy(p,&command,2); p+=2;
    *p++ = target_system;
    *p++ = target_component;
    *p++ = confirmation;
}

static inline void mavlink_msg_set_position_target_global_int_pack(uint8_t sysid, uint8_t compid, mavlink_message_t* msg,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask,
    int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate) {
    mavlink_message_init(msg, sysid, compid, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_LEN);
    uint8_t *p = msg->payload;
    std::memcpy(p,&time_boot_ms,4); p+=4;
    *p++ = target_system;
    *p++ = target_component;
    *p++ = coordinate_frame;
    std::memcpy(p,&type_mask,2); p+=2;
    std::memcpy(p,&lat_int,4); p+=4; std::memcpy(p,&lon_int,4); p+=4; std::memcpy(p,&alt,4); p+=4;
    std::memcpy(p,&vx,4); p+=4; std::memcpy(p,&vy,4); p+=4; std::memcpy(p,&vz,4); p+=4;
    std::memcpy(p,&afx,4); p+=4; std::memcpy(p,&afy,4); p+=4; std::memcpy(p,&afz,4); p+=4;
    std::memcpy(p,&yaw,4); p+=4; std::memcpy(p,&yaw_rate,4);
}

static inline void mavlink_msg_param_set_pack(uint8_t sysid, uint8_t compid, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type) {
    mavlink_message_init(msg, sysid, compid, MAVLINK_MSG_ID_PARAM_SET, MAVLINK_MSG_ID_PARAM_SET_LEN);
    uint8_t *p = msg->payload;
    std::memcpy(p,&param_value,4); p+=4;
    *p++ = target_system;
    *p++ = target_component;
    std::memset(p, 0, 16);
    std::strncpy((char*)p, param_id, 16);
    p += 16;
    *p++ = param_type;
}

// Decoders
static inline void mavlink_msg_global_position_int_decode(const mavlink_message_t* msg, mavlink_global_position_int_t* out) {
    const uint8_t *p = msg->payload;
    std::memcpy(&out->time_boot_ms, p,4); p+=4;
    std::memcpy(&out->lat, p,4); p+=4;
    std::memcpy(&out->lon, p,4); p+=4;
    std::memcpy(&out->alt, p,4); p+=4;
    std::memcpy(&out->relative_alt, p,4); p+=4;
    std::memcpy(&out->vx, p,2); p+=2;
    std::memcpy(&out->vy, p,2); p+=2;
    std::memcpy(&out->vz, p,2); p+=2;
    std::memcpy(&out->hdg, p,2);
}

static inline void mavlink_msg_estimator_status_decode(const mavlink_message_t* msg, mavlink_estimator_status_t* out) {
    const uint8_t *p = msg->payload;
    std::memcpy(&out->time_usec, p,8); p+=8;
    std::memcpy(&out->flags, p,2); p+=2;
    std::memcpy(&out->vel_ratio, p,4); p+=4;
    std::memcpy(&out->pos_horiz_ratio, p,4); p+=4;
    std::memcpy(&out->pos_vert_ratio, p,4); p+=4;
    std::memcpy(&out->mag_ratio, p,4); p+=4;
    std::memcpy(&out->hagl_ratio, p,4); p+=4;
    std::memcpy(&out->tas_ratio, p,4); p+=4;
    std::memcpy(&out->pos_horiz_accuracy, p,4); p+=4;
    std::memcpy(&out->pos_vert_accuracy, p,4); p+=4;
}

static inline void mavlink_msg_command_ack_decode(const mavlink_message_t* msg, mavlink_command_ack_t* out) {
    const uint8_t *p = msg->payload;
    std::memcpy(&out->command, p,2); p+=2;
    out->result = *p++;
    out->progress = *p++;
    std::memcpy(&out->result_param2, p,4); p+=4;
    out->target_system = *p++;
    out->target_component = *p++;
}

static inline void mavlink_msg_statustext_decode(const mavlink_message_t* msg, mavlink_statustext_t* out) {
    const uint8_t *p = msg->payload;
    out->severity = *p++;
    std::memcpy(out->text, p, 50);
    out->text[50-1] = 0;
    p += 50;
    std::memcpy(&out->id, p,2); p+=2;
    out->chunk_seq = *p;
}

static inline void mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* out) {
    const uint8_t *p = msg->payload;
    std::memcpy(&out->param_value, p,4); p+=4;
    std::memcpy(&out->param_count, p,2); p+=2;
    std::memcpy(&out->param_index, p,2); p+=2;
    std::memcpy(out->param_id, p,16); p+=16;
    out->param_id[15] = 0;
    out->param_type = *p;
}

static inline void mavlink_msg_command_long_decode(const mavlink_message_t* msg, mavlink_command_long_t* out) {
    const uint8_t *p = msg->payload;
    std::memcpy(&out->param1,p,4); p+=4; std::memcpy(&out->param2,p,4); p+=4; std::memcpy(&out->param3,p,4); p+=4; std::memcpy(&out->param4,p,4); p+=4;
    std::memcpy(&out->param5,p,4); p+=4; std::memcpy(&out->param6,p,4); p+=4; std::memcpy(&out->param7,p,4); p+=4;
    std::memcpy(&out->command,p,2); p+=2;
    out->target_system = *p++;
    out->target_component = *p++;
    out->confirmation = *p++;
}

static inline int mavlink_check_frame(uint32_t msgid, uint16_t payload_len, uint8_t crc_extra) {
    if (crc_extra == 0) return 0;
    uint8_t expected = mavlink_get_payload_len(msgid);
    return expected == payload_len;
}

static inline uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* status)
{
    (void)chan;
    if (status->parse_state == 0) {
        if (c == MAVLINK_STX) {
            status->parse_state = 1;
            status->index = 0;
            status->msg_received = 0;
        }
        return 0;
    }
    status->buffer[status->index++] = c;
    if (status->index == 1) {
        status->payload_len = c;
        if (status->payload_len > MAVLINK_MAX_PAYLOAD_LEN) { status->parse_state = 0; return 0; }
    }
    if (status->index == 5) status->sysid = c;
    if (status->index == 6) status->compid = c;
    if (status->index == 7) status->msgid = c;
    if (status->index == 8) status->msgid |= ((uint32_t)c) << 8;
    if (status->index == 9) status->msgid |= ((uint32_t)c) << 16;

    uint16_t needed = (uint16_t)status->payload_len + 10; // bytes after magic including crc
    if (status->index >= needed) {
        // compute CRC
        uint16_t recv_crc = (uint16_t)status->buffer[needed-2] | ((uint16_t)status->buffer[needed-1] << 8);
        uint8_t crc_buf[MAVLINK_MAX_PAYLOAD_LEN + 10];
        crc_buf[0] = status->buffer[0]; // len
        crc_buf[1] = status->buffer[1]; // incompat
        crc_buf[2] = status->buffer[2]; // compat
        crc_buf[3] = status->buffer[3]; // seq
        crc_buf[4] = status->sysid;
        crc_buf[5] = status->compid;
        crc_buf[6] = status->msgid & 0xFF;
        crc_buf[7] = (status->msgid >> 8) & 0xFF;
        crc_buf[8] = (status->msgid >> 16) & 0xFF;
        std::memcpy(&crc_buf[9], &status->buffer[9], status->payload_len);
        uint16_t crc = mavlink_crc_calculate(crc_buf, (size_t)status->payload_len + 9);
        crc = mavlink_crc_accumulate(mavlink_get_crc_extra(status->msgid), crc);
        if (crc == recv_crc && mavlink_check_frame(status->msgid, status->payload_len, mavlink_get_crc_extra(status->msgid))) {
            r_message->magic = MAVLINK_STX;
            r_message->len = status->payload_len;
            r_message->incompat_flags = status->buffer[1];
            r_message->compat_flags = status->buffer[2];
            r_message->seq = status->buffer[3];
            r_message->sysid = status->sysid;
            r_message->compid = status->compid;
            r_message->msgid = status->msgid;
            std::memcpy(r_message->payload, &status->buffer[9], status->payload_len);
            r_message->checksum = crc;
            status->msg_received = 1;
            status->parse_state = 0;
            return 1;
        }
        status->parse_error++;
        status->parse_state = 0;
    }
    return 0;
}

