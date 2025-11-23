#include "mavlink_client.h"
#include <cstdio>
#include <cstring>

MavlinkClient::MavlinkClient()
    : sock_(-1), system_id_(255), component_id_(MAV_COMP_ID_MISSIONPLANNER), force_arm_(false), hb_running_(false), guided_enabled_(false), armed_(false), custom_mode_(0) {
    std::memset(&bind_addr_, 0, sizeof(bind_addr_));
    std::memset(&remote_, 0, sizeof(remote_));
}

MavlinkClient::~MavlinkClient() {
    stopHeartbeat();
    if (sock_ >= 0) ::close(sock_);
}

bool MavlinkClient::init(uint16_t port, double recv_timeout_sec) {
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) { perror("socket"); return false; }
    bind_addr_.sin_family = AF_INET;
    bind_addr_.sin_port = htons(port);
    bind_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(sock_, (sockaddr*)&bind_addr_, sizeof(bind_addr_)) != 0) { perror("bind"); return false; }
    timeval tv{}; tv.tv_sec = (int)recv_timeout_sec; tv.tv_usec = (int)((recv_timeout_sec - tv.tv_sec)*1e6);
    ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t rbuf[2048]; socklen_t rlen = sizeof(remote_);
    ssize_t rn = ::recvfrom(sock_, rbuf, sizeof(rbuf), 0, (sockaddr*)&remote_, &rlen);
    if (rn <= 0) { std::fprintf(stderr, "No telemetry on udp:*:%u, cannot determine return path\n", port); return false; }
    sendHeartbeat();
    return true;
}

bool MavlinkClient::sendMessage(const mavlink_message_t &msg) {
    uint8_t buffer[512];
    unsigned len = mavlink_msg_to_send_buffer(buffer, &msg);
    ssize_t n = ::sendto(sock_, buffer, len, 0, (sockaddr*)&remote_, sizeof(remote_));
    return n == (ssize_t)len;
}

bool MavlinkClient::setModeGuided(uint8_t target_sys) {
    mavlink_message_t m{};
    float param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
    float param2 = 4; // GUIDED custom mode for ArduCopter
    mavlink_msg_command_long_pack(system_id_, component_id_, &m, target_sys, target_sys,
                                  MAV_CMD_DO_SET_MODE, 0,
                                  param1, param2, 0, 0, 0, 0, 0);
    if (!sendMessage(m)) { perror("sendto set_mode"); return false; }
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t rbuf2[2048]; sockaddr_in r2{}; socklen_t rl2 = sizeof(r2);
        ssize_t n = ::recvfrom(sock_, rbuf2, sizeof(rbuf2), 0, (sockaddr*)&r2, &rl2);
        if (n <= 0) continue;
        mavlink_message_t rx; mavlink_status_t st{};
        for (ssize_t k=0;k<n;++k) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rbuf2[k], &rx, &st)) {
                if (rx.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                    mavlink_command_ack_t ack; mavlink_msg_command_ack_decode(&rx, &ack);
                    if (ack.command == MAV_CMD_DO_SET_MODE) {
                        if (ack.result == MAV_RESULT_ACCEPTED) {
                            guided_enabled_ = true;
                            custom_mode_ = 4;
                            return true;
                        }
                        std::fprintf(stderr, "GUIDED set_mode rejected: result=%u\n", (unsigned)ack.result);
                        return false;
                    }
                }
            }
        }
    }
    std::fprintf(stderr, "GUIDED set_mode no ACK within timeout\n");
    return false;
}

bool MavlinkClient::arm(uint8_t target_sys, uint8_t target_comp, double ack_timeout_sec, bool force) {
    mavlink_message_t m{};
    float param2 = force ? 21196 /*force arm magic*/ : 0;
    mavlink_msg_command_long_pack(system_id_, component_id_, &m, target_sys, target_comp, MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                  1,param2,0,0,0,0,0);
    if (!sendMessage(m)) { perror("sendto arm"); return false; }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(ack_timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t rbuf2[2048]; sockaddr_in r2{}; socklen_t rl2 = sizeof(r2);
        ssize_t n = ::recvfrom(sock_, rbuf2, sizeof(rbuf2), 0, (sockaddr*)&r2, &rl2);
        if (n <= 0) break;
        mavlink_message_t rx; mavlink_status_t st{};
        for (ssize_t k=0;k<n;++k) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rbuf2[k], &rx, &st)) {
                if (rx.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                    mavlink_command_ack_t ack; mavlink_msg_command_ack_decode(&rx, &ack);
                    std::fprintf(stdout, "ARM ACK command=%u result=%u\n", (unsigned)ack.command, (unsigned)ack.result);
                    if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                        if (ack.result == MAV_RESULT_ACCEPTED) {
                            armed_ = true;
                            return true;
                        }
                        return false;
                    }
                }
            }
        }
    }
    std::fprintf(stderr, "ARM no ACK within timeout\n");
    return false; // require acks to ensure vehicle state
}

bool MavlinkClient::waitGlobal(double sec, mavlink_global_position_int_t &out) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(sec);
    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t rbuf2[2048]; sockaddr_in r2{}; socklen_t rl2 = sizeof(r2);
        ssize_t n = ::recvfrom(sock_, rbuf2, sizeof(rbuf2), 0, (sockaddr*)&r2, &rl2);
        if (n <= 0) continue;
        mavlink_message_t rx; mavlink_status_t st{};
        for (ssize_t k=0;k<n;++k) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rbuf2[k], &rx, &st)) {
                if (rx.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_msg_global_position_int_decode(&rx, &out);
                    return true;
                }
            }
        }
    }
    return false;
}

bool MavlinkClient::waitRelativeAltGE(float alt_m, double timeout_sec, mavlink_global_position_int_t &out) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t rbuf2[2048]; sockaddr_in r2{}; socklen_t rl2 = sizeof(r2);
        ssize_t n = ::recvfrom(sock_, rbuf2, sizeof(rbuf2), 0, (sockaddr*)&r2, &rl2);
        if (n <= 0) continue;
        mavlink_message_t rx; mavlink_status_t st{};
        for (ssize_t k=0;k<n;++k) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rbuf2[k], &rx, &st)) {
                if (rx.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_msg_global_position_int_decode(&rx, &out);
                    float rel_alt = out.relative_alt * 0.001f; // mm -> m
                    if (rel_alt >= alt_m) return true;
                }
            }
        }
    }
    return false;
}

bool MavlinkClient::setPositionTargetGlobalHover(uint8_t target_sys, uint8_t target_comp, const mavlink_global_position_int_t &gp, float alt_rel) {
    int32_t lat_int = gp.lat;
    int32_t lon_int = gp.lon;
    uint32_t time_boot_ms = 0;
    uint8_t coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    uint16_t type_mask = (1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<9)|(1<<10);
    float vx=0,vy=0,vz=0,afx=0,afy=0,afz=0,yaw=0,yaw_rate=0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (std::chrono::steady_clock::now() < deadline) {
        mavlink_message_t m{};
        mavlink_msg_set_position_target_global_int_pack(system_id_, component_id_, &m,
            time_boot_ms, target_sys, target_comp, coordinate_frame, type_mask,
            lat_int, lon_int, alt_rel, vx, vy, vz, afx, afy, afz, yaw, yaw_rate);
        if (!sendMessage(m)) { perror("sendto set_position_target_global_int"); return false; }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    return true;
}

bool MavlinkClient::autoHover100m(uint8_t target_sys, uint8_t target_comp) {
    startHeartbeat(1.0f);
    // 确保可以获取到定位/状态数据
    requestMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000, target_sys, target_comp);
    requestMessageInterval(MAVLINK_MSG_ID_ESTIMATOR_STATUS, 200000, target_sys, target_comp);
    waitEkfHealthy(5.0);
    if (!setModeGuided(target_sys)) return false;
    sendHeartbeat();
    if (!arm(target_sys, target_comp, 1.0, force_arm_)) return false;
    sendHeartbeat();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    mavlink_global_position_int_t gp{};
    if (!waitGlobal(2.0, gp)) {
        std::fprintf(stderr, "No global position received before takeoff\n");
        return false;
    }
    {
        float p1=0,p2=0,p3=0,p4=0;
        // 传入当前位置经纬度，使用相对高度100m
        float p5 = gp.lat/1e7f;
        float p6 = gp.lon/1e7f;
        float p7 = 100.0f;

        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            mavlink_message_t m{};
            mavlink_msg_command_long_pack(system_id_, component_id_, &m, target_sys, target_comp, MAV_CMD_NAV_TAKEOFF, 0,
                                          p1,p2,p3,p4,p5,p6,p7);
            if (!sendMessage(m)) { perror("sendto takeoff"); return false; }

            auto ack_wait = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
            while (std::chrono::steady_clock::now() < ack_wait) {
                uint8_t rbuf2[2048]; sockaddr_in r2{}; socklen_t rl2 = sizeof(r2);
                ssize_t n = ::recvfrom(sock_, rbuf2, sizeof(rbuf2), 0, (sockaddr*)&r2, &rl2);
                if (n <= 0) continue;
                mavlink_message_t rx; mavlink_status_t st{};
                for (ssize_t k=0;k<n;++k) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, rbuf2[k], &rx, &st)) {
                        if (rx.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                            mavlink_command_ack_t ack; mavlink_msg_command_ack_decode(&rx, &ack);
                            if (ack.command == MAV_CMD_NAV_TAKEOFF) {
                                std::fprintf(stdout, "TAKEOFF ACK command=%u result=%u\n", (unsigned)ack.command, (unsigned)ack.result);
                                if (ack.result == MAV_RESULT_ACCEPTED) goto takeoff_ack_done2;
                                return false;
                            }
                        }
                    }
                }
            }
        }
        std::fprintf(stderr, "TAKEOFF no ACK within timeout\n");
        return false;
        takeoff_ack_done2:;
    }
    // 等待相对高度接近目标再切换位置保持
    mavlink_global_position_int_t gp_after{};
    if (!waitRelativeAltGE(95.0f, 10.0, gp_after)) {
        std::fprintf(stderr, "Relative altitude did not reach takeoff target\n");
        return false;
    }
    return setPositionTargetGlobalHover(target_sys, target_comp, gp_after, 100.0f);
}
void MavlinkClient::sendHeartbeat() {
    mavlink_message_t m{};
    mavlink_heartbeat_t hb{};
    hb.type = MAV_TYPE_GCS;
    hb.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (guided_enabled_) base |= MAV_MODE_FLAG_GUIDED_ENABLED;
    if (armed_) base |= MAV_MODE_FLAG_SAFETY_ARMED;
    hb.base_mode = base;
    hb.custom_mode = custom_mode_.load();
    hb.system_status = armed_ ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    mavlink_msg_heartbeat_encode(system_id_, component_id_, &m, &hb);
    (void)sendMessage(m);
}
void MavlinkClient::startHeartbeat(float hz) {
    if (hb_running_) return;
    hb_running_ = true;
    hb_thread_ = std::thread([this,hz]{
        auto period = std::chrono::duration<double>(1.0/hz);
        while (hb_running_) {
            sendHeartbeat();
            std::this_thread::sleep_for(period);
        }
    });
}
void MavlinkClient::stopHeartbeat() {
    if (!hb_running_) return;
    hb_running_ = false;
    if (hb_thread_.joinable()) hb_thread_.join();
}
bool MavlinkClient::setParamInt(const char* name, int32_t value, uint8_t target_sys, uint8_t target_comp, double wait_sec) {
    mavlink_message_t m{};
    mavlink_msg_param_set_pack(system_id_, component_id_, &m, target_sys, target_comp, name, *(float*)&value, MAV_PARAM_TYPE_INT32);
    if (!sendMessage(m)) { perror("sendto param_set"); return false; }
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t rbuf2[2048]; sockaddr_in r2{}; socklen_t rl2 = sizeof(r2);
        ssize_t n = ::recvfrom(sock_, rbuf2, sizeof(rbuf2), 0, (sockaddr*)&r2, &rl2);
        if (n <= 0) continue;
        mavlink_message_t rx; mavlink_status_t st{};
        for (ssize_t k=0;k<n;++k) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rbuf2[k], &rx, &st)) {
                if (rx.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
                    return true;
                }
            }
        }
    }
    return true;
}
bool MavlinkClient::requestMessageInterval(uint16_t msgid, int32_t interval_us, uint8_t target_sys, uint8_t target_comp) {
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(system_id_, component_id_, &m, target_sys, target_comp, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                  (float)msgid, *(float*)&interval_us, 0,0,0,0,0);
    return sendMessage(m);
}

void MavlinkClient::drainStatus(double sec) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(sec);
    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t rbuf2[2048]; sockaddr_in r2{}; socklen_t rl2 = sizeof(r2);
        ssize_t n = ::recvfrom(sock_, rbuf2, sizeof(rbuf2), 0, (sockaddr*)&r2, &rl2);
        if (n <= 0) continue;
        mavlink_message_t rx; mavlink_status_t st{};
        for (ssize_t k=0;k<n;++k) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rbuf2[k], &rx, &st)) {
                if (rx.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                    mavlink_statustext_t stx; mavlink_msg_statustext_decode(&rx, &stx);
                    char buf[51]{}; std::memcpy(buf, stx.text, sizeof(stx.text)); buf[50]=0;
                    std::fprintf(stdout, "ST[%u]: %s\n", (unsigned)stx.severity, buf);
                }
            }
        }
    }
}
bool MavlinkClient::waitEkfHealthy(double sec) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(sec);
    uint16_t flags_good = (uint16_t)(ESTIMATOR_ATTITUDE | ESTIMATOR_VELOCITY_HORIZ | ESTIMATOR_VELOCITY_VERT | ESTIMATOR_POS_HORIZ_ABS | ESTIMATOR_POS_VERT_ABS);
    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t rbuf2[2048]; sockaddr_in r2{}; socklen_t rl2 = sizeof(r2);
        ssize_t n = ::recvfrom(sock_, rbuf2, sizeof(rbuf2), 0, (sockaddr*)&r2, &rl2);
        if (n <= 0) continue;
        mavlink_message_t rx; mavlink_status_t st{};
        for (ssize_t k=0;k<n;++k) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rbuf2[k], &rx, &st)) {
                if (rx.msgid == MAVLINK_MSG_ID_ESTIMATOR_STATUS) {
                    mavlink_estimator_status_t ekf; mavlink_msg_estimator_status_decode(&rx, &ekf);
                    if ((ekf.flags & flags_good) == flags_good && ekf.pos_horiz_ratio > 0.2f && ekf.pos_vert_ratio > 0.2f) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}