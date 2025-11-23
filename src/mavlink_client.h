#pragma once
#include <cstdint>
#include <chrono>
#include <string>
#include <thread>
#include <atomic>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "mavlink/mavlink.h"

class MavlinkClient {
public:
    MavlinkClient();
    ~MavlinkClient();
    bool init(uint16_t port, double recv_timeout_sec);
    bool autoHover100m(uint8_t target_sys, uint8_t target_comp);

private:
    bool sendMessage(const mavlink_message_t &msg);
    bool setModeGuided(uint8_t target_sys);
    bool arm(uint8_t target_sys, uint8_t target_comp, double ack_timeout_sec, bool force);
    bool waitGlobal(double sec, mavlink_global_position_int_t &out);
    bool setPositionTargetGlobalHover(uint8_t target_sys, uint8_t target_comp, const mavlink_global_position_int_t &gp, float alt_rel);
    void sendHeartbeat();
    bool setParamInt(const char* name, int32_t value, uint8_t target_sys, uint8_t target_comp, double wait_sec);
    bool requestMessageInterval(uint16_t msgid, int32_t interval_us, uint8_t target_sys, uint8_t target_comp);
    void drainStatus(double sec);
    void startHeartbeat(float hz);
    void stopHeartbeat();
    bool waitEkfHealthy(double sec);
    bool waitRelativeAltGE(float alt_m, double timeout_sec, mavlink_global_position_int_t &out);

    int sock_;
    sockaddr_in bind_addr_;
    sockaddr_in remote_;
    uint8_t system_id_;
    uint8_t component_id_;
    bool force_arm_;
    std::atomic<bool> hb_running_;
    std::thread hb_thread_;
    std::atomic<bool> guided_enabled_;
    std::atomic<bool> armed_;
    std::atomic<uint32_t> custom_mode_;
};