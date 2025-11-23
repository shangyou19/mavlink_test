#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include <chrono>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "mavlink/mavlink.h"
#include "mavlink_client.h"

static void usage() {
    std::fprintf(stdout, "Usage: send_mavlink_cpp --conn udp:127.0.0.1:14551 --msg my_guided_target --lat <deg> --lon <deg> --alt <m> --yaw <deg>\n");
}

static bool parse_conn(const std::string &conn, std::string &host, uint16_t &port) {
    if (conn.rfind("udp:", 0) != 0) return false;
    auto rest = conn.substr(4);
    auto pos = rest.find(":");
    if (pos == std::string::npos) return false;
    host = rest.substr(0, pos);
    port = static_cast<uint16_t>(std::stoi(rest.substr(pos+1)));
    return true;
}

int main(int argc, char **argv) {
    uint16_t port = 14551;
    uint8_t target_sys = 1;
    uint8_t target_comp = 1;
    MavlinkClient client;
    // 如需强制解锁可开启：client.force_arm_ = true; 但不建议在真机环境
    if (!client.init(port, 3.0)) {
        return 6;
    }
    if (!client.autoHover100m(target_sys, target_comp)) {
        std::fprintf(stderr, "autoHover100m failed\n");
        return 7;
    }
    std::fprintf(stdout, "Done\n");
    return 0;
}