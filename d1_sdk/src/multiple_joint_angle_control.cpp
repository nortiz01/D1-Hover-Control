#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include "arm_command_validator.hpp"
#include "msg/ArmString_.hpp"

using namespace unitree::robot;

namespace {

constexpr const char *kTopic = "rt/arm_Command";
constexpr int kDefaultUdpPort = 8888;
constexpr const char *kDefaultBindAddress = "127.0.0.1";

bool ParseArguments(
    int argc,
    char **argv,
    int &port,
    std::string &bind_address,
    bool &help_requested
)
{
    port = kDefaultUdpPort;
    bind_address = kDefaultBindAddress;
    help_requested = false;
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--help" || argument == "-h") {
            std::cout << "Usage: " << argv[0] << " [--bind IPv4-address] [--port 1-65535]\n";
            help_requested = true;
            return false;
        }
        if (argument == "--bind" && index + 1 < argc) {
            bind_address = argv[++index];
            continue;
        }
        if (argument == "--port" && index + 1 < argc) {
            char *end = nullptr;
            errno = 0;
            const long parsed = std::strtol(argv[++index], &end, 10);
            if (errno != 0 || end == argv[index] || *end != '\0' || parsed < 1 || parsed > 65535) {
                std::cerr << "[Error] UDP port must be an integer between 1 and 65535.\n";
                return false;
            }
            port = static_cast<int>(parsed);
            continue;
        }
        std::cerr << "[Error] Unknown or incomplete argument: " << argument << '\n';
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char **argv)
{
    int udp_port = kDefaultUdpPort;
    std::string bind_address;
    bool help_requested = false;
    if (!ParseArguments(argc, argv, udp_port, bind_address, help_requested)) {
        return help_requested ? 0 : 2;
    }

    // 1. Initialize Unitree DDS on the default network interface.
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(kTopic);
    publisher.InitChannel();

    // 2. Setup UDP Socket
    int sockfd;
    struct sockaddr_in server_addr{};
    struct sockaddr_in client_addr{};
    char buffer[2048];

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "[Error] Socket creation failed: " << std::strerror(errno) << std::endl;
        return 1;
    }

    server_addr.sin_family = AF_INET;
    if (inet_pton(AF_INET, bind_address.c_str(), &server_addr.sin_addr) != 1) {
        std::cerr << "[Error] --bind must be a numeric IPv4 address.\n";
        close(sockfd);
        return 2;
    }
    server_addr.sin_port = htons(static_cast<uint16_t>(udp_port));

    if (bind(sockfd, reinterpret_cast<const struct sockaddr *>(&server_addr), sizeof(server_addr)) < 0) {
        std::cerr << "[Error] Bind failed on " << bind_address << ':' << udp_port
                  << ": " << std::strerror(errno) << std::endl;
        close(sockfd);
        return 1;
    }

    std::cout << "[System] UDP listener active on " << bind_address << ':' << udp_port
              << ". Waiting for validated Python commands..." << std::endl;

    socklen_t len = sizeof(client_addr);
    auto last_log = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    std::size_t accepted_since_log = 0;
    std::size_t oversized_since_log = 0;
    std::size_t rejected_since_log = 0;
    std::string latest_rejection;

    // 3. Main Loop: Listen and Publish
    while (true) {
        len = sizeof(client_addr);
        const ssize_t n = recvfrom(
            sockfd,
            buffer,
            sizeof(buffer) - 1,
            MSG_TRUNC,
            reinterpret_cast<struct sockaddr *>(&client_addr),
            &len
        );
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "[Error] UDP receive failed: " << std::strerror(errno) << std::endl;
            break;
        }
        if (n == 0) {
            continue;
        }

        if (static_cast<std::size_t>(n) >= sizeof(buffer)) {
            ++oversized_since_log;
        } else {
            const std::string json_payload(buffer, static_cast<std::size_t>(n));
            const d1_bridge::ValidationResult validation = d1_bridge::ValidateArmCommand(json_payload);
            if (!validation.ok) {
                ++rejected_since_log;
                latest_rejection = validation.error;
            } else {
                ++accepted_since_log;

                // Instantiate fresh message to prevent pointer corruption by CycloneDDS.
                unitree_arm::msg::dds_::ArmString_ msg{};
                msg.data_() = json_payload;
                publisher.Write(msg);
            }
        }

        const auto current_time = std::chrono::steady_clock::now();
        if (current_time - last_log >= std::chrono::seconds(1)) {
            std::cout << "[UDP] accepted=" << accepted_since_log
                      << " rejected=" << rejected_since_log
                      << " oversized=" << oversized_since_log;
            if (!latest_rejection.empty()) {
                std::cout << " last_rejection=\"" << latest_rejection << '"';
            }
            std::cout << std::endl;
            accepted_since_log = 0;
            oversized_since_log = 0;
            rejected_since_log = 0;
            latest_rejection.clear();
            last_log = current_time;
        }
    }

    close(sockfd);
    return 0;
}
