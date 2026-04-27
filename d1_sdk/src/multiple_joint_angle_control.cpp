#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"
#define UDP_PORT 8888

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    // 1. Initialize Unitree DDS on the default network interface.
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    // 2. Setup UDP Socket
    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    char buffer[2048];

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "[Error] Socket creation failed." << std::endl;
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    memset(&client_addr, 0, sizeof(client_addr));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(UDP_PORT);

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "[Error] Bind failed on port " << UDP_PORT << std::endl;
        close(sockfd);
        return -1;
    }

    std::cout << "[System] UDP Listener active on port " << UDP_PORT << ". Waiting for Python IK data..." << std::endl;

    socklen_t len = sizeof(client_addr);

    // 3. Main Loop: Listen and Publish
    while (true) {
        int n = recvfrom(sockfd, (char *)buffer, sizeof(buffer) - 1, MSG_WAITALL, (struct sockaddr *)&client_addr, &len);
        if (n > 0) {
            buffer[n] = '\0';
            std::string json_payload(buffer);
            std::cout << "[Received] " << json_payload << std::endl;

            // Instantiate fresh message to prevent pointer corruption by CycloneDDS
            unitree_arm::msg::dds_::ArmString_ msg{};
            msg.data_() = json_payload;
            
            publisher.Write(msg);
        }
    }

    close(sockfd);
    return 0;
}
