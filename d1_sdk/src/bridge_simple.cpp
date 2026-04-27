#include <iostream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

int main() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(5005);
    bind(sock, (const sockaddr *)&servaddr, sizeof(servaddr));

    std::cout << "IK-to-D1 Bridge Active. Port 5005." << std::endl;

    char buffer[1024];
    while (true) {
        int n = recvfrom(sock, buffer, 1024, 0, NULL, NULL);
        if (n <= 0) continue;
        buffer[n] = '\0';
        
        std::string json_payload = std::string(buffer);
        std::cout << "Relaying IK: " << json_payload << std::endl;

        // We wrap the JSON in the exact command the D1 expects
        // This runs the binary you already have that doesn't segfault on start
        std::string cmd = "./joint_angle_control '" + json_payload + "' &";
        system(cmd.c_str());
    }
    return 0;
}
