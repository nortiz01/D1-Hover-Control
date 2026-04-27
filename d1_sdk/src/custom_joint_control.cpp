#include <iostream>
#include <string>
#include <limits>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>

// Include the compiled DDS message header
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace std;
using namespace unitree::robot;
using namespace unitree::common;

int main() {
    cout << "--- Unitree D1 Arm Interactive Controller ---" << endl;

    // 1. Initialize Network
    ChannelFactory::Instance()->Init(0);

    // 2. Create the DDS Publisher using Unitree's specific syntax
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();
    cout << "[Network] Publisher initialized on topic: " << TOPIC << endl;

    int seq_num = 0; // The arm expects a sequence number

    // 3. Main Interactive Loop
    while (true) {
        int joint_id;
        float target_angle;

        cout << "\nEnter joint ID (e.g., 1 to 6) or '-1' to exit: ";
        cin >> joint_id;

        // Catch non-number inputs to prevent terminal crashing
        if (cin.fail()) {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << "[Error] Invalid input. Please enter numbers only." << endl;
            continue;
        }

        // Exit condition
        if (joint_id == -1) {
            cout << "Exiting controller..." << endl;
            break;
        }

        cout << "Enter target angle: ";
        cin >> target_angle;

        if (cin.fail()) {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << "[Error] Invalid input. Please enter numbers only." << endl;
            continue;
        }

        seq_num++; // Increment sequence for each new command

        // 4. Construct the JSON payload required by the D1 Arm
        // Format: {"seq":X,"address":1,"funcode":1,"data":{"id":Y,"angle":Z,"delay_ms":0}}
        string json_payload = "{\"seq\":" + to_string(seq_num) + 
                              ",\"address\":1,\"funcode\":1,\"data\":{\"id\":" + to_string(joint_id) + 
                              ",\"angle\":" + to_string(target_angle) + 
                              ",\"delay_ms\":0}}";

        // 5. Assign to the string wrapper and publish
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = json_payload;
        publisher.Write(msg);
        
        cout << "[Success] Command sent -> " << json_payload << endl;
        cout << "---------------------------------------------" << endl;
    }

    return 0;
}
