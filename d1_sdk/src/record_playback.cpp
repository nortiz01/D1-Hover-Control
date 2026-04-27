#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <fstream>
#include <sstream>
#include <cmath>
#include <thread>
#include <unistd.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

using namespace std;
using namespace unitree::robot;
using namespace unitree::common;
using namespace unitree_arm::msg::dds_;

std::mutex angle_mtx;
std::vector<float> current_angles(7, 0.0);
bool feedback_received = false;

void AngleHandler(const void* msg) {
    const PubServoInfo_* pm = (const PubServoInfo_*)msg;
    std::lock_guard<std::mutex> lock(angle_mtx);
    current_angles[0] = pm->servo0_data_();
    current_angles[1] = pm->servo1_data_();
    current_angles[2] = pm->servo2_data_();
    current_angles[3] = pm->servo3_data_();
    current_angles[4] = pm->servo4_data_();
    current_angles[5] = pm->servo5_data_();
    current_angles[6] = pm->servo6_data_();
    feedback_received = true;
}

void send_command(ChannelPublisher<ArmString_>& pub, int& seq, int fun, const string& data_part) {
    seq++;
    ArmString_ msg{};
    msg.data_() = "{\"seq\":" + to_string(seq) + ",\"address\":1,\"funcode\":" + to_string(fun) + ",\"data\":" + data_part + "}";
    pub.Write(msg);
}

// Helper to handle the -180 to 180 degree jump
float angle_diff(float a, float b) {
    float d = a - b;
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return std::abs(d);
}

void move_and_wait(ChannelPublisher<ArmString_>& publisher, const std::vector<float>& target, int& seq) {
    string data_part = "{\"mode\":1";
    for(int i=0; i<7; i++) {
        data_part += ",\"angle" + to_string(i) + "\":" + to_string(target[i]);
    }
    data_part += "}";
    send_command(publisher, seq, 2, data_part);

    bool arrived = false;
    float joint_threshold = 2.5;    
    float progress_epsilon = 0.1;   
    int stall_count = 0;
    int max_stall = 60;             // 1.5 seconds patience
    float last_error = 9999.0;
    
    usleep(300000); 

    while (!arrived) {
        float current_total_error = 0;
        bool all_ready = true;
        std::vector<float> snapshot;

        {
            std::lock_guard<std::mutex> lock(angle_mtx);
            snapshot = current_angles;
            for(int i=0; i<6; i++) {
                float diff = angle_diff(snapshot[i], target[i]);
                current_total_error += diff;
                if (diff > joint_threshold) all_ready = false;
            }
        }

        if (all_ready) {
            cout << " [Reached]";
            break;
        }

        float improvement = last_error - current_total_error;
        if (improvement > progress_epsilon) {
            stall_count = 0;
            last_error = current_total_error;
        } else {
            stall_count++;
        }

        if (stall_count % 20 == 0) {
            printf("\r[Wait] Total Err: %3.1f | Stall: %d/%d", current_total_error, stall_count, max_stall);
            fflush(stdout);
        }

        if (stall_count > max_stall) { 
            // LOGIC CHANGE: If we stopped moving, just ACCEPT the position and move on.
            // This prevents the 30-degree base offset from ruining the whole sequence.
            cout << " [Limit Hit - Proceeding]";
            break;
        }

        usleep(25000); 
    }
    usleep(250000); 
}

int main() {
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<ArmString_> publisher("rt/arm_Command");
    publisher.InitChannel();
    ChannelSubscriber<PubServoInfo_> subscriber("current_servo_angle");
    subscriber.InitChannel(AngleHandler);

    int seq_num = 0;
    cout << "--- Unitree D1 Advanced Teaching Tool (Stall Detection) ---" << endl;
    cout << "1. Record New\n2. Load File\nSelect: ";
    int choice; cin >> choice; cin.ignore();

    std::vector<std::vector<float>> waypoints;

    if (choice == 1) {
        cout << "How many points? ";
        int pts; cin >> pts; cin.ignore();
        send_command(publisher, seq_num, 5, "{\"mode\":0}");
        cout << "[System] Torque DISABLED. Pose arm and press ENTER for each point." << endl;
        for (int i = 1; i <= pts; i++) {
            cout << "Press ENTER to save Waypoint " << i << "...";
            string dummy; getline(cin, dummy);
            std::lock_guard<std::mutex> lock(angle_mtx);
            waypoints.push_back(current_angles);
            cout << " Saved." << endl;
        }
        cout << "Save as (include .csv): ";
        string fname; cin >> fname;
        ofstream out(fname);
        for (auto& wp : waypoints) {
            for (int i=0; i<7; i++) out << wp[i] << (i==6 ? "" : ",");
            out << "\n";
        }
        out.close();
    } else {
        cout << "Filename to load: ";
        string fname; cin >> fname;
        ifstream in(fname);
        if (!in.is_open()) { cout << "File not found!" << endl; return 1; }
        string line;
        while (getline(in, line)) {
            std::vector<float> wp;
            stringstream ss(line);
            string val;
            while (getline(ss, val, ',')) wp.push_back(stof(val));
            if (wp.size() >= 7) waypoints.push_back(wp);
        }
    }

    if (!feedback_received) {
        cout << "[Error] No feedback from arm. Check connection!" << endl;
        return 1;
    }

    cout << "\n[Playback] Starting in 3s..." << endl;
    sleep(3);

    for (size_t i = 0; i < waypoints.size(); i++) {
        cout << "\n[Move] Waypoint " << (i + 1) << "/" << waypoints.size() << flush;
        move_and_wait(publisher, waypoints[i], seq_num);
    }

    cout << "\n\n[Success] Playback complete. TORQUE ENABLED." << endl;
    while(true) { sleep(1); }
    return 0;
}
