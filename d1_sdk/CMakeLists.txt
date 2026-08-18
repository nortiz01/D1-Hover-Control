cmake_minimum_required(VERSION 3.5)
project(d1_sdk)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(
    D1_BUILD_LEGACY_BRIDGE
    "Build the deprecated bridge_simple compatibility executable"
    OFF
)

find_package(unitree_sdk2 REQUIRED)

add_library(d1_arm_msgs
    src/msg/ArmString_.cpp
    src/msg/PubServoInfo_.cpp
    src/msg/SetServoAngle_.cpp
    src/msg/SetServoDumping_.cpp
)
target_link_libraries(d1_arm_msgs PUBLIC unitree_sdk2)

add_library(d1_command_validator src/arm_command_validator.cpp)
target_include_directories(d1_command_validator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(arm_zero_control src/arm_zero_control.cpp)
if(D1_BUILD_LEGACY_BRIDGE)
    add_executable(bridge_simple src/bridge_simple.cpp)
endif()
add_executable(custom_joint_control src/custom_joint_control.cpp)
add_executable(get_arm_joint_angle src/get_arm_joint_angle.cpp)
add_executable(joint_angle_control src/joint_angle_control.cpp)
add_executable(joint_enable_control src/joint_enable_control.cpp)
add_executable(multiple_joint_angle_control src/multiple_joint_angle_control.cpp)
add_executable(record_playback src/record_playback.cpp)

target_link_libraries(arm_zero_control PRIVATE d1_arm_msgs)
target_link_libraries(custom_joint_control PRIVATE d1_arm_msgs)
target_link_libraries(get_arm_joint_angle PRIVATE d1_arm_msgs)
target_link_libraries(joint_angle_control PRIVATE d1_arm_msgs)
target_link_libraries(joint_enable_control PRIVATE d1_arm_msgs)
target_link_libraries(multiple_joint_angle_control PRIVATE d1_arm_msgs d1_command_validator)
target_link_libraries(record_playback PRIVATE d1_arm_msgs)
