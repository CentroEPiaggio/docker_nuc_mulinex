#pragma once

#include "teleop_mulinex/keyboard_reader.hpp"
#include "omni_utils/quat_math.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <array>

#define KEYCODE_w 0x77
#define KEYCODE_s 0x73
#define KEYCODE_a 0x61
#define KEYCODE_d 0x64
#define KEYCODE_q 0x71
#define KEYCODE_e 0x65
#define KEYCODE_i 0x69
#define KEYCODE_k 0x6b
#define KEYCODE_j 0x6a
#define KEYCODE_l 0x6c
#define KEYCODE_u 0x75
#define KEYCODE_o 0x6f
#define KEYCODE_SPACE 0x20

namespace teleop_mulinex {

class TeleopMulinex : public rclcpp::Node {
public:
    TeleopMulinex();
    void shutdown() { keyboard_reader_.shutdown(); }

private:
    void teleop_callback();
    bool process_key(char c);
    void update_messages();
    void publish_messages();
    void print_instructions();

    KeyboardReader keyboard_reader_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double linear_step_;
    double angular_step_;
    double body_pos_step_;
    double body_ang_step_;

    // State: body pose
    std::array<double, 3> body_pos_ = {0.0, 0.0, 0.0};
    std::array<double, 4> body_quat_ = {0.0, 0.0, 0.0, 1.0}; // {x, y, z, w}

    // State: wheel velocities
    double wheel_vx_ = 0.0;
    double wheel_vy_ = 0.0;
    double wheel_omega_ = 0.0;

    // Messages
    geometry_msgs::msg::Pose pose_msg_;
    geometry_msgs::msg::Twist wheel_msg_;
};

} // namespace teleop_mulinex
