#pragma once

#include "omni_utils/quat_math.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_command.hpp"

namespace omni_mulinex_joystick {

class OmniMulinexJoystick: public rclcpp::Node {
public:
    OmniMulinexJoystick();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void joy_callback_pan_tilt(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();
    void timer_callback_pan_tilt();
    void state_callback(const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr msg);
    void deactivate_ik();
    void print_instructions();
    void print_instructions_pan_tilt();

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<pi3hat_moteus_int_msgs::msg::JointsCommand>::SharedPtr joints_command_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<pi3hat_moteus_int_msgs::msg::JointsStates>::SharedPtr state_sub_;

    // Service clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr activate_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr rest_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stand_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ik_reinit_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ik_activate_client_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double sup_vel_x_;
    double sup_vel_y_;
    double sup_omega_;
    double sup_pan_vel_;
    double sup_tilt_vel_;
    double height_step_;
    double yaw_step_;
    double roll_step_;
    double pitch_step_;
    double pos_step_;
    double max_height_;
    double max_roll_;
    double max_pitch_;
    double max_yaw_;
    double max_pos_x_;
    double max_pos_y_;
    double deadzone_;
    int timer_period_ms_;
    bool subscribe_state_;
    std::string mode_;

    // Stored joystick analog inputs
    double joy_left_x_ = 0.0;
    double joy_left_y_ = 0.0;
    double joy_right_x_ = 0.0;
    double joy_right_y_ = 0.0;
    double joy_right_trigger_ = 0.0;
    double joy_left_trigger_ = 0.0;
    bool l2_held_ = false;
    bool ik_active_ = false;

    bool rt_initialized_ = false;
    bool lt_initialized_ = false;

    // State: body pose (accumulated via rate control)
    double body_x_ = 0.0;
    double body_y_ = 0.0;
    double body_height_ = 0.0;
    double body_roll_ = 0.0;
    double body_pitch_ = 0.0;
    double body_yaw_ = 0.0;

    // Edge-detection for buttons
    std::vector<bool> prev_buttons_;

    // Edge-detection for D-pad axes (track previous non-zero state)
    bool prev_dpad_lr_active_ = false;
    bool prev_dpad_ud_active_ = false;
};

} // namespace omni_mulinex_joystick
