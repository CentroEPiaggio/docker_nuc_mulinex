#include "omni_mulinex_joystick/omni_mulinex_joystick.hpp"

#include <algorithm>
#include <cmath>

namespace omni_mulinex_joystick {

OmniMulinexJoystick::OmniMulinexJoystick() : Node("omni_mulinex_joystick")
{
    // Declare parameters
    this->declare_parameter("sup_vel_x", 1.0);
    this->declare_parameter("sup_vel_y", 1.0);
    this->declare_parameter("sup_omega", 2.0);
    this->declare_parameter("height_step", 0.005);
    this->declare_parameter("yaw_step", 0.02);
    this->declare_parameter("roll_step", 0.02);
    this->declare_parameter("pitch_step", 0.02);
    this->declare_parameter("max_height", 0.05);
    this->declare_parameter("max_roll", 0.3);
    this->declare_parameter("max_pitch", 0.3);
    this->declare_parameter("max_yaw", 0.15);
    this->declare_parameter("deadzone", 0.1);
    this->declare_parameter("timer_period_ms", 100);
    this->declare_parameter("subscribe_state", true);

    // Read parameters
    sup_vel_x_ = this->get_parameter("sup_vel_x").as_double();
    sup_vel_y_ = this->get_parameter("sup_vel_y").as_double();
    sup_omega_ = this->get_parameter("sup_omega").as_double();
    height_step_ = this->get_parameter("height_step").as_double();
    yaw_step_ = this->get_parameter("yaw_step").as_double();
    roll_step_ = this->get_parameter("roll_step").as_double();
    pitch_step_ = this->get_parameter("pitch_step").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    max_roll_ = this->get_parameter("max_roll").as_double();
    max_pitch_ = this->get_parameter("max_pitch").as_double();
    max_yaw_ = this->get_parameter("max_yaw").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();
    timer_period_ms_ = this->get_parameter("timer_period_ms").as_int();
    subscribe_state_ = this->get_parameter("subscribe_state").as_bool();

    // Twist publisher with BestEffort QoS and deadline
    rclcpp::QoS twist_qos(10);
    twist_qos.best_effort();
    twist_qos.deadline(std::chrono::milliseconds(timer_period_ms_ + 5));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/omni_controller/twist_cmd", twist_qos);

    // Pose publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/ik_controller/base_pose", 1);

    // Joy subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&OmniMulinexJoystick::joy_callback, this, std::placeholders::_1));

    // State subscriber (optional)
    if (subscribe_state_) {
        state_sub_ = this->create_subscription<pi3hat_moteus_int_msgs::msg::JointsStates>(
            "/omni_controller/joints_state", 10,
            std::bind(&OmniMulinexJoystick::state_callback, this, std::placeholders::_1));
    }

    // Service clients
    activate_client_ = this->create_client<std_srvs::srv::SetBool>(
        "/omni_controller/activate_srv");
    emergency_client_ = this->create_client<std_srvs::srv::SetBool>(
        "/omni_controller/emergency_srv");

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms_),
        std::bind(&OmniMulinexJoystick::timer_callback, this));

    print_instructions();
}

void OmniMulinexJoystick::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    const auto& axes = msg->axes;
    const auto& buttons = msg->buttons;

    // Initialize previous button state on first message
    if (prev_buttons_.empty()) {
        prev_buttons_.resize(buttons.size(), false);
    }

    // Helper: apply deadzone
    auto dz = [this](double val) -> double {
        return (std::abs(val) < deadzone_) ? 0.0 : val;
    };

    // Helper: rising edge detection for buttons
    auto btn_rising = [&](size_t idx) -> bool {
        if (idx >= buttons.size()) return false;
        bool rising = buttons[idx] && (idx >= prev_buttons_.size() || !prev_buttons_[idx]);
        return rising;
    };

    // ── Wheel analog ────────────────────────────────────────────────────
    if (axes.size() > 1) wheel_vx_ = dz(axes[1]) * sup_vel_x_;
    if (axes.size() > 0) wheel_vy_ = dz(axes[0]) * sup_vel_y_;
    if (axes.size() > 3) wheel_omega_ = dz(axes[3]) * sup_omega_;

    // ── Body incremental (edge-triggered) ───────────────────────────────

    // D-pad left/right (axes[6]) → roll
    if (axes.size() > 6) {
        bool active = std::abs(axes[6]) > 0.5;
        if (active && !prev_dpad_lr_active_) {
            double sign = (axes[6] > 0.0) ? 1.0 : -1.0;
            body_roll_ = std::clamp(body_roll_ + sign * roll_step_, -max_roll_, max_roll_);
        }
        prev_dpad_lr_active_ = active;
    }

    // D-pad up/down (axes[7]) → pitch
    if (axes.size() > 7) {
        bool active = std::abs(axes[7]) > 0.5;
        if (active && !prev_dpad_ud_active_) {
            double sign = (axes[7] > 0.0) ? 1.0 : -1.0;
            body_pitch_ = std::clamp(body_pitch_ + sign * pitch_step_, -max_pitch_, max_pitch_);
        }
        prev_dpad_ud_active_ = active;
    }

    // △ (btn 2) → height up, ✕ (btn 0) → height down
    if (btn_rising(2)) {
        body_height_ = std::clamp(body_height_ + height_step_, -max_height_, max_height_);
    }
    if (btn_rising(0)) {
        body_height_ = std::clamp(body_height_ - height_step_, -max_height_, max_height_);
    }

    // □ (btn 3) → yaw left, ○ (btn 1) → yaw right
    if (btn_rising(3)) {
        body_yaw_ = std::clamp(body_yaw_ + yaw_step_, -max_yaw_, max_yaw_);
    }
    if (btn_rising(1)) {
        body_yaw_ = std::clamp(body_yaw_ - yaw_step_, -max_yaw_, max_yaw_);
    }

    // ── Resets ──────────────────────────────────────────────────────────
    // L3 (btn 11) → reset wheels
    if (btn_rising(11)) {
        wheel_vx_ = wheel_vy_ = wheel_omega_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Wheels reset");
    }

    // R3 (btn 12) → reset body pose
    if (btn_rising(12)) {
        body_height_ = body_roll_ = body_pitch_ = body_yaw_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Body pose reset");
    }

    // PS (btn 10) → reset all
    if (btn_rising(10)) {
        wheel_vx_ = wheel_vy_ = wheel_omega_ = 0.0;
        body_height_ = body_roll_ = body_pitch_ = body_yaw_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "All reset");
    }

    // ── Services ────────────────────────────────────────────────────────
    // L1 (btn 4) → activate
    if (btn_rising(4)) {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        if (activate_client_->service_is_ready()) {
            activate_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "Activate service called");
        } else {
            RCLCPP_WARN(this->get_logger(), "Activate service not available");
        }
    }

    // R1 (btn 5) → emergency stop
    if (btn_rising(5)) {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        if (emergency_client_->service_is_ready()) {
            emergency_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "Emergency stop service called");
        } else {
            RCLCPP_WARN(this->get_logger(), "Emergency stop service not available");
        }
    }

    // Update previous button state
    prev_buttons_.resize(buttons.size());
    for (size_t i = 0; i < buttons.size(); i++) {
        prev_buttons_[i] = buttons[i];
    }
}

void OmniMulinexJoystick::timer_callback()
{
    // Publish twist
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = wheel_vx_;
    twist_msg.linear.y = wheel_vy_;
    twist_msg.angular.z = wheel_omega_;
    twist_pub_->publish(twist_msg);

    // Build quaternion from axis-angle vector (roll, pitch, yaw)
    auto q = quat_exp_vec({body_roll_, body_pitch_, body_yaw_});

    // Publish pose
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.z = body_height_;
    pose_msg.orientation.x = q[0];
    pose_msg.orientation.y = q[1];
    pose_msg.orientation.z = q[2];
    pose_msg.orientation.w = q[3];
    pose_pub_->publish(pose_msg);
}

void OmniMulinexJoystick::state_callback(
    const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr /*msg*/)
{
    // State available for future logging/monitoring
}

void OmniMulinexJoystick::print_instructions()
{
    // ANSI green
    const char* g = "\033[32m";
    const char* r = "\033[0m";

    printf("\n%s", g);
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║              Omni Mulinex Joystick (PS4)                   ║\n");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║  LEFT SIDE (locomotion)       RIGHT SIDE (body pose)       ║\n");
    printf("║  ─────────────────────        ──────────────────────       ║\n");
    printf("║  Left Stick Y  → vx          D-Pad L/R   → roll  (step)  ║\n");
    printf("║  Left Stick X  → vy          D-Pad U/D   → pitch (step)  ║\n");
    printf("║  Right Stick X → omega       △ / ✕       → height ±      ║\n");
    printf("║                               □ / ○       → yaw ±        ║\n");
    printf("║                                                            ║\n");
    printf("║  L1 → ACTIVATE               R1 → EMERGENCY STOP          ║\n");
    printf("║  L3 → reset wheels            R3 → reset body pose        ║\n");
    printf("║  PS → reset ALL                                            ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("%s\n", r);
}

} // namespace omni_mulinex_joystick
