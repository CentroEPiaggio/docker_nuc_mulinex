#include "omni_mulinex_joystick/omni_mulinex_joystick.hpp"

#include <algorithm>
#include <cmath>

namespace omni_mulinex_joystick {

OmniMulinexJoystick::OmniMulinexJoystick(): Node("omni_mulinex_joystick")
{
    print_instructions();

    // Declare parameters
    this->declare_parameter("sup_vel_x", 1.0);
    this->declare_parameter("sup_vel_y", 1.0);
    this->declare_parameter("sup_omega", 2.0);
    this->declare_parameter("height_step", 0.005);
    this->declare_parameter("yaw_step", 0.02);
    this->declare_parameter("roll_step", 0.02);
    this->declare_parameter("pitch_step", 0.02);
    this->declare_parameter("pos_step", 0.005);
    this->declare_parameter("max_height", 0.05);
    this->declare_parameter("max_roll", 0.3);
    this->declare_parameter("max_pitch", 0.3);
    this->declare_parameter("max_yaw", 0.15);
    this->declare_parameter("max_pos_x", 0.05);
    this->declare_parameter("max_pos_y", 0.05);
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
    pos_step_ = this->get_parameter("pos_step").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    max_roll_ = this->get_parameter("max_roll").as_double();
    max_pitch_ = this->get_parameter("max_pitch").as_double();
    max_yaw_ = this->get_parameter("max_yaw").as_double();
    max_pos_x_ = this->get_parameter("max_pos_x").as_double();
    max_pos_y_ = this->get_parameter("max_pos_y").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();
    timer_period_ms_ = this->get_parameter("timer_period_ms").as_int();
    subscribe_state_ = this->get_parameter("subscribe_state").as_bool();

    // Twist publisher with BestEffort QoS and deadline
    rclcpp::QoS twist_qos(10);
    twist_qos.best_effort();
    twist_qos.deadline(std::chrono::milliseconds(timer_period_ms_ + 5));
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/omni_controller/twist_cmd", twist_qos);

    // Pose publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/ik_controller/base_pose", 1);

    // Joy subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&OmniMulinexJoystick::joy_callback, this, std::placeholders::_1)
    );

    // State subscriber (optional)
    if (subscribe_state_) {
        state_sub_ = this->create_subscription<pi3hat_moteus_int_msgs::msg::JointsStates>(
            "/omni_controller/joints_state", 10,
            std::bind(&OmniMulinexJoystick::state_callback, this, std::placeholders::_1)
        );
    }

    // Service clients
    activate_client_ = this->create_client<std_srvs::srv::SetBool>("/omni_controller/activate_srv");
    emergency_client_ =
        this->create_client<std_srvs::srv::SetBool>("/omni_controller/emergency_srv");
    rest_client_ = this->create_client<std_srvs::srv::SetBool>("/omni_controller/rest_srv");
    stand_client_ = this->create_client<std_srvs::srv::SetBool>("/omni_controller/stand_srv");
    ik_reinit_client_ =
        this->create_client<std_srvs::srv::SetBool>("/ik_controller/reinitialize_srv");
    ik_activate_client_ =
        this->create_client<std_srvs::srv::SetBool>("/ik_controller/activate_srv");

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_ms_),
        std::bind(&OmniMulinexJoystick::timer_callback, this)
    );
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
    auto dz = [this](double val) -> double { return (std::abs(val) < deadzone_) ? 0.0 : val; };

    // Helper: rising edge detection for buttons
    auto btn_rising = [&](size_t idx) -> bool {
        if (idx >= buttons.size())
            return false;
        return buttons[idx] && (idx >= prev_buttons_.size() || !prev_buttons_[idx]);
    };

    // ── Store analog stick values ───────────────────────────────────────
    if (axes.size() > 0)
        joy_left_x_ = dz(axes[0]);
    if (axes.size() > 1)
        joy_left_y_ = dz(axes[1]);
    if (axes.size() > 3)
        joy_right_x_ = dz(axes[3]);
    if (axes.size() > 4)
        joy_right_y_ = dz(axes[4]);

    // L2 modifier (button 6) to activate hardware
    l2_held_ = (buttons.size() > 6) && buttons[6];
    // R2 hold (button 7) used as deadman for publishing twist
    r2_held_ = (buttons.size() > 7) && buttons[7];

    // ── D-pad: base x/y position (edge-triggered, only when IK active) ─
    // D-pad up/down (axes[7]) → x position
    if (axes.size() > 7) {
        bool active = std::abs(axes[7]) > 0.5;
        if (active && !prev_dpad_ud_active_ && ik_active_) {
            double sign = (axes[7] > 0.0) ? 1.0 : -1.0;
            body_x_ = std::clamp(body_x_ + sign * pos_step_, -max_pos_x_, max_pos_x_);
        }
        prev_dpad_ud_active_ = active;
    }

    // D-pad left/right (axes[6]) → y position
    if (axes.size() > 6) {
        bool active = std::abs(axes[6]) > 0.5;
        if (active && !prev_dpad_lr_active_ && ik_active_) {
            double sign = (axes[6] > 0.0) ? 1.0 : -1.0;
            body_y_ = std::clamp(body_y_ + sign * pos_step_, -max_pos_y_, max_pos_y_);
        }
        prev_dpad_lr_active_ = active;
    }

    // ── Services ────────────────────────────────────────────────────────
    // L1 (btn 4) → activate hardware
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

    // □ (btn 3) → activate IK controller (reinitialize + start)
    if (btn_rising(3)) {
        // Reset body pose to zero for clean start
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;

        if (ik_activate_client_->service_is_ready()) {
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = true;
            ik_activate_client_->async_send_request(req);
            ik_active_ = true;
            RCLCPP_INFO(this->get_logger(), "IK controller activated");
        } else {
            RCLCPP_WARN(this->get_logger(), "IK activate service not available");
        }
    }

    // R1 (btn 5) → emergency stop (also deactivates IK)
    if (btn_rising(5)) {
        deactivate_ik();

        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        if (emergency_client_->service_is_ready()) {
            emergency_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "Emergency stop service called");
        } else {
            RCLCPP_WARN(this->get_logger(), "Emergency stop service not available");
        }
    }

    // X (btn 0) → rest (also deactivates IK)
    if (btn_rising(0)) {
        deactivate_ik();

        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        if (rest_client_->service_is_ready()) {
            rest_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "Rest service called");
        } else {
            RCLCPP_WARN(this->get_logger(), "Rest service not available");
        }
    }

    // △ (btn 2) → stand (also deactivates IK)
    if (btn_rising(2)) {
        deactivate_ik();

        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        if (stand_client_->service_is_ready()) {
            stand_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "Stand service called");
        } else {
            RCLCPP_WARN(this->get_logger(), "Stand service not available");
        }
    }

    // ── Resets ──────────────────────────────────────────────────────────
    // L3 (btn 11) → reset wheels
    if (btn_rising(11)) {
        joy_left_x_ = joy_left_y_ = joy_right_x_ = joy_right_y_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Wheels reset");
    }

    // R3 (btn 12) → reset body pose
    if (btn_rising(12)) {
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Body pose reset");
    }

    // PS (btn 10) → reset all
    if (btn_rising(10)) {
        joy_left_x_ = joy_left_y_ = joy_right_x_ = joy_right_y_ = 0.0;
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "All reset");
    }

    // Update previous button state
    prev_buttons_.resize(buttons.size());
    for (size_t i = 0; i < buttons.size(); i++) {
        prev_buttons_[i] = buttons[i];
    }
}

void OmniMulinexJoystick::timer_callback()
{
    // Only publish while deadman (R2) is held. TODO: this culd be configured at launch
    if (!r2_held_)
        return;

    // ── Twist (wheels) ──────────────────────────────────────────────────
    geometry_msgs::msg::Twist twist_msg;
    if (!l2_held_) {
        // Normal: sticks → wheels
        twist_msg.linear.x = -joy_left_y_ * sup_vel_x_;
        twist_msg.linear.y = -joy_left_x_ * sup_vel_y_;
        twist_msg.angular.z = -joy_right_x_ * sup_omega_;

        // Right stick Y → height rate (only when IK active)
        if (ik_active_) {
            body_height_ =
                std::clamp(body_height_ + joy_right_y_ * height_step_, -max_height_, max_height_);
        }
    } else {
        // L2 held: wheels zero
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        // Sticks → body orientation rate (only when IK active)
        if (ik_active_) {
            body_roll_ = std::clamp(body_roll_ + joy_left_x_ * roll_step_, -max_roll_, max_roll_);
            body_pitch_ =
                std::clamp(body_pitch_ + joy_left_y_ * pitch_step_, -max_pitch_, max_pitch_);
            body_yaw_ = std::clamp(body_yaw_ + joy_right_x_ * yaw_step_, -max_yaw_, max_yaw_);
        }
    }
    twist_pub_->publish(twist_msg);

    // ── Pose (body) – only publish when IK is active ────────────────────
    if (ik_active_) {
        auto q = quat_exp_vec({body_roll_, body_pitch_, body_yaw_});

        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = body_x_;
        pose_msg.position.y = body_y_;
        pose_msg.position.z = body_height_;
        pose_msg.orientation.x = q[0];
        pose_msg.orientation.y = q[1];
        pose_msg.orientation.z = q[2];
        pose_msg.orientation.w = q[3];
        pose_pub_->publish(pose_msg);
    }
}

void OmniMulinexJoystick::
    state_callback(const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr /*msg*/)
{
    // State available for future logging/monitoring
}

void OmniMulinexJoystick::deactivate_ik()
{
    if (ik_active_) {
        ik_active_ = false;
        body_x_ = body_y_ = body_height_ = 0.0;
        body_roll_ = body_pitch_ = body_yaw_ = 0.0;

        if (ik_activate_client_->service_is_ready()) {
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = false;
            ik_activate_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "IK controller deactivated");
        }
    }
}

void OmniMulinexJoystick::print_instructions()
{
    RCLCPP_INFO(
        this->get_logger(), "\n\033[32m"
                            "╔════════════════════════════════════════════════════════════╗\n"
                            "║              Omni Mulinex Joystick (PS4)                   ║\n"
                            "╠════════════════════════════════════════════════════════════╣\n"
                            "║  NORMAL MODE                                               ║\n"
                            "║  ───────────                                               ║\n"
                            "║  Left Stick X/Y   → wheel vy / vx                          ║\n"
                            "║  Right Stick X    → wheel omega                            ║\n"
                            "║  Right Stick Y    → base height (rate)                     ║\n"
                            "║                                                            ║\n"
                            "║  L2 + STICKS (body orientation)                            ║\n"
                            "║  ──────────────────────────────                            ║\n"
                            "║  L2 + Left Stick  → roll / pitch (rate)                    ║\n"
                            "║  L2 + Right Stick → yaw (rate)                             ║\n"
                            "║                                                            ║\n"
                            "║  D-PAD (base position)                                     ║\n"
                            "║  ────────────────────                                      ║\n"
                            "║  D-Pad U/D → x position (step)                             ║\n"
                            "║  D-Pad L/R → y position (step)                             ║\n"
                            "║                                                            ║\n"
                            "║  BUTTONS                                                   ║\n"
                            "║  ───────                                                   ║\n"
                            "║  L1 → ACTIVATE HW     R1 → EMERGENCY STOP                  ║\n"
                            "║  HOLD R2 → TAKE CONTROL                                    ║\n"
                            "║  □  → ACTIVATE IK     ✕  → REST                            ║\n"
                            "║  △  → STAND                                                 ║\n"
                            "║  L3 → reset wheels    R3 → reset body pose                 ║\n"
                            "║  PS → reset ALL                                            ║\n"
                            "╚════════════════════════════════════════════════════════════╝"
                            "\033[0m"
    );
}

} // namespace omni_mulinex_joystick
