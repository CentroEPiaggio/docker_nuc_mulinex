#include "teleop_mulinex_node.hpp"

namespace teleop_mulinex {

TeleopMulinex::TeleopMulinex() : Node("teleop_mulinex")
{
    this->declare_parameter("linear_step", 0.1);
    this->declare_parameter("angular_step", 0.1);
    this->declare_parameter("body_pos_step", 0.005);
    this->declare_parameter("body_ang_step", 0.02);

    this->get_parameter("linear_step", linear_step_);
    this->get_parameter("angular_step", angular_step_);
    this->get_parameter("body_pos_step", body_pos_step_);
    this->get_parameter("body_ang_step", body_ang_step_);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/ik_controller/base_pose", 1);
    wheel_pub_ = this->create_publisher<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand>(
        "/omni_controller/command", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(40),  // 25 Hz
        std::bind(&TeleopMulinex::teleop_callback, this));

    update_messages();
    print_instructions();
}

void TeleopMulinex::teleop_callback()
{
    char c;
    bool dirty = false;

    try {
        if (keyboard_reader_.read_one(&c)) {
            dirty = process_key(c);
        }
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Read error: %s", e.what());
        return;
    }

    if (dirty) {
        update_messages();
        print_instructions();
    }

    publish_messages();
}

bool TeleopMulinex::process_key(char c)
{
    switch (c) {
    // Wheel velocities
    case KEYCODE_w: wheel_vx_ += linear_step_; return true;
    case KEYCODE_s: wheel_vx_ -= linear_step_; return true;
    case KEYCODE_a: wheel_vy_ += linear_step_; return true;
    case KEYCODE_d: wheel_vy_ -= linear_step_; return true;
    case KEYCODE_u: wheel_omega_ += angular_step_; return true;
    case KEYCODE_o: wheel_omega_ -= angular_step_; return true;

    // Body pose - height
    case KEYCODE_q: body_pos_[2] += body_pos_step_; return true;
    case KEYCODE_e: body_pos_[2] -= body_pos_step_; return true;

    // Body pose - pitch (around Y axis)
    case KEYCODE_i:
        body_quat_ = quat_int(body_quat_, {0.0, -1.0, 0.0}, body_ang_step_);
        return true;
    case KEYCODE_k:
        body_quat_ = quat_int(body_quat_, {0.0, 1.0, 0.0}, body_ang_step_);
        return true;

    // Body pose - roll (around X axis)
    case KEYCODE_j:
        body_quat_ = quat_int(body_quat_, {-1.0, 0.0, 0.0}, body_ang_step_);
        return true;
    case KEYCODE_l:
        body_quat_ = quat_int(body_quat_, {1.0, 0.0, 0.0}, body_ang_step_);
        return true;

    // SPACE: reset everything
    case KEYCODE_SPACE:
        body_pos_ = {0.0, 0.0, 0.0};
        body_quat_ = {0.0, 0.0, 0.0, 1.0};
        wheel_vx_ = wheel_vy_ = wheel_omega_ = 0.0;
        return true;
    }

    return false;
}

void TeleopMulinex::update_messages()
{
    pose_msg_.position.x = body_pos_[0];
    pose_msg_.position.y = body_pos_[1];
    pose_msg_.position.z = body_pos_[2];
    pose_msg_.orientation.x = body_quat_[0];
    pose_msg_.orientation.y = body_quat_[1];
    pose_msg_.orientation.z = body_quat_[2];
    pose_msg_.orientation.w = body_quat_[3];

    wheel_msg_.v_x = wheel_vx_;
    wheel_msg_.v_y = wheel_vy_;
    wheel_msg_.omega = wheel_omega_;
    wheel_msg_.height_rate = 0.0;
}

void TeleopMulinex::publish_messages()
{
    pose_pub_->publish(pose_msg_);
    wheel_pub_->publish(wheel_msg_);
}

void TeleopMulinex::print_instructions()
{
    puts("\033[2J\033[1;1H");  // clear terminal
    puts("Mulinex Teleop - Reading from keyboard");
    puts("---------------------------");
    puts("Wheel velocity commands:");
    puts("  w/s: +/- forward velocity (v_x)");
    puts("  a/d: +/- lateral velocity (v_y)");
    puts("  u/o: +/- angular velocity (omega)");
    puts("---------------------------");
    puts("Body pose commands:");
    puts("  q/e: +/- height (z)");
    puts("  i/k: +/- pitch");
    puts("  j/l: +/- roll");
    puts("---------------------------");
    puts("  SPACE: reset all to zero");
    puts("---------------------------");
    printf("Wheel velocity: vx=%.2f, vy=%.2f, omega=%.2f\n",
           wheel_vx_, wheel_vy_, wheel_omega_);
    printf("Body position: x=%.3f, y=%.3f, z=%.3f\n",
           body_pos_[0], body_pos_[1], body_pos_[2]);
    printf("Body orientation: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f\n",
           body_quat_[0], body_quat_[1], body_quat_[2], body_quat_[3]);
}

} // namespace teleop_mulinex
