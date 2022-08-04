#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "motors_controller.hpp"

using std::placeholders::_1;


MotorsController::MotorsController()
        : Node("minimal_subscriber") {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorsController::topic_callback, this, _1));
}

void MotorsController::topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear.x);

    auto out = sensor_msgs::msg::JointState();
    out.header.stamp = this->get_clock()->now();

    out.name.resize(2);
    out.name[0] = "left_wheel_joint";
    out.name[1] = "right_wheel_joint";
    out.position.resize(2);
    out.position[0] = 0;
    out.position[1] = 0;
    out.velocity.resize(2);
    out.velocity[0] = 1;
    out.velocity[1] = 1;

    publisher_->publish(out);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorsController>());
    rclcpp::shutdown();
    return 0;
}
