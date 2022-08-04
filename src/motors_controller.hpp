#ifndef MOBILE_ROBOT_TUTORIAL_MOTORS_CONTROLLER_HPP
#define MOBILE_ROBOT_TUTORIAL_MOTORS_CONTROLLER_HPP


class MotorsController : public rclcpp::Node {
public:
    MotorsController();

private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

#endif //MOBILE_ROBOT_TUTORIAL_MOTORS_CONTROLLER_HPP
