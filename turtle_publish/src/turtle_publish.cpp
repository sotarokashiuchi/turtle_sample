#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("turtle_publisher");

    auto pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

    rclcpp::WallRate loop(0.5);

    while(rclcpp::ok()){
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 1.0;
        msg.angular.z = 1.0;

        RCLCPP_INFO(node->get_logger(), "Publishing: '%f'", msg.linear.x);
        pub_twist->publish(msg);

        loop.sleep();
    }

    rclcpp::shutdown();
    return 0;
}