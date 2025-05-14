#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

int main(int argc, char **argv){
    // Initialize the ROS2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("turtle_publish");

    // Create a publisher
    auto pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

    // Create a timer to publish messages at a fixed rate
    rclcpp::WallRate loop(0.5); // 2Hz

    while(rclcpp::ok()){
        // Create a message
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 1.0;
        msg.angular.z = 1.0;

        // Display the message in the console
        RCLCPP_INFO(node->get_logger(), "Publishing: '%f'", msg.linear.x);

        // Publish the message
        pub_twist->publish(msg);

        // Spin the node to process callbacks
        loop.sleep();
    }

    rclcpp::shutdown();
    return 0;
}