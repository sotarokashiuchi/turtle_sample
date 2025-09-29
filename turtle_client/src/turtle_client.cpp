#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <std_srvs/srv/empty.hpp>

#define Kp      0.2
#define ref_x   3.0
#define ref_y   3.0

using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        Node_Class() : Node("turtle_client"){
            // Publisher
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/turtle1/cmd_vel", 10
            );

            // Subscriber
            subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose", 10,
                std::bind(&Node_Class::subscribe_callback_pose,this, std::placeholders::_1)
            );

            // Publish timer
            timer_pub = this->create_wall_timer(
                500ms, std::bind(&Node_Class::timer_pub_callback, this)
            );

            // ServiceのClientを作成
            client_reset_ = this->create_client<std_srvs::srv::Empty>("reset");
        }
        
        // Service呼び出し関数
        void call_reset_service(){
            // リグエストを作成
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();

            // Serverにリクエストを送信
            auto result = client_reset_->async_send_request(request);
        }
    
    private:
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_reset_;
        rclcpp::TimerBase::SharedPtr timer_pub;
        turtlesim::msg::Pose pose;

        void timer_pub_callback(){
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = Kp * (ref_x - pose.x);
            message.linear.y = Kp * (ref_y - pose.y);

            publisher_->publish(message);
        }

        void subscribe_callback_pose(const turtlesim::msg::Pose::SharedPtr msg){
            pose.x = msg->x;
            pose.y = msg->y;
        }
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // ノードを作成
    auto node = std::make_shared<Node_Class>();
    rclcpp::sleep_for(1s);

    // サービスの呼び出し
    node->call_reset_service();

    // ノードをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}