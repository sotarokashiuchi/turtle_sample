#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

#define Kp      0.2
#define ref_x   3.0
#define ref_y   3.0

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        Node_Class() : Node("turtle_subscribe"){
            // Topic publisher
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/turtle1/cmd_vel", 10
            );

            // subscribeを作成
            subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose", 10,
                std::bind(&Node_Class::subscribe_callback_pose,this, std::placeholders::_1)
            );

            // Pulish timer
            timer_pub = this->create_wall_timer(
                500ms, std::bind(&Node_Class::timer_pub_callback, this)
            );
        }
    
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_pub;
        turtlesim::msg::Pose pose;

        void timer_pub_callback(){
            // 目標位置に向かうように速度指令を送る
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = Kp * (ref_x - pose.x);
            message.linear.y = Kp * (ref_y - pose.y);

            publisher_->publish(message);
        }

        void subscribe_callback_pose(const turtlesim::msg::Pose::SharedPtr msg){
            // subscribeで受け取った位置情報を更新
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

    // ノードをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}