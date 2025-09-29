#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        // コンストラクタ
        Node_Class() : Node("turtle_publish"){
            // TopicのPublisherを作成
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/turtle1/cmd_vel", 10
            );

            // 500ms周期でtimer_pub_callbackを実行するタイマーを作成
            timer_pub = this->create_wall_timer(
                1.5s, std::bind(&Node_Class::timer_pub_callback, this)
            );
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_pub;
        turtlesim::msg::Pose pose;

        void timer_pub_callback(){
            // 速度指令を作成
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = 1.0;
            message.angular.z = 1.0;

            // 速度指令をpublish
            publisher_->publish(message);
        }
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // ノードを作成
    auto node = std::make_shared<Node_Class>();

    // ノードをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}