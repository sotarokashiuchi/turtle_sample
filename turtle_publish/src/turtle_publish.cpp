#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

// cmd_velをpublishするノードのクラスを作成
class publish_cmd_vel : public rclcpp::Node{
    public:
        // コンストラクタ：ノード名を"turtle_publish_node"に設定
        publish_cmd_vel() : Node("turtle_publish_node"){
            // 送信するTopic名，変数の型を設定
            publicher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            
            // 2秒ごとにコールバック関数( {}内に記述 )を呼び出す
            timer_ = this->create_wall_timer(2s, [this](){
                // 変数を定義して，値を設定
                auto message = geometry_msgs::msg::Twist();
                message.linear.x = 1.0;
                message.angular.z = 0.5;

                // publishする
                publicher_->publish(message);
            });
        }
    
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publicher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<publish_cmd_vel>());

    // 終了処理
    rclcpp::shutdown();
    return 0;
}