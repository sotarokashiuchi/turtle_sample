#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

// cmd_velをpublishするノードのクラスを定義
class Publish_cmd_vel : public rclcpp::Node{
    public:
        // コンストラクタ：ノード名を"turtle_publish_node"に設定
        explicit Publish_cmd_vel() : Node("turtle_publish_node"){
            // Qosの設定：KeepLast(10)は，最新の10個のメッセージを保持する
            rclcpp::QoS qos(rclcpp::KeepLast(10));

            // 送信するTopic名，変数の型を設定
            publicher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/turtle1/cmd_vel", qos
            );
            
            // 2secごとに呼び出すコールバック関数
            publish_message = this->create_wall_timer(2s, [this](){
                // 変数を定義して，値を設定
                auto message = geometry_msgs::msg::Twist();
                message.linear.x = 1.0;
                message.angular.z = 1.0;

                // publishする
                publicher_->publish(message);
            });
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publicher_;
        rclcpp::TimerBase::SharedPtr publish_message;
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // ノードを作成
    auto node_pub_cmd_vel = std::make_shared<Publish_cmd_vel>();
    // ノードをspinする
    rclcpp::spin(node_pub_cmd_vel);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}