#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        // コンストラクタ
        Node_Class() : Node("turtle_subscribe"){
            // turtle1/cmd_velへgeometry_msgs::msg::Twist型の信号を送信するPublisherを作成
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "turtle1/cmd_vel", 10
            );

            // turtle1/poseからturtlesim::msg::Pose型の信号を受信するSubscriberを作成
            // subscribe時にsubscribe_callback_poseを実行
            subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
                "turtle1/pose", 10,
                std::bind(&Node_Class::subscribe_callback_pose,this, std::placeholders::_1)
            );

            // 500ms周期でtimer_pub_callbackを実行するタイマーを作成
            timer_pub = this->create_wall_timer(
                500ms, std::bind(&Node_Class::timer_pub_callback, this)
            );
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_pub;
        turtlesim::msg::Pose pose;
        double Kp=0.2;
        double ref_x=3.0;
        double ref_y=3.0;

        // タイマー呼び出し関数（周期的にPublish）
        void timer_pub_callback(){
            // Publishするメッセージを作成
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = Kp * (ref_x - pose.x);
            message.linear.y = Kp * (ref_y - pose.y);

            // TopicへPublish
            publisher_->publish(message);
        }

        // Subscriber呼び出し関数（Subscribe時に実行）
        void subscribe_callback_pose(const turtlesim::msg::Pose::SharedPtr msg){
            pose.x = msg->x;
            pose.y = msg->y;
        }
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // Nodeを作成
    auto node = std::make_shared<Node_Class>();

    // Nodeをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}