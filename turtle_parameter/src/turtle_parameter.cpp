#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/set_pen.hpp>

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        // コンストラクタ
        Node_Class() : Node("turtle_parameter"){
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

            // パラメータの宣言
            this->declare_parameter("Kp", 0.2);
            this->declare_parameter("ref_x", 3.0);
            this->declare_parameter("ref_y", 3.0);

            // パラメータの取得
            this->get_parameter("Kp", Kp);
            this->get_parameter("ref_x", ref_x);
            this->get_parameter("ref_y", ref_y);

            // turtle1/set_penへturtlesim::srv::SetPen型のRequestを送りServiceを呼び出すClientを作成
            client_setpen_ = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
        }
        
        // Service呼び出し関数
        void call_setpen_service(){
            // Serviceに送るRequestを作成
            auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
            request->r = 255;
            request->g = 0;
            request->b = 0;
            request->width = 5;
            request->off = 0;

            // ServiceへRequestを送信
            auto result = client_setpen_->async_send_request(request);
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_setpen_;
        rclcpp::TimerBase::SharedPtr timer_pub;
        turtlesim::msg::Pose pose;
        double Kp;
        double ref_x;
        double ref_y;

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
    rclcpp::sleep_for(1s);

    // Serviceの呼び出し
    node->call_setpen_service();

    // Nodeをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}