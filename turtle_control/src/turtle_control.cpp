#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <std_srvs/srv/empty.hpp>

// #define Kp      0.2
// #define ref_x   3.0
// #define ref_y   3.0

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        Node_Class() : Node("turtle_control"){
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/turtle1/cmd_vel", 10
            );

            timer_pub = this->create_wall_timer(
                500ms, std::bind(&Node_Class::timer_pub_callback, this)
            );

            subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose", 10,
                std::bind(&Node_Class::subscribe_callback_pose,this, std::placeholders::_1)
            );

            this->declare_parameter("Kp", 0.2);
            this->declare_parameter("ref_x", 3.0);
            this->declare_parameter("ref_y", 3.0);
            
            // Kp = this->get_parameter("Kp").as_double();
            // ref_x = this->get_parameter("ref_x").as_double();
            // ref_y = this->get_parameter("ref_y").as_double();

            timer_param = this->create_wall_timer(
                5s, std::bind(&Node_Class::timer_param_callback, this)
            );
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_pub;
        rclcpp::TimerBase::SharedPtr timer_param;
        double Kp;
        double ref_x;
        double ref_y;

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

        void timer_param_callback(){
            Kp = this->get_parameter("Kp").as_double();
            ref_x = this->get_parameter("ref_x").as_double();
            ref_y = this->get_parameter("ref_y").as_double();
        }   
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // ノードを作成
    auto node = std::make_shared<Node_Class>();

    // サービスクライアントの作成
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_reset =
        node->create_client<std_srvs::srv::Empty>("reset");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    // サービス起動確認
    while((!client_reset->wait_for_service(1s))){
        if (!rclcpp::ok()){
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }

    // サービスのリクエストを送信
    auto result = client_reset->async_send_request(request);

    // サービスのリクエストが完了するまで待機
    if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(node->get_logger(), "Service \"/reset\" succeeded");
    }

    // ノードをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}