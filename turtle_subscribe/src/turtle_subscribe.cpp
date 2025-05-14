#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

#define Kp 0.1
#define ref_x 3.0
#define ref_y 3.0

// グローバル変数を宣言
rclcpp::Node::SharedPtr node = nullptr;
turtlesim::msg::Pose pose_common;

//　subscribe時に実行されるコールバック関数
void callback_sub_pose(const turtlesim::msg::Pose::SharedPtr msg){
    // 受信したメッセージをグローバル変数に格納
    pose_common.x = msg->x;
    pose_common.y = msg->y;
    pose_common.theta = msg->theta;
    
    RCLCPP_INFO(node->get_logger(), "Subscribe Pose: x=%f, y=%f, theta=%f", pose_common.x, pose_common.y, pose_common.theta);
}

int main(int argc, char **argv){
    // ROS2を初期化
    rclcpp::init(argc, argv);

    // ノードを作成
    node = rclcpp::Node::make_shared("turtle_subscribe");

    // publisherとsubscriberを登録
    auto pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    auto sub_pose = node->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, callback_sub_pose);
    
    // publish用のメッセージを作成
    auto cmd = geometry_msgs::msg::Twist();

    // loop rateを設定
    rclcpp::WallRate loop(0.5); // 2Hz

    while(rclcpp::ok()){
        // Callback関数を1回だけ実行
        rclcpp::spin_some(node);

        // メッセージをpublish
        pub_twist->publish(cmd);

        // 目標位置に向けて速度を計算
        cmd.linear.x = Kp*( ref_x - pose_common.x );
        cmd.linear.y = Kp*( ref_y - pose_common.y );

        // Display the message in the console
        RCLCPP_INFO(node->get_logger(), "Publish: vel_x=%f, vel_y=%f", cmd.linear.x, cmd.linear.y);

        // loop rateまで待機
        loop.sleep();
    }

    rclcpp::shutdown();
    return 0;
}