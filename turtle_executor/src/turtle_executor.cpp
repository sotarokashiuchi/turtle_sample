#include <rclcpp/rclcpp.hpp>
#include <thread>

using std::this_thread::sleep_for;

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        // コンストラクタ
        Node_Class(const std::string &name) : Node(name){
            // どちらか片方のコメントを削除
            // callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            // callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            timer_pub1 = this->create_wall_timer(
                1s, std::bind(&Node_Class::timer_pub_callback1, this), callback_group
            );

            timer_pub2 = this->create_wall_timer(
                3s, std::bind(&Node_Class::timer_pub_callback2, this), callback_group
            );
        }
    
    private:
        // メンバ変数の定義
        rclcpp::TimerBase::SharedPtr timer_pub1;
        rclcpp::TimerBase::SharedPtr timer_pub2;
        rclcpp::CallbackGroup::SharedPtr callback_group;

        void timer_pub_callback1(){
            auto start = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "Start");

            sleep_for(800ms);

            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            RCLCPP_INFO(this->get_logger(), "End (elapsed: %ld ms)", elapsed);
        }

        void timer_pub_callback2(){
            auto start = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "Start");

            sleep_for(2000ms);

            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            RCLCPP_INFO(this->get_logger(), "End (elapsed: %ld ms)", elapsed);
        }
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // Nodeを作成
    auto node1 = std::make_shared<Node_Class>("node1");
    auto node2 = std::make_shared<Node_Class>("node2");

    // どちらか片方のコメントを削除
    // rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();

    // 終了処理
    rclcpp::shutdown();
    return 0;
}