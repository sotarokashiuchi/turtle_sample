#include <rclcpp/rclcpp.hpp>
#include <thread>

using std::this_thread::sleep_for;

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        // コンストラクタ
        Node_Class(const std::string &name) : Node(name){
            // 500ms周期でtimer_pub_callbackを実行するタイマーを作成
            timer_pub = this->create_wall_timer(
                1s, std::bind(&Node_Class::timer_pub_callback, this)
            );
        }
    
    private:
        // メンバ変数の定義
        rclcpp::TimerBase::SharedPtr timer_pub;

        // タイマー呼び出し関数（周期的にPublish）
        void timer_pub_callback(){
            // Publishするメッセージを作成
            auto start = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "Start");

            // 2秒間CPUを使う重い処理
            sleep_for(800ms);

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