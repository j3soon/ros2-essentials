#include <memory>
#include <thread>

#include "dg3f_driver/dg3f_operator_TCP.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

std::vector<std::string> joint_names = {
    "j_dg_1_1", "j_dg_1_2", "j_dg_1_3", "j_dg_1_4", "j_dg_2_1",
    "j_dg_2_2", "j_dg_2_3", "j_dg_2_4", "j_dg_3_1", "j_dg_3_2"};

class DG3FDriver : public rclcpp::Node {
 public:
  DG3FDriver() : Node("DG3FDriver") {
    this->declare_parameter<std::string>("ip", "169.254.186.72");
    this->declare_parameter<int>("port", 502);

    this->get_parameter("ip", ip_);
    this->get_parameter("port", port_);
    this->get_parameter("joint_prefix", joint_prefix_);

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_joint", 10,
        std::bind(&DG3FDriver::topic_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        50ms, std::bind(&DG3FDriver::timer_callback, this));

    try {
      delto_client_ = std::make_unique<DG3F_TCP>(ip_, port_);
      delto_client_->connect();
    } catch (const boost::system::system_error& e) {
      if (e.code() != boost::asio::error::operation_aborted &&
          e.code() != boost::asio::error::interrupted) {
        throw;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    delto_client_->start_control();
  }

  ~DG3FDriver() {
    // Cancel timers first
    if (timer_) {
      timer_->cancel();
    }

    // delto_client_->disconnect();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    delto_client_.reset();
  }

 private:
  void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // Handle the incoming message

    std::cout << "Received target joint: ";
    for (const auto& value : msg->data) {
      std::cout << value << " ";
    }

    delto_client_->set_position_rad(msg->data);   
  }

  void timer_callback() {
    data = delto_client_->get_data();

    // Publish joint states
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = this->get_clock()->now();

    joint_state.name = joint_names;
    joint_state.position = data.position;
    joint_state.velocity = data.velocity;
    joint_state.effort = data.current;
    publisher_->publish(joint_state);
  }

  std::string ip_;
  int port_;
  std::string joint_prefix_;
  std::unique_ptr<DG3F_TCP> delto_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      subscription_;
  ReceivedData data;
  std::string hand_type_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);

  auto node = std::make_shared<DG3FDriver>();

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
}
