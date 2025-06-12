#include <memory>
#include <thread>


#include <iostream>
#include "dg5f_driver/dg5f_operator_TCP.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

std::vector<std::string> joint_names_left = {
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4", "lj_dg_2_1",
    "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4", "lj_dg_3_1", "lj_dg_3_2",
    "lj_dg_3_3", "lj_dg_3_4", "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3",
    "lj_dg_4_4", "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4"};

std::vector<std::string> joint_names_right = {
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4", "rj_dg_2_1",
    "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4", "rj_dg_3_1", "rj_dg_3_2",
    "rj_dg_3_3", "rj_dg_3_4", "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3",
    "rj_dg_4_4", "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4"};

class dg5fDriver : public rclcpp::Node {
 public:
  dg5fDriver() : Node("dg5FDriver") {
    this->declare_parameter<std::string>("ip", "169.254.186.72");
    this->declare_parameter<int>("port", 502);
    this->declare_parameter<std::string>("hand_type", "right");

    this->get_parameter("ip", ip_);
    this->get_parameter("port", port_);
    this->get_parameter("joint_prefix", joint_prefix_);
    this->get_parameter("hand_type", hand_type_);

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/target_joint", 10,
        std::bind(&dg5fDriver::topic_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        50ms, std::bind(&dg5fDriver::timer_callback, this));

    try {
      delto_client_ = std::make_unique<DG5F_TCP>(ip_, port_);
      delto_client_->connect();
    } catch (const boost::system::system_error& e) {
      if (e.code() != boost::asio::error::operation_aborted &&
          e.code() != boost::asio::error::interrupted) {
        throw;
      }
    }
  }

  ~dg5fDriver() {
    // Cancel timers first
    if (timer_) {
      timer_->cancel();
    }

    // delto_client_->disconnect();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    delto_client_.reset();
  }

 private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Handle the incoming message

    std::cout << "Received target joint: ";
    for (const auto& value : msg->data) {
      std::cout << value << " ";
    }

    delto_client_->start_control();
    std::cout << std::endl;
  }

  void timer_callback() {
    data = delto_client_->get_data();
    RCLCPP_INFO(this->get_logger(), "Received data from DG5F");
    // Publish joint states
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = this->get_clock()->now();

    if (hand_type_ == "left") {
      joint_state.name = joint_names_left;
    } else if (hand_type_ == "right") {
      joint_state.name = joint_names_right;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid hand type: %s", hand_type_.c_str());
      return;
    }
    joint_state.position = data.position;
    joint_state.velocity = data.velocity;
    joint_state.effort = data.current;
    publisher_->publish(joint_state);
  }

  std::string ip_;
  int port_;
  std::string joint_prefix_;
  std::unique_ptr<DG5F_TCP> delto_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscription_;
  ReceivedData data;
  std::string hand_type_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);

  auto node = std::make_shared<dg5fDriver>();

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
