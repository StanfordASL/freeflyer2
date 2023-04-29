#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "ff_msgs/msg/thruster_command.hpp"

using namespace std::chrono_literals;
using ff_msgs::msg::ThrusterCommand;


class TestAllThrustersNode : public rclcpp::Node {
 public:
  TestAllThrustersNode() : rclcpp::Node("test_all_thrusters_node") {
    thrust_cmd_pub_ = this->create_publisher<ThrusterCommand>("commands/duty_cycle", 10);
    timer_ = this->create_wall_timer(5s, std::bind(&TestAllThrustersNode::TimerCallback, this));
    this->declare_parameter("duty_cycle", .2);
  }

 private:
  rclcpp::Publisher<ThrusterCommand>::SharedPtr thrust_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int th_idx_ = 0;

  void TimerCallback() {
    double duty_cycle = this->get_parameter("duty_cycle").as_double();

    // populate thrust msg
    ThrusterCommand msg{};
    for (int i = 0; i < 8; ++i) {
      msg.duty_cycle[i] = 0.;
      if (i == th_idx_) {
        msg.duty_cycle[i] = duty_cycle;
      }
    }

    // publish thrust msg
    this->thrust_cmd_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "opening valve %d", th_idx_);

    // increment th_idx
    th_idx_ = (th_idx_ + 1) % 8;
  }
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestAllThrustersNode>());
  rclcpp::shutdown();
  return 0;
}
