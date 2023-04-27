#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "ff_drivers/pwm.hpp"

using namespace std::chrono_literals;


class TestHardSingleNode : public ff::PWMManager {
 public:
  TestHardSingleNode() : ff::PWMManager("test_single_node") {
    bool use_hard = this->declare_parameter("use_hard", true);
    this->declare_parameter("max_duty_cycle", .2);

    if (use_hard) {
      RCLCPP_INFO(this->get_logger(), "Using hardware PWM: PWM_C");
      this->AddHardPWM(ff::HardPWM::PWM_C);
    } else {
      RCLCPP_INFO(this->get_logger(), "Using software PWM: pin 477");
      this->AddSoftPWM(477);
    }

    this->SetPeriodAll(100ms);
    this->EnableAll();

    timer_ = this->create_wall_timer(5s, [this]() { TimerCallback(); });
  }

 private:
  void TimerCallback() {
    double max_duty_cycle = this->get_parameter("max_duty_cycle").as_double();

    this->SetDutyCycle(0, cnt_ * max_duty_cycle * .1);
    RCLCPP_INFO(this->get_logger(), "Dutycycle set to %f", cnt_ * max_duty_cycle * .1);
    cnt_ = (cnt_ + 1) % 11;
  }

  int cnt_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestHardSingleNode>());
  rclcpp::shutdown();
  return 0;
}
