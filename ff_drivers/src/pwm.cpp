#include "ff_drivers/pwm.hpp"

namespace ff {

void PWM::Enable() {
  enabled_ = true;
}

void PWM::Disable() {
  enabled_ = false;
}

void PWM::SetPolarityNormal() {
  polarity_inversed_ = false;
}

void PWM::SetPolarityInverse() {
  polarity_inversed_ = true;
}

void PWM::SetPeriod(const std::chrono::duration<double>& period) {
  period_ = period;
}

void PWM::SetDutyCycle(const std::chrono::duration<double>& duty_cycle) {
  duty_cycle_ = duty_cycle;
}

void PWM::SetDutyCyclePercent(double percent) {
  SetDutyCycle(percent * period_);
}

void PWM::Initialize() {
  Disable();
  SetPolarityNormal();
  SetPeriod(period_);
  SetDutyCycle(duty_cycle_);
}

constexpr int pwmchip[] = { 4, 4, 8, 8 };
constexpr int pwmpin[] = { 0, 1, 0, 1 };

HardPWM::HardPWM(const HardPWM::PinDef& pwm) : pwm_(pwm) {
  // request device
  {
    std::ofstream f_export(ChipBasePath() + "export");
    f_export << pwmpin[pwm_];
  }

  f_enable_.open(PWMBasePath() + "enable");
  f_period_.open(PWMBasePath() + "period");
  f_duty_cycle_.open(PWMBasePath() + "duty_cycle");
  f_polarity_.open(PWMBasePath() + "polarity");

  Initialize();
}

HardPWM::~HardPWM() {
  Disable();

  // close files
  f_enable_.close();
  f_period_.close();
  f_duty_cycle_.close();
  f_polarity_.close();

  // free device
  std::ofstream f_unexport(ChipBasePath() + "unexport");
  f_unexport << pwmpin[pwm_];
}

void HardPWM::Enable() {
  f_enable_ << 1;
  f_enable_.flush();
  PWM::Enable();
}

void HardPWM::Disable() {
  f_enable_ << 0;
  f_enable_.flush();
  PWM::Disable();
}

void HardPWM::SetPolarityNormal() {
  f_polarity_ << "normal";
  f_polarity_.flush();
  PWM::SetPolarityNormal();
}

void HardPWM::SetPolarityInverse() {
  f_polarity_ << "inversed";
  f_polarity_.flush();
  PWM::SetPolarityInverse();
}

void HardPWM::SetPeriod(const std::chrono::duration<double>& period) {
  f_period_ << std::chrono::duration_cast<std::chrono::nanoseconds>(period).count();
  f_period_.flush();
  PWM::SetPeriod(period);
}

void HardPWM::SetDutyCycle(const std::chrono::duration<double>& duty_cycle) {
  f_duty_cycle_ << std::chrono::duration_cast<std::chrono::nanoseconds>(duty_cycle).count();
  f_duty_cycle_.flush();
  PWM::SetDutyCycle(duty_cycle);
}

std::string HardPWM::ChipBasePath() const {
  return "/sys/class/pwm/pwmchip" + std::to_string(pwmchip[pwm_]) + "/";
}

std::string HardPWM::PWMBasePath() const {
  return ChipBasePath() + "pwm" + std::to_string(pwmpin[pwm_]) + "/";
}

SoftPWM::SoftPWM(int pin) : pin_(pin) {
  // request gpio device
  {
    std::ofstream f_export("/sys/class/gpio/export");
    f_export << pin;
  }

  // configure GPIO to be an output
  {
    std::ofstream f_direction(GPIOBasePath() + "direction");
    f_direction << "out";
  }

  f_value_.open(GPIOBasePath() + "value");
  f_active_low_.open(GPIOBasePath() + "active_low");

  Initialize();
}

SoftPWM::~SoftPWM() {
  Disable();
  Low();

  // close files
  f_value_.close();
  f_active_low_.close();

  // free gpio device
  std::ofstream f_unexport("/sys/class/gpio/unexport");
  f_unexport << pin_;
}

void SoftPWM::High() {
  f_value_ << 1;
  f_value_.flush();
}

void SoftPWM::Low() {
  f_value_ << 0;
  f_value_.flush();
}

void SoftPWM::SetPolarityNormal() {
  f_active_low_ << 0;
  f_active_low_.flush();
  PWM::SetPolarityNormal();
}

void SoftPWM::SetPolarityInverse() {
  f_active_low_ << 1;
  f_active_low_.flush();
  PWM::SetPolarityInverse();
}


std::string SoftPWM::GPIOBasePath() const {
  return "/sys/class/gpio/gpio" + std::to_string(pin_) + "/";
}

PWMManager::PWMManager(const std::string& node_name) : rclcpp::Node(node_name) {}

void PWMManager::AddSoftPWM(int pin) {
  pwms_.push_back(std::make_shared<SoftPWM>(pin));
  switch_timers_.push_back(nullptr);
}

void PWMManager::AddHardPWM(const HardPWM::PinDef& pin) {
  pwms_.push_back(std::make_shared<HardPWM>(pin));
  switch_timers_.push_back(nullptr);
}

void PWMManager::EnableAll() {
  for (auto& pwm : pwms_) {
    pwm->Enable();
  }
}

void PWMManager::DisableAll() {
  for (auto& pwm: pwms_) {
    pwm->Disable();
  }
}

void PWMManager::SetPeriodAll(const std::chrono::duration<double>& period) {
  for (auto& pwm: pwms_) {
    pwm->SetPeriod(period);
  }

  soft_period_ = period;
  period_timer_ = this->create_wall_timer(period, std::bind(&PWMManager::PeriodCallback, this));
}

void PWMManager::SetDutyCycle(size_t idx, double duty_cycle_percent) {
  if (idx >= pwms_.size()) {
    RCLCPP_ERROR(this->get_logger(), "PWM idx out of range");
    return;
  }

  if (duty_cycle_percent < 0 || duty_cycle_percent > 1) {
    RCLCPP_WARN(this->get_logger(), "commanded duty cycle = %f clipped", duty_cycle_percent);
    duty_cycle_percent = std::max(std::min(duty_cycle_percent, 1.), 0.);
  }

  pwms_[idx]->SetDutyCyclePercent(duty_cycle_percent);
}

void PWMManager::PeriodCallback() {
  for (size_t i = 0; i < pwms_.size(); ++i) {
    if (!pwms_[i]) {
      RCLCPP_ERROR(this->get_logger(), "encountered unitialized PWM pointer");
      continue;
    }

    if (pwms_[i]->IsSoft() && pwms_[i]->enabled_) {
      pwms_[i]->High();
      switch_timers_[i] = this->create_wall_timer(pwms_[i]->duty_cycle_,
                                                  [this, i]() { SwitchCallback(i); });
    }
  }
}

void PWMManager::SwitchCallback(size_t idx) {
  if (idx >= pwms_.size()) {
    RCLCPP_ERROR(this->get_logger(), "PWM idx out of range");
    return;
  }

  if (!pwms_[idx] || ! switch_timers_[idx]) {
    RCLCPP_ERROR(this->get_logger(), "encountered unitialized pointer");
    return;
  }

  switch_timers_[idx]->cancel();
  pwms_[idx]->Low();
}

}  // namespace ff
