// MIT License
//
// Copyright (c) 2023 Stanford Autonomous Systems Lab
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#pragma once

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace ff
{

class PWM
{
public:
  virtual void High() {}
  virtual void Low() {}
  virtual bool IsSoft() const = 0;
  virtual void Enable();
  virtual void Disable();
  virtual void SetPolarityNormal();
  virtual void SetPolarityInverse();
  virtual void SetPeriod(const std::chrono::duration<double> & period);
  virtual void SetDutyCycle(const std::chrono::duration<double> & duty_cycle);
  void SetDutyCyclePercent(double percent);
  void Initialize();

private:
  bool enabled_ = false;
  bool polarity_inversed_ = false;
  std::chrono::duration<double> period_ = std::chrono::seconds(0);
  std::chrono::duration<double> duty_cycle_ = std::chrono::seconds(0);

  friend class PWMManager;
};

/**
 * @brief hardware PWM control
 *
 * @see https://wiki.odroid.com/odroid-n2l/application_note/gpio/pwm#tab__odroid-n21 for pin maps
 */
class HardPWM : public PWM
{
public:
  enum PinDef
  {
    PWM_C = 0,
    PWM_D,
    PWM_E,
    PWM_F,
  };

  explicit HardPWM(const PinDef & pwm);
  ~HardPWM();

  bool IsSoft() const override {return false;}
  void Enable() override;
  void Disable() override;
  void SetPolarityNormal() override;
  void SetPolarityInverse() override;
  void SetPeriod(const std::chrono::duration<double> & period) override;
  void SetDutyCycle(const std::chrono::duration<double> & duty_cycle) override;

private:
  std::string ChipBasePath() const;
  std::string PWMBasePath() const;

  const PinDef pwm_;
  std::ofstream f_enable_;
  std::ofstream f_period_;
  std::ofstream f_duty_cycle_;
  std::ofstream f_polarity_;
};

/**
 * @brief software PWM control with GPIO
 *
 * @see https://wiki.odroid.com/odroid-n2l/application_note/gpio/enhancement_40pins#tab__odroid-n2
 *      for pin numbering
 */
class SoftPWM : public PWM
{
public:
  explicit SoftPWM(int pin);
  ~SoftPWM();

  bool IsSoft() const override {return true;}
  void High() override;
  void Low() override;
  void SetPolarityNormal() override;
  void SetPolarityInverse() override;

private:
  int pin_;
  std::ofstream f_value_;
  std::ofstream f_active_low_;

  std::string GPIOBasePath() const;
};

class PWMManager : public rclcpp::Node
{
public:
  explicit PWMManager(const std::string & node_name);

  void AddSoftPWM(int pin);
  void AddHardPWM(const HardPWM::PinDef & pin);

  void EnableAll();
  void DisableAll();
  void SetPeriodAll(const std::chrono::duration<double> & period);
  void SetDutyCycle(size_t idx, double duty_cycle_percent);

private:
  // software PWM control
  std::chrono::duration<double> soft_period_;
  rclcpp::TimerBase::SharedPtr period_timer_;
  std::vector<rclcpp::TimerBase::SharedPtr> switch_timers_;

  std::vector<std::shared_ptr<PWM>> pwms_;

  void PeriodCallback();
  void SwitchCallback(size_t idx);
};

}  // namespace ff
