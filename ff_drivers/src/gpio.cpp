// MIT License
//
// Copyright (c) 2024 Stanford Autonomous Systems Lab
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


#include "ff_drivers/gpio.hpp"

namespace ff
{

GPIO::GPIO(int pin)
: pin_(pin)
{
  // request gpio device
  {
    std::ofstream f_export("/sys/class/gpio/export");
    f_export << pin;
  }

  // configure GPIO to be an output
  {
    std::ofstream f_direction(BasePath() + "direction");
    f_direction << "out";
  }

  f_value_.open(BasePath() + "value");
  f_active_low_.open(BasePath() + "active_low");

  SetPolarity(true);
  SetState(false);
}

GPIO::~GPIO()
{
  SetState(false);

  // close files
  f_value_.close();
  f_active_low_.close();

  // free gpio device
  std::ofstream f_unexport("/sys/class/gpio/unexport");
  f_unexport << pin_;
}

void GPIO::SetState(bool state)
{
  f_value_ << static_cast<int>(state);
  f_value_.flush();
  state_ = state;
}

bool GPIO::GetState() const
{
  return state_;
}

void GPIO::SetPolarity(bool normal)
{
  f_active_low_ << static_cast<int>(!normal);
  f_active_low_.flush();
}

std::string GPIO::BasePath() const
{
  return "/sys/class/gpio/gpio" + std::to_string(pin_) + "/";
}

}  // namespace ff
