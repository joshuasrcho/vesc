/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#include "vesc_hw_interface/vesc_servo_controller.hpp"
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

namespace vesc_hw_interface
{
VescServoController::VescServoController() : num_motor_pole_pairs_(1), gear_ratio_(1.0), torque_const_(1.0)
{
}

VescServoController::~VescServoController()
{
  interface_ptr_->setDutyCycle(0.0);
}

void VescServoController::init(hardware_interface::HardwareInfo& info, VescInterface* interface_ptr)
{
  // initializes members
  if (interface_ptr == NULL)
  {
    rclcpp::shutdown();
  }
  else
  {
    interface_ptr_ = interface_ptr;
  }

  calibration_flag_ = true;
  zero_position_ = 0.0;
  error_integ_ = 0.0;

  // reads parameters
  Kp_ = std::stod(info.hardware_parameters["servo/Kp"]);
  Ki_ = std::stod(info.hardware_parameters["servo/Ki"]);
  Kd_ = std::stod(info.hardware_parameters["servo/Kd"]);
  // control_rate_ = std::stod(info.hardware_parameters["servo/control_rate"]);
  // control_period_ = 1.0 / control_rate_;
  calibration_current_ = std::stod(info.hardware_parameters["servo/calibration_current"]);
  calibration_duty_ = std::stod(info.hardware_parameters["servo/calibration_duty"]);
  calibration_mode_ = info.hardware_parameters["servo/calibration_mode"];
  calibration_position_ = std::stod(info.hardware_parameters["servo/calibration_position"]);
  speed_limit_ = std::stod(info.hardware_parameters["servo/speed_limit"]);

  // shows parameters
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[Servo Gains] P: %f, I: %f, D: %f", Kp_, Ki_, Kd_);
  if (calibration_mode_ == CURRENT_)
  {
    RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[Servo Calibration] Mode: %s, value: %f", CURRENT_.data(),
                calibration_current_);
  }
  else if (calibration_mode_ == DUTY_)
  {
    RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[Servo Calibration] Mode: %s, value: %f", DUTY_.data(),
                calibration_duty_);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("VescHwInterface"), "[Servo Calibration] Invalid mode");
  }
  // Create timer callback for PID servo control
  // control_timer_ =
  //     rclcpp::create_timer(rclcpp::Duration(control_period_), &VescServoController::controlTimerCallback, this);
  return;
}

void VescServoController::control(const double position_reference, const double position_current)
{
  auto clock = rclcpp::Clock(RCL_ROS_TIME);
  // executes caribration
  if (calibration_flag_)
  {
    calibrate(position_current);
    // initializes/resets control variables
    time_previous_ = clock.now();
    position_sens_previous_ = position_current;
    position_reference_ = calibration_position_;
    position_reference_previous_ = calibration_position_;
    error_previous_ = 0.0;
    return;
  }

  const rclcpp::Time time_current = clock.now();
  // calculates PD control
  const double error_current = position_reference - position_current;
  const double u_pd = Kp_ * error_current + Kd_ * (error_current - error_previous_) / control_period_;

  double u = 0.0;

  // calculates I control if PD input is not saturated
  if (isSaturated(u_pd))
  {
    u = saturate(u_pd);
  }
  else
  {
    double error_integ_new = error_integ_ + (error_current + error_previous_) / 2.0 * control_period_;
    const double u_pid = u_pd + Ki_ * error_integ_new;

    // not use I control if PID input is saturated
    // since error integration causes bugs
    if (isSaturated(u_pid))
    {
      u = saturate(u_pid);
    }
    else
    {
      u = u_pid;
      error_integ_ = error_integ_new;
    }
  }

  // updates previous data
  error_previous_ = error_current;
  time_previous_ = time_current;
  position_sens_previous_ = position_current;
  position_reference_previous_ = position_reference;

  // command duty
  interface_ptr_->setDutyCycle(u);
  return;
}

void VescServoController::setTargetPosition(const double position)
{
  position_target_ = position;
}

void VescServoController::setGearRatio(const double gear_ratio)
{
  gear_ratio_ = gear_ratio;
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[VescServoController]Gear ratio is set to %f", gear_ratio_);
}

void VescServoController::setTorqueConst(const double torque_const)
{
  torque_const_ = torque_const;
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[VescServoController]Torque constant is set to %f",
              torque_const_);
}

void VescServoController::setMotorPolePairs(const int motor_pole_pairs)
{
  num_motor_pole_pairs_ = motor_pole_pairs;
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[VescServoController]The number of motor pole pairs is set to %d",
              num_motor_pole_pairs_);
}

double VescServoController::getZeroPosition() const
{
  return zero_position_;
}

double VescServoController::getPositionSens(void)
{
  if (calibration_flag_)
  {
    return calibration_position_;
  }
  return position_sens_;
}

double VescServoController::getVelocitySens(void)
{
  return velocity_sens_;
}

double VescServoController::getEffortSens(void)
{
  return effort_sens_;
}

void VescServoController::executeCalibration()
{
  calibration_flag_ = true;
  return;
}

bool VescServoController::calibrate(const double position_current)
{
  static double position_previous;
  static uint16_t step = 0;

  // sends a command for calibration
  if (calibration_mode_ == CURRENT_)
  {
    interface_ptr_->setCurrent(calibration_current_);
  }
  else if (calibration_mode_ == DUTY_)
  {
    interface_ptr_->setDutyCycle(calibration_duty_);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("VescHwInterface"), "Please set the calibration mode surely");
    return false;
  }

  step++;

  if (step % 20 == 0)
  {
    if (position_current == position_previous)
    {
      // finishes calibrating
      step = 0;
      zero_position_ = position_current - calibration_position_;
      position_sens_ = calibration_position_;
      position_sens_previous_ = calibration_position_;
      position_target_ = calibration_position_;
      position_reference_ = calibration_position_;
      position_reference_previous_ = calibration_position_;
      RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "Calibration Finished");
      calibration_flag_ = false;
      return true;
    }
    else
    {
      position_previous = position_current;
      return false;
    }
  }
  else
  {
    // continues calibration
    return false;
  }
}

bool VescServoController::isSaturated(const double arg) const
{
  if (std::abs(arg) > 1.0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

double VescServoController::saturate(const double arg) const
{
  if (arg > 1.0)
  {
    return 1.0;
  }
  else if (arg < -1.0)
  {
    return -1.0;
  }
  else
  {
    return arg;
  }
}

void VescServoController::updateSpeedLimitedPositionReference(void)
{
  if (position_target_ > (position_reference_previous_ + speed_limit_ * control_period_))
  {
    position_reference_ = position_reference_previous_ + speed_limit_ * control_period_;
  }
  else if (position_target_ < (position_reference_previous_ - speed_limit_ * control_period_))
  {
    position_reference_ = position_reference_previous_ - speed_limit_ * control_period_;
  }
  else
  {
    position_reference_ = position_target_;
  }
}

// void VescServoController::controlTimerCallback(const ros::TimerEvent& e)
// {
//   updateSpeedLimitedPositionReference();
//   control(position_reference_, position_sens_);
//   interface_ptr_->requestState();
// }

void VescServoController::updateSensor(const std::shared_ptr<VescPacket const>& packet)
{
  if (packet->getName() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);
    const double current = values->getMotorCurrent();
    const double velocity_rpm = values->getVelocityERPM() / static_cast<double>(num_motor_pole_pairs_);
    const double position_pulse = values->getPosition();
    // 3.0 represents the number of hall sensors
    position_sens_ = position_pulse / num_motor_pole_pairs_ / 3.0 * gear_ratio_ - getZeroPosition();  // unit: rad or m
    velocity_sens_ = velocity_rpm / 60.0 * 2.0 * M_PI * gear_ratio_;  // unit: rad/s or m/s
    effort_sens_ = current * torque_const_ / gear_ratio_;             // unit: Nm or N
  }
  return;
}
}  // namespace vesc_hw_interface
