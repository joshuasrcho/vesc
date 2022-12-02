/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/** NOTE *************************************************************
 * This program had been developed by Michael T. Boulet at MIT under
 * the BSD 3-clause License until Dec. 2016. Since Nov. 2019, Softbank
 * Corp. takes over development as new packages.
 ********************************************************************/

#ifndef VESC_DRIVER_VESC_DRIVER_HPP_
#define VESC_DRIVER_VESC_DRIVER_HPP_

#include <cassert>
#include <cmath>
#include <experimental/optional>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "vesc_driver/vesc_interface.hpp"
#include "vesc_driver/vesc_packet.hpp"

namespace vesc_driver
{
class VescDriver : public rclcpp::Node
{
public:
  VescDriver(const rclcpp::NodeOptions& options);

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const std::shared_ptr<VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);

  // limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(rclcpp::Node* node_ptr, const std::string& str, const std::optional<double>& min_lower = std::nullopt,
                 const std::optional<double>& max_upper = std::nullopt);
    double clip(double value);
    rclcpp::Logger logger;
    std::string name;
    std::optional<double> lower;
    std::optional<double> upper;
  };
  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services
  rclcpp::Publisher<vesc_msgs::msg::VescStateStamped>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_sensor_pub_;
  rclcpp::SubscriptionBase::SharedPtr duty_cycle_sub_;
  rclcpp::SubscriptionBase::SharedPtr current_sub_;
  rclcpp::SubscriptionBase::SharedPtr brake_sub_;
  rclcpp::SubscriptionBase::SharedPtr speed_sub_;
  rclcpp::SubscriptionBase::SharedPtr position_sub_;
  rclcpp::SubscriptionBase::SharedPtr servo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // driver modes (possible states)
  typedef enum
  {
    MODE_INITIALIZING,
    MODE_OPERATING
  } driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;  ///< driver state machine mode (state)
  int fw_version_major_;       ///< firmware major version reported by vesc
  int fw_version_minor_;       ///< firmware minor version reported by vesc
  int num_motor_pole_pairs_;   // the number of motor pole pairs

  // ROS callbacks
  void timerCallback();
  void dutyCycleCallback(const std_msgs::msg::Float64::SharedPtr duty_cycle);
  void currentCallback(const std_msgs::msg::Float64::SharedPtr current);
  void brakeCallback(const std_msgs::msg::Float64::SharedPtr brake);
  void speedCallback(const std_msgs::msg::Float64::SharedPtr speed);
  void positionCallback(const std_msgs::msg::Float64::SharedPtr position);
  void servoCallback(const std_msgs::msg::Float64::SharedPtr servo);
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER_VESC_DRIVER_HPP_
