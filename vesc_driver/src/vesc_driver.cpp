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

#include "vesc_driver/vesc_driver.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace vesc_driver
{
VescDriver::VescDriver(const rclcpp::NodeOptions& options)
  : rclcpp::Node("vesc_driver", options)
  , vesc_(std::string(), std::bind(&VescDriver::vescPacketCallback, this, std::placeholders::_1),
          std::bind(&VescDriver::vescErrorCallback, this, std::placeholders::_1))
  , duty_cycle_limit_(this, "duty_cycle", -1.0, 1.0)
  , current_limit_(this, "current")
  , brake_limit_(this, "brake")
  , speed_limit_(this, "speed")
  , position_limit_(this, "position")
  , servo_limit_(this, "servo", 0.0, 1.0)
  , driver_mode_(MODE_INITIALIZING)
  , fw_version_major_(-1)
  , fw_version_minor_(-1)
{
  // get vesc serial port address
  std::string port = declare_parameter<std::string>("port", "");
  if (port.empty())
  {
    RCLCPP_FATAL(get_logger(), "VESC communication port parameter required.");
    rclcpp::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try
  {
    vesc_.connect(port);
  }
  catch (const SerialException& e)
  {
    RCLCPP_FATAL(get_logger(), "Failed to connect to the VESC, %s.", e.what());
    rclcpp::shutdown();
    return;
  }

  // get the number of motor pole pairs
  num_motor_pole_pairs_ = declare_parameter<int>("num_motor_pole_pairs", 1);
  RCLCPP_INFO(get_logger(), "The number of motor pole pairs is set to %d", num_motor_pole_pairs_);

  // create vesc state (telemetry) publisher
  state_pub_ = create_publisher<vesc_msgs::msg::VescStateStamped>("sensors/core", rclcpp::QoS{ 10 });

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ = create_publisher<std_msgs::msg::Float64>("sensors/servo_position_command", rclcpp::QoS{ 10 });

  // subscribe to motor and servo command topics

  duty_cycle_sub_ = create_subscription<std_msgs::msg::Float64>(
      "commands/motor/duty_cycle", rclcpp::QoS{ 10 },
      std::bind(&VescDriver::dutyCycleCallback, this, std::placeholders::_1));
  current_sub_ =
      create_subscription<std_msgs::msg::Float64>("commands/motor/current", rclcpp::QoS{ 10 },
                                                  std::bind(&VescDriver::currentCallback, this, std::placeholders::_1));
  brake_sub_ = create_subscription<std_msgs::msg::Float64>(
      "commands/motor/brake", rclcpp::QoS{ 10 }, std::bind(&VescDriver::brakeCallback, this, std::placeholders::_1));
  speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "commands/motor/speed", rclcpp::QoS{ 10 }, std::bind(&VescDriver::speedCallback, this, std::placeholders::_1));
  position_sub_ = create_subscription<std_msgs::msg::Float64>(
      "commands/motor/position", rclcpp::QoS{ 10 },
      std::bind(&VescDriver::positionCallback, this, std::placeholders::_1));
  servo_sub_ = create_subscription<std_msgs::msg::Float64>(
      "commands/servo/position", rclcpp::QoS{ 10 }, std::bind(&VescDriver::servoCallback, this, std::placeholders::_1));

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(20ms, std::bind(&VescDriver::timerCallback, this));
}

/* TODO or TO-THINKABOUT LIST
  - what should we do on startup? send brake or zero command?
  - what to do if the vesc interface gives an error?
  - check version number against know compatable?
  - should we wait until we receive telemetry before sending commands?
  - should we track the last motor command
  - what to do if no motor command received recently?
  - what to do if no servo command received recently?
  - what is the motor safe off state (0 current?)
  - what to do if a command parameter is out of range, ignore?
  - try to predict vesc bounds (from vesc config) and command detect bounds errors
*/

void VescDriver::timerCallback()
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected())
  {
    RCLCPP_FATAL(get_logger(), "Unexpectedly disconnected from serial port.");
    timer_.reset();
    rclcpp::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING)
  {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0)
    {
      RCLCPP_INFO(get_logger(), "Connected to VESC with firmware version %d.%d", fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING)
  {
    // poll for vesc state (telemetry)
    vesc_.requestState();
  }
  else
  {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const std::shared_ptr<VescPacket const>& packet)
{
  if (packet->getName() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);

    auto state_msg = vesc_msgs::msg::VescStateStamped();
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    state_msg.header.stamp = ros_clock.now();
    state_msg.state.voltage_input = values->getInputVoltage();
    state_msg.state.temperature_pcb = values->getMosTemp();
    state_msg.state.current_motor = values->getMotorCurrent();
    state_msg.state.current_input = values->getInputCurrent();
    state_msg.state.speed = values->getVelocityERPM() / static_cast<double>(num_motor_pole_pairs_) / 60.0 * 2.0 * M_PI;
    state_msg.state.duty_cycle = values->getDuty();
    state_msg.state.charge_drawn = values->getConsumedCharge();
    state_msg.state.charge_regen = values->getInputCharge();
    state_msg.state.energy_drawn = values->getConsumedPower();
    state_msg.state.energy_regen = values->getInputPower();
    state_msg.state.displacement = values->getPosition();
    state_msg.state.distance_traveled = values->getDisplacement();
    state_msg.state.fault_code = values->getFaultCode();

    state_pub_->publish(state_msg);
  }
  else if (packet->getName() == "FWVersion")
  {
    std::shared_ptr<VescPacketFWVersion const> fw_version =
        std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void VescDriver::vescErrorCallback(const std::string& error)
{
  RCLCPP_ERROR(get_logger(), "%s", error.c_str());
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void VescDriver::dutyCycleCallback(const std_msgs::msg::Float64::SharedPtr duty_cycle)
{
  if (driver_mode_ == MODE_OPERATING)
  {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
  }
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void VescDriver::currentCallback(const std_msgs::msg::Float64::SharedPtr current)
{
  if (driver_mode_ == MODE_OPERATING)
  {
    vesc_.setCurrent(current_limit_.clip(current->data));
  }
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void VescDriver::brakeCallback(const std_msgs::msg::Float64::SharedPtr brake)
{
  if (driver_mode_ == MODE_OPERATING)
  {
    vesc_.setBrake(brake_limit_.clip(brake->data));
  }
}

/**
 * @param speed Commanded VESC speed in rad/s. Although any value is accepted by this driver, the VESC may impose a
 *              more restrictive bounds on the range depending on its configuration.
 */
void VescDriver::speedCallback(const std_msgs::msg::Float64::SharedPtr speed)
{
  if (driver_mode_ == MODE_OPERATING)
  {
    const double speed_erpm = speed->data / 2.0 / M_PI * 60.0 * static_cast<double>(num_motor_pole_pairs_);
    vesc_.setSpeed(speed_limit_.clip(speed_erpm));
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriver::positionCallback(const std_msgs::msg::Float64::SharedPtr position)
{
  if (driver_mode_ == MODE_OPERATING)
  {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
void VescDriver::servoCallback(const std_msgs::msg::Float64::SharedPtr servo)
{
  if (driver_mode_ == MODE_OPERATING)
  {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    auto servo_sensor_msg = std_msgs::msg::Float64();
    servo_sensor_msg.data = servo_clipped;
    servo_sensor_pub_->publish(servo_sensor_msg);
  }
}

VescDriver::CommandLimit::CommandLimit(rclcpp::Node* node_ptr, const std::string& str,
                                       const std::optional<double>& min_lower, const std::optional<double>& max_upper)
  : logger(node_ptr->get_logger()), name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  auto param_min = node_ptr->declare_parameter(name + "_min", rclcpp::ParameterValue(0.0));
  if (param_min.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    if (min_lower && param_min.get<double>() < *min_lower)
    {
      lower = *min_lower;
      RCLCPP_WARN_STREAM(logger, "Parameter " << name << "_min (" << param_min.get<double>()
                                              << ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_min.get<double>() > *max_upper)
    {
      lower = *max_upper;
      RCLCPP_WARN_STREAM(logger, "Parameter " << name << "_min (" << param_min.get<double>()
                                              << ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else
    {
      lower = param_min.get<double>();
    }
  }
  else if (min_lower)
  {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper

  auto param_max = node_ptr->declare_parameter(name + "_max", rclcpp::ParameterValue(0.0));
  if (param_min.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    if (min_lower && param_max.get<double>() < *min_lower)
    {
      upper = *min_lower;
      RCLCPP_WARN_STREAM(logger, "Parameter " << name << "_max (" << param_max.get<double>()
                                              << ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_max.get<double>() > *max_upper)
    {
      upper = *max_upper;
      RCLCPP_WARN_STREAM(logger, "Parameter " << name << "_max (" << param_max.get<double>()
                                              << ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else
    {
      upper = param_max.get<double>();
    }
  }
  else if (max_upper)
  {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper)
  {
    RCLCPP_WARN_STREAM(logger, "Parameter " << name << "_max (" << *upper << ") is less than parameter " << name
                                            << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower)
    oss << *lower << " ";
  else
    oss << "(none) ";
  if (upper)
    oss << *upper;
  else
    oss << "(none)";
  RCLCPP_DEBUG_STREAM(logger, oss.str());
}

double VescDriver::CommandLimit::clip(double value)
{
  auto clock = rclcpp::Clock(RCL_ROS_TIME);
  if (lower && value < lower)
  {
    RCLCPP_INFO_THROTTLE(logger, clock, 10, "%s command value (%f) below minimum limit (%f), clipping.", name.c_str(),
                         value, *lower);
    return *lower;
  }
  if (upper && value > upper)
  {
    RCLCPP_INFO_THROTTLE(logger, clock, 10, "%s command value (%f) above maximum limit (%f), clipping.", name.c_str(),
                         value, *upper);
    return *upper;
  }
  return value;
}

}  // namespace vesc_driver

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_driver::VescDriver)
