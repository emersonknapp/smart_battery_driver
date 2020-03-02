// Copyright 2020 Emerson Knapp
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "smart_battery_driver/sbs.hpp"
#include "smart_battery_driver/visibility_control.h"

using namespace std::chrono_literals;

namespace smart_battery_driver
{

typedef std::unique_ptr<sensor_msgs::msg::BatteryState> BatteryMsg;

class SmartBatteryDriver : public rclcpp::Node
{
public:
  explicit SmartBatteryDriver(const rclcpp::NodeOptions & options)
  : Node("smart_battery_driver", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;

    declare_parameter("device_path", rclcpp::ParameterValue(), descriptor);
    declare_parameter("battery_address", rclcpp::ParameterValue(), descriptor);
    declare_parameter("publish_frequency", 1.0);

    double publish_frequency = get_parameter("publish_frequency").as_double();
    std::chrono::duration<double> publish_period(1.0 / publish_frequency);

    std::string device_path = get_parameter("device_path").as_string();
    int battery_address = get_parameter("battery_address").as_int();

    battery_ = std::make_unique<SBS::SmartBattery>(device_path.c_str(), battery_address);

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", qos);
    timer_ = this->create_wall_timer(
      publish_period,
      std::bind(&SmartBatteryDriver::publish_battery_state, this));
  }

private:
  uint8_t interpret_status(int status, float current) const
  {
    using sensor_msgs::msg::BatteryState;
    using SBS::BatteryStatus;
    if (status & static_cast<int>(BatteryStatus::Discharging)) {
      return BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }
    // TODO(ek) figure out what this condition means
    // return POWER_SUPPLY_STATUS_NOT_CHARGING;
    if (status & static_cast<int>(BatteryStatus::FullyCharged)) {
      return BatteryState::POWER_SUPPLY_STATUS_FULL;
    }
    if (current > 0) {
      return BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    }

    return BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }

  uint8_t interpret_health(int status) const
  {
    using sensor_msgs::msg::BatteryState;
    using SBS::BatteryStatusAlarm;
    using SBS::BatteryStatus;

    if (status & static_cast<int>(BatteryStatusAlarm::OverTemp)) {
      return BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    }
    if (status & static_cast<int>(BatteryStatusAlarm::OverCharged)) {
      return BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    }
    if (status & static_cast<int>(BatteryStatus::FullyDischarged)) {
      return BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    }
    if (status & static_cast<int>(BatteryStatusAlarm::Mask)) {
      // Any alarm set and haven't returned yet
      return BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
    }

    return BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  }

  uint8_t interpret_chemistry(const std::string & chemistry)
  {
    using sensor_msgs::msg::BatteryState;
    std::string lower(chemistry);
    std::transform(
      lower.begin(), lower.end(), lower.begin(),
      [](unsigned char c) {return std::tolower(c);});
    if (lower == "nimh") {
      return BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
    } else if (lower == "lion") {
      return BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    } else if (lower == "lip") {
      return BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    } else if (lower == "nicd") {
      return BatteryState::POWER_SUPPLY_TECHNOLOGY_NICD;
    }
    return BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  }

  void publish_battery_state()
  {
    msg_ = std::make_unique<sensor_msgs::msg::BatteryState>();
    msg_->header.stamp = get_clock()->now();
    msg_->voltage = battery_->voltage() / 1000.0;
    msg_->temperature = battery_->temperature();
    msg_->current = battery_->current() / 1000.0;
    msg_->charge = -msg_->current;
    msg_->capacity = battery_->fullChargeCapacity() / 1000.0;
    msg_->design_capacity = battery_->designCapacity() / 1000.0;
    msg_->percentage = battery_->relativeStateOfCharge() / 100.0;

    int status = battery_->batteryStatus();
    msg_->power_supply_status = interpret_status(status, msg_->current);
    msg_->power_supply_health = interpret_health(status);

    std::string chemistry;
    battery_->cellChemistry(chemistry);
    msg_->power_supply_technology = interpret_chemistry(chemistry);
    // TODO(ek) detect case when it's not
    msg_->present = true;
    // msg->cell_voltage =
    // msg->location =
    msg_->serial_number = std::to_string(battery_->serialNumber());
    pub_->publish(std::move(msg_));
  }

  BatteryMsg msg_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<SBS::SmartBattery> battery_;
};

}  // namespace smart_battery_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<smart_battery_driver::SmartBatteryDriver>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// RCLCPP_COMPONENTS_REGISTER_NODE(smart_battery_driver::SmartBatteryDriver)
