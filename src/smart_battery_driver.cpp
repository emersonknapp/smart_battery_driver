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
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "smart_battery_driver/visibility_control.h"

using namespace std::chrono_literals;

namespace smart_battery_driver {

class SmartBatteryDriver : public rclcpp::Node
{
public:
  SMART_BATTERY_DRIVER_PUBLIC
  explicit SmartBatteryDriver(const rclcpp::NodeOptions & options)
  : Node("smart_battery_driver", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", qos);
    timer_ = this->create_wall_timer(
      1s, std::bind(&SmartBatteryDriver::publish_battery_state, this));
  }

private:
  void publish_battery_state() {
    msg_ = std::make_unique<sensor_msgs::msg::BatteryState>();

    RCLCPP_INFO(get_logger(), "Publishing!");
    pub_->publish(std::move(msg_));
  }

  size_t count_ = 1;
  std::unique_ptr<sensor_msgs::msg::BatteryState> msg_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace smart_battery_driver

int main(int argc, char ** argv) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<smart_battery_driver::SmartBatteryDriver>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// RCLCPP_COMPONENTS_REGISTER_NODE(smart_battery_driver::SmartBatteryDriver)
