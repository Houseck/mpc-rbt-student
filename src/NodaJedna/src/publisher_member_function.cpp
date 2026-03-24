// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    this->declare_parameter<double>("min_voltage", 32.0);
    this->declare_parameter<double>("max_voltage", 42.0);

    publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));

    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_voltage", 10, std::bind(&MinimalPublisher::topic_callback, this, std::placeholders::_1));
    publisher_b = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "minimal_publisher"/* + std::to_string(count_++)*/;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    double min_v = this->get_parameter("min_voltage").as_double();
    double max_v = this->get_parameter("max_voltage").as_double();

    float voltage = msg->data;
    float percentage = ((voltage - min_v)/(max_v-min_v))*100;
    auto out_msg = std_msgs::msg::Float32();
    out_msg.data = percentage;
    publisher_b->publish(out_msg);
    RCLCPP_INFO(this->get_logger(), "I heard: '%.2f' per: '%.2f'", voltage, percentage);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_b;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
