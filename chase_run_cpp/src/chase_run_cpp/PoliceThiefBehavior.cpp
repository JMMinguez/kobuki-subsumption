// Copyright 2021 Intelligent Robotics Lab
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

#include <utility>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp/rclcpp.hpp"
#include "chase_run_cpp/PoliceThiefBehavior.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"

#include "kobuki_ros_interfaces/msg/led.hpp"

#include "std_msgs/msg/float32.hpp"

namespace chase_run
{

using namespace std::chrono_literals;
using std::placeholders::_1;

PoliceThiefBehavior::PoliceThiefBehavior()
: CascadeLifecycleNode("police_thief_behavior"),
  state_(POLICE),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  led_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("output_led", 0);
  distance_sub_ = create_subscription<std_msgs::msg::Float32>(
    "distance2person", 10,
    std::bind(&PoliceThiefBehavior::distance_callback, this, _1));
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoliceThiefBehavior::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = create_wall_timer(50ms, std::bind(&PoliceThiefBehavior::control_cycle, this));

  state_ts_ = now();

  go_state(POLICE);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoliceThiefBehavior::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = nullptr;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
PoliceThiefBehavior::control_cycle()
{
  switch (state_) {
    case POLICE:
      RCLCPP_INFO(get_logger(), "Role: Police \t START");

      out_led.value = BLUE;
      led_pub_->publish(out_led);

      add_activation("police_behavior");

      if (check_distance()) {
        out_led.value = OFF;
        led_pub_->publish(out_led);

        prev_fsm_state_ = 0;
        go_state(TURN);
      }
      break;

    case THIEF:
      RCLCPP_INFO(get_logger(), "Role: Thief \t START");

      out_led.value = RED;
      led_pub_->publish(out_led);

      add_activation("thief_behavior");

      if (check_thief_time()) {
        out_led.value = OFF;
        led_pub_->publish(out_led);

        prev_fsm_state_ = 1;
        go_state(TURN);
      }
      break;

    case TURN:
      vel_pub_->on_activate();
      out_vel_.angular.z = 0.3;

      out_led.value = GREEN;
      led_pub_->publish(out_led);

      if (check_turn()) {
        out_led.value = OFF;
        led_pub_->publish(out_led);

        first_turn_check_ = true;

        if (prev_fsm_state_ == 0) {
          remove_activation("police_behavior");
          go_state(THIEF);
        } else {
          remove_activation("thief_behavior");
          go_state(POLICE);
        }
      }

      vel_pub_->publish(out_vel_);

      break;
  }
}

void
PoliceThiefBehavior::go_state(int new_state)
{
  //clear_activation();

  state_ = new_state;
  state_ts_ = now();
}

bool
PoliceThiefBehavior::check_distance()
{
  std::cerr << "DISTANCIA 2: \t" << distance2person_ << std::endl;
  if (distance2person_ > 0.1) {
    return distance2person_ < 1;
  }
  return false;
}

bool
PoliceThiefBehavior::check_thief_time()
{
  return (now() - state_ts_) > THIEF_TIME;
}

bool
PoliceThiefBehavior::check_turn()
{
  std::string error;
  //  Gets the tf from 'odom' to 'base_footprint' at the start position
  if (first_turn_check_) {
    if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
      auto odom2bf_msg = tf_buffer_.lookupTransform(
        "odom", "base_footprint", tf2::TimePointZero);
      tf2::fromMsg(odom2bf_msg, odom2bf_);
      first_turn_check_ = false;
    }
  }

  tf2::Transform odom2bf_inverse = odom2bf_.inverse();
  tf2::Stamped<tf2::Transform> odom2bf1;

  //  Gets the tf between 'odom' and actual 'base_footprint'
  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    auto odom2bf1_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);

    tf2::fromMsg(odom2bf1_msg, odom2bf1);

    // Gets the tf from start 'base_footprint' and actual 'base_footprint'
    tf2::Transform bf2bf1 = odom2bf_inverse * odom2bf1;

    //  Calculate the angle between (0,0) and (x,y)
    tf2::Matrix3x3 mat(bf2bf1.getRotation());
    mat.getRPY(roll_, pitch_, yaw_);

    return yaw_ < TURN_LIMIT;
  }
}

void
PoliceThiefBehavior::distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  distance2person_ = msg->data;
  std::cerr << "DISTANCIA 1: \t" << distance2person_ << std::endl;
}

}  // namespace chase_run
