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

#ifndef CHASE_RUN_CPP__POLICETHIEFBEHAVIOR_HPP_
#define CHASE_RUN_CPP__POLICETHIEFBEHAVIOR_HPP_

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"

namespace chase_run
{

using namespace std::chrono_literals;

class PoliceThiefBehavior : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  PoliceThiefBehavior();

private:
  void control_cycle();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);
  
  int state_;
  rclcpp::Time state_ts_;
  
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped transform_;
  tf2::Stamped<tf2::Transform> odom2bf_;

  static const int POLICE = 0;
  static const int THIEF = 1;
  static const int TURN = 2;

  int prev_fsm_state_;
  bool first_turn_check_ {true};

  static const int TURN_LIMIT = M_PI;
  double roll_, pitch_, yaw_;


  void go_state(int new_state);
  bool check_distance();
  bool check_thief_time();
  bool check_turn();
  
  rclcpp::TimerBase::SharedPtr timer_;
  const rclcpp::Duration THIEF_TIME {10s};

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  geometry_msgs::msg::Twist out_vel_;
};

}  // namespace chase_run

#endif  //  CHASE_RUN_CPP__POLICETHIEFBEHAVIOR_HPP_
