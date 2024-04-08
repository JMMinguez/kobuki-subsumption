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

#ifndef CHASE_RUN_CPP__POLICEBEHAVIOR_HPP_
#define CHASE_RUN_CPP__POLICEBEHAVIOR_HPP_

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"


namespace chase_run
{

using namespace std::chrono_literals;

class PoliceBehavior : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  PoliceBehavior();

private:
  void control_cycle();
  void detection_callback(const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  static const int SEARCH = 0;
  static const int PURSUIT = 1;

  int state_;
  rclcpp::Time state_ts_;

  void go_state(int new_state);
  bool check_person();

  bool person_detection_ {false};

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;

};

}  // namespace chase_run

#endif  // CHASE_RUN_CPP__THIEFBEHAVIOR_HPP_
