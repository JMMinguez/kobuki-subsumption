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

#ifndef CHASE_RUN_CPP__PURSUITBEHAVIOR_HPP_
#define CHASE_RUN_CPP__PURSUITBEHAVIOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_msgs/msg/float32.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "chase_run_cpp/PIDController.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace chase_run
{

using namespace std::chrono_literals;

class PursuitBehavior : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  PursuitBehavior();

private:
  void control_cycle();
  void transform_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg);
  void detection_2d_callback(vision_msgs::msg::Detection2DArray);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  geometry_msgs::msg::Twist vel;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transform_sub_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  PIDController lin_pid_, ang_pid_;

  double angulo;
  float distancia;

  vision_msgs::msg::Detection2DArray::UniquePtr msg;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_2d_sub_;

};

}  // namespace chase_run

#endif  // CHASE_RUN_CPP__PURSUITBEHAVIOR_HPP_
