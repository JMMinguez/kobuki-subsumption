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

#include "rclcpp/rclcpp.hpp"

#include "chase_run_cpp/PoliceBehavior.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace chase_run
{

using namespace std::chrono_literals;
using std::placeholders::_1;

PoliceBehavior::PoliceBehavior()
: CascadeLifecycleNode("police_behavior"),
  state_(SEARCH)
{
  detection_sub_ = create_subscription<yolov8_msgs::msg::DetectionArray>(
    "yolo/detections", rclcpp::SensorDataQoS().reliable(),
    std::bind(&PoliceBehavior::detection_callback, this, _1));
  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "detection_2d", rclcpp::SensorDataQoS().reliable());
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoliceBehavior::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = create_wall_timer(50ms, std::bind(&PoliceBehavior::control_cycle, this));

  state_ts_ = now();

  go_state(SEARCH);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PoliceBehavior::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = nullptr;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
PoliceBehavior::control_cycle()
{
  switch (state_) {
    case SEARCH:
      RCLCPP_INFO(get_logger(), "Role: Police \t SEARCH");
      add_activation("search_behavior");

      if (check_person()) {
        go_state(PURSUIT);
      }
      break;

    case PURSUIT:
      RCLCPP_INFO(get_logger(), "Role: Police \t PURSUIT");
      add_activation("pursuit_behavior");

      if (!check_person()) {
        go_state(SEARCH);
      }
      break;
  }
}

void
PoliceBehavior::go_state(int new_state)
{
  //clear_activation();
  remove_activation("search_behavior");
  remove_activation("pursuit_behavior");
  state_ = new_state;
  state_ts_ = now();
}

bool
PoliceBehavior::check_person()
{
  return person_detection_;
}

void
PoliceBehavior::detection_callback(const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg)
{
  vision_msgs::msg::Detection2DArray detection_array_msg;
  detection_array_msg.header = msg->header;

  person_detection_ = false;

  for (const auto & detection : msg->detections) {
    vision_msgs::msg::Detection2D detection_msg;
    detection_msg.header = msg->header;

    detection_msg.bbox.center.position.x = detection.bbox.center.position.x;
    detection_msg.bbox.center.position.y = detection.bbox.center.position.y;
    detection_msg.bbox.size_x = detection.bbox.size.x;
    detection_msg.bbox.size_y = detection.bbox.size.y;

    vision_msgs::msg::ObjectHypothesisWithPose obj_msg;
    obj_msg.hypothesis.class_id = detection.class_name;
    obj_msg.hypothesis.score = detection.score;

    detection_msg.results.push_back(obj_msg);
    detection_array_msg.detections.push_back(detection_msg);

    // Si el nombre de la clase es "persona", entonces se ha detectado una persona
    if (obj_msg.hypothesis.class_id == "person") {
      person_detection_ = true;
      //detection_pub_->publish(detection_array_msg);
    }
  }
}
}  // namespace chase_run
