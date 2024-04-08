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

#include "rclcpp/rclcpp.hpp"

#include "chase_run_cpp/PursuitBehavior.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "chase_run_cpp/PIDController.hpp"

#include "std_msgs/msg/float32.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace chase_run
{

using namespace std::chrono_literals;
using std::placeholders::_1;

PursuitBehavior::PursuitBehavior()
: CascadeLifecycleNode("police_behavior"),
  lin_pid_(0.0, 5.0, 0.0, 0.5),
  ang_pid_(0.0, M_PI / 2, 0.0, 0.5),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  distance_pub_ = create_publisher<std_msgs::msg::Float32>("distance2person", 10);
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  transform_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "tf_static", rclcpp::SensorDataQoS().reliable(),
    std::bind(&PursuitBehavior::transform_callback, this, _1));
  detection_2d_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "detection_2d", rclcpp::SensorDataQoS().reliable(),
    std::bind(&PursuitBehavior::detection_2d_callback, this, _1));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PursuitBehavior::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = create_wall_timer(50ms, std::bind(&PursuitBehavior::control_cycle, this));

  vel_pub_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PursuitBehavior::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = nullptr;

  vel_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
PursuitBehavior::control_cycle()
{
  lin_pid_.set_pid(0.6, 0.05, 0.35);
  ang_pid_.set_pid(0.6, 0.08, 0.32);

  vel.angular.z = ang_pid_.get_output(angulo);

  vel.linear.x = lin_pid_.get_output(distancia);

  vel_pub_->publish(vel);
}

void
PursuitBehavior::detection_2d_callback(vision_msgs::msg::Detection2DArray)
{
  tf2::Transform camera2person;
  camera2person.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  camera2person.setRotation(tf2::Quaternion(0, 0, 0, 1));

  geometry_msgs::msg::TransformStamped odom2camera_msg;
  tf2::Stamped<tf2::Transform> odom2camera;

  for (const auto & detection : msg->detections) {
    float x = detection.bbox.center.position.x;
    float y = detection.bbox.center.position.y;
  }
  
  odom2camera_msg = tf_buffer_.lookupTransform(
    "odom",   "camera_depth_optical_frame", //"camera_link",
    tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds())); //  -0.03
  tf2::fromMsg(odom2camera_msg, odom2camera);

  tf2::Transform odom2person = odom2camera * camera2person;

  geometry_msgs::msg::TransformStamped odom2person_msg;
  odom2person_msg.transform = tf2::toMsg(odom2person);

  odom2person_msg.header.stamp = msg->header.stamp;
  odom2person_msg.header.frame_id = "odom";
  odom2person_msg.child_frame_id = "person";

  tf_broadcaster_->sendTransform(odom2person_msg);
}

void
PursuitBehavior::transform_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg)
{
  for (int i = 0; i < std::size(msg->transforms); i++) {
    fprintf(
      stderr, "Received transform: %f, %f, %f, %f",
      msg->transforms[i].transform.translation.x, msg->transforms[i].transform.translation.y,
      msg->transforms[i].transform.rotation.x, msg->transforms[i].transform.rotation.y);

    geometry_msgs::msg::TransformStamped odom2person_msg;

    try {
      odom2person_msg = tf_buffer_.lookupTransform(
        "base_link", "person",
        tf2::timeFromSec(rclcpp::Time(odom2person_msg.header.stamp).seconds()));
      RCLCPP_INFO(get_logger(), "Cambio de Base_link");
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Person transform not found: %s", ex.what());
      return;
    }

    const auto & x = odom2person_msg.transform.translation.x;
    const auto & y = odom2person_msg.transform.translation.y;

    // Cálculo de distancia y ángulo
    distancia = sqrt(pow(x, 2) + pow(y, 2) );
    angulo = atan2(y, x);

    //publicar distancia
    auto out_distance = std::make_shared<std_msgs::msg::Float32>();
    out_distance->data = distancia;

    distance_pub_->publish(*out_distance);
  }
}

}  // namespace chase_run
