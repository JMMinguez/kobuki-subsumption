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