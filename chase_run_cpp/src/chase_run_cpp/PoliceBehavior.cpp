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
#include "chase_run_cpp/PoliceBehavior.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace chase_run
{

using namespace std::chrono_literals;
using std::placeholders::_1;

PoliceBehavior::PoliceBehavior()
: CascadeLifecycleNode("police_behavior"),
  state_(SEARCH)
{
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
      break;

    case PURSUIT:
      RCLCPP_INFO(get_logger(), "Role: Police \t PURSUIT");
      add_activation("pursuit_behavior");
      break;
  }
}

void
PoliceBehavior::go_state(int new_state)
{
  clear_activation();

  state_ = new_state;
  state_ts_ = now();
}

}  // namespace chase_run
