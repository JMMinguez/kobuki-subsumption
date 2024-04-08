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

#include <memory>

#include "chase_run_cpp/PoliceThiefBehavior.hpp"

#include "chase_run_cpp/ThiefBehavior.hpp"
#include "chase_run_cpp/ForwardBehavior.hpp"
#include "chase_run_cpp/AvoidBehavior.hpp"

#include "chase_run_cpp/PoliceBehavior.hpp"
#include "chase_run_cpp/SearchBehavior.hpp"
#include "chase_run_cpp/PursuitBehavior.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 3);

  auto policethief_node = std::make_shared<chase_run::PoliceThiefBehavior>();

  auto thief_node = std::make_shared<chase_run::ThiefBehavior>();
  auto forward_node = std::make_shared<chase_run::ForwardBehavior>();
  auto avoid_node = std::make_shared<chase_run::AvoidBehavior>();

  auto police_node = std::make_shared<chase_run::PoliceBehavior>();
  auto search_node = std::make_shared<chase_run::SearchBehavior>();
  //auto pursuit_node = std::make_shared<chase_run::PursuitBehavior>();

  exe.add_node(policethief_node->get_node_base_interface());

  exe.add_node(thief_node->get_node_base_interface());
  exe.add_node(forward_node->get_node_base_interface());
  exe.add_node(avoid_node->get_node_base_interface());

  exe.add_node(police_node->get_node_base_interface());
  exe.add_node(search_node->get_node_base_interface());
  //exe.add_node(pursuit_node->get_node_base_interface());

  policethief_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  thief_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  forward_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  avoid_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  police_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  search_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  //pursuit_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  policethief_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  exe.spin();

  rclcpp::shutdown();

  return 0;
}