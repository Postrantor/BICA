// Copyright 2019 Intelligent Robotics Lab
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

#include "bica_behavior_tree/control/activation_sequence.hpp"

#include <sstream>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp_v3/control_node.h"
#include "bica_behavior_tree/action/activation_action.hpp"

namespace bica_behavior_tree {

// clang-format off
/*
该代码段是一个行为树（behavior tree）中的控制节点（ControlNode），名为ActivationSequenceNode。该节点维护了一个子节点列表，按顺序执行这些子节点，直到所有子节点都返回成功（SUCCESS）或者其中一个子节点返回失败（FAILURE）。当该节点被激活时，它会激活第一个子节点，并将自己的状态设置为RUNNING。当一个子节点返回成功时，该节点会激活下一个子节点，并将当前子节点的状态设置为SUCCESS。如果一个子节点返回失败，则该节点会停止所有子节点并将自己的状态设置为FAILURE。如果一个子节点返回IDLE，则会抛出异常。如果所有子节点都返回了SUCCESS，则该节点会停止所有子节点并将自己的状态设置为SUCCESS。

在代码中，ActivationSequenceNode继承自ControlNode，重写了reset、tick和halt函数。其中，reset函数用于重置current_child_idx_为0；tick函数是该节点的核心函数，用于执行子节点的行为逻辑；halt函数用于停止该节点及其所有子节点的行为逻辑。
*/
// clang-format on

/**
 * @brief ActivationSequenceNode类的构造函数，初始化current_child_idx_为0
 * @param name 组件名称
 * @details 调用父类ControlNode的构造函数，并设置RegistrationID为"ActivationSequence"
 */
ActivationSequenceNode::ActivationSequenceNode(const std::string &name)
    : BT::ControlNode(name, {}), current_child_idx_(0) {
  setRegistrationID("ActivationSequence");
}

/**
 * @brief 重置current_child_idx_为0
 */
void ActivationSequenceNode::reset() { current_child_idx_ = 0; }

/**
 * @brief 激活所有子节点
 * @return 返回RUNNING状态
 * @details
 * 当该节点处于IDLE状态时，激活当前子节点（如果是ActivationActionNode类型），并将该节点状态设置为RUNNING
 */
/**
 * @brief ActivationSequenceNode的tick函数，用于执行ActivationSequenceNode节点的逻辑
 * @details
 * 该函数会遍历所有子节点，并执行它们的executeTick()函数。如果某个子节点返回RUNNING，则该函数也返回RUNNING。
 *          如果某个子节点返回FAILURE，则该函数会将所有子节点重置，并返回FAILURE。
 *          如果所有子节点都返回SUCCESS，则该函数会返回SUCCESS。
 * @return BT::NodeStatus 返回当前节点的状态，可能为IDLE、RUNNING、SUCCESS或FAILURE
 */
BT::NodeStatus ActivationSequenceNode::tick() {
  const size_t children_count = children_nodes_.size();

  // 如果当前节点处于IDLE状态
  if (status() == BT::NodeStatus::IDLE) {
    BT::TreeNode *current_child_node = children_nodes_[current_child_idx_];

    // 如果当前子节点是ActivationActionNode类型的节点
    if (auto control_node =
            dynamic_cast<bica_behavior_tree::ActivationActionNode *>(current_child_node)) {
      control_node->onStart();
    }
  }

  setStatus(BT::NodeStatus::RUNNING);

  // 遍历所有子节点
  while (current_child_idx_ < children_count) {
    BT::TreeNode *current_child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = current_child_node->executeTick();

    switch (child_status) {
      // 如果当前子节点返回RUNNING，则该函数也返回RUNNING
      case BT::NodeStatus::RUNNING: {
        return child_status;
      }
      // 如果当前子节点返回FAILURE，则该函数会将所有子节点重置，并返回FAILURE
      case BT::NodeStatus::FAILURE: {
        haltChildren(0);
        current_child_idx_ = 0;
        return child_status;
      }
      // 如果当前子节点返回SUCCESS，则执行下一个子节点
      case BT::NodeStatus::SUCCESS: {
        // 如果当前子节点是ActivationActionNode类型的节点，则调用onStop()函数
        if (auto control_node =
                dynamic_cast<bica_behavior_tree::ActivationActionNode *>(current_child_node)) {
          control_node->onStop();
        }

        current_child_idx_++;

        // 执行下一个子节点
        current_child_node = children_nodes_[current_child_idx_];
        if (current_child_idx_ < children_count) {
          // 如果下一个子节点是ActivationActionNode类型的节点，则调用onStart()函数
          if (auto control_node =
                  dynamic_cast<bica_behavior_tree::ActivationActionNode *>(current_child_node)) {
            control_node->onStart();
          }
        }
      } break;

      // 如果当前子节点返回IDLE，则抛出异常
      case BT::NodeStatus::IDLE: {
        throw BT::LogicError("A child node must never return IDLE");
      }
    }  // end switch
  }    // end while loop

  // 如果所有子节点都返回SUCCESS，则该函数会返回SUCCESS
  if (current_child_idx_ == children_count) {
    haltChildren(0);
    current_child_idx_ = 0;
  }
  return BT::NodeStatus::SUCCESS;
}

/**
 * @brief 重置current_child_idx_为0，并调用父类ControlNode的halt函数
 */
void ActivationSequenceNode::halt() {
  current_child_idx_ = 0;
  BT::ControlNode::halt();
}

}  // namespace bica_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<bica_behavior_tree::ActivationSequenceNode>("ActivationSequence");
}
