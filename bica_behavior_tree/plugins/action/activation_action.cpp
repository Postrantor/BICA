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

#include "bica_behavior_tree/action/activation_action.hpp"

#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp_v3/control_node.h"

namespace bica_behavior_tree {

/*
  该代码段是一个激活行为树节点类，用于在BICA（基于行为的迭代式组件架构）组件中实现行为树相关功能。该类包含四个函数：

  - 构造函数：初始化激活行为树节点的名称和前一次的状态。
  -
  tick函数：执行激活行为树节点的操作。如果前一次的状态是IDLE，则设置当前状态为RUNNING，并将前一次的状态也设置为RUNNING。然后返回前一次的状态。
  - init函数：初始化激活行为树节点。将组件指针和激活列表保存到成员变量中。
  - onStart函数：开始执行激活行为树节点。遍历激活列表中的每一个元素，将其添加为组件的依赖项。
  - onStop函数：停止执行激活行为树节点。遍历激活列表中的每一个元素，将其从组件的依赖项中移除。
*/

/**
 * @brief 激活行为树节点类的构造函数
 * @param name 节点名称
 * @details 初始化激活行为树节点的名称和前一次的状态
 */
ActivationActionNode::ActivationActionNode(const std::string& name)
    : BT::ActionNodeBase(name, {}), previous_status_(BT::NodeStatus::IDLE) {
  setRegistrationID("ActivationAction");
}

/**
 * @brief 执行激活行为树节点的操作
 * @return 返回执行结果的状态
 * @details 如果前一次的状态是IDLE，则设置当前状态为RUNNING，并将前一次的状态也设置为RUNNING。
 *          然后返回前一次的状态。
 */
BT::NodeStatus ActivationActionNode::tick() {
  BT::NodeStatus previous_status_ = status();

  if (previous_status_ == BT::NodeStatus::IDLE) {
    setStatus(BT::NodeStatus::RUNNING);
    previous_status_ = BT::NodeStatus::RUNNING;
  }

  return previous_status_;
}

/**
 * @brief 初始化激活行为树节点
 * @param component 组件指针
 * @param activations 激活列表
 * @details 将组件指针和激活列表保存到成员变量中
 */
void ActivationActionNode::init(bica::Component* component, std::set<std::string> activations) {
  component_ = component;
  activations_ = activations;
}

/**
 * @brief 开始执行激活行为树节点
 * @return 返回执行结果的状态
 * @details 遍历激活列表中的每一个元素，将其添加为组件的依赖项
 */
BT::NodeStatus ActivationActionNode::onStart() {
  for (const auto act : activations_) {
    component_->addDependency(act);
  }
}

/**
 * @brief 停止执行激活行为树节点
 * @return 返回执行结果的状态
 * @details 遍历激活列表中的每一个元素，将其从组件的依赖项中移除
 */
BT::NodeStatus ActivationActionNode::onStop() {
  for (const auto act : activations_) {
    component_->removeDependency(act);
  }
}

}  // namespace bica_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<bica_behavior_tree::ActivationActionNode>("ActivationAction");
}
