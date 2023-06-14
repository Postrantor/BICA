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

#include "bica/Component.hpp"

#include <memory>
#include <string>

#include "bica_msgs/srv/activate_component.hpp"
#include "bica_msgs/srv/deactivate_component.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

namespace bica {

using namespace std::chrono_literals;

/**
 * @brief Component类的构造函数，继承自rclcpp_lifecycle::LifecycleNode类
 * @param id 组件的ID
 * @param rate 组件的频率
 * @details 在构造函数中，创建了发布者、订阅者和服务，并触发了TRANSITION_CONFIGURE状态的转换。
 */
Component::Component(const std::string& id, float rate)
    : rclcpp_lifecycle::LifecycleNode(id), rate_(rate), rate_freq_(rate) {
  using namespace std::placeholders;

  // 创建一个名为/bica_activations的话题的发布者，消息类型为std_msgs::msg::String，QoS设置为reliable
  activation_pub_ =
      create_publisher<std_msgs::msg::String>("/bica_activations", rclcpp::QoS(1).reliable());
  // 创建一个名为/bica_activations的话题的订阅者，消息类型为std_msgs::msg::String，QoS设置为reliable，回调函数为activations_callback
  activation_sub_ = create_subscription<std_msgs::msg::String>(
      "/bica_activations", rclcpp::QoS(10).reliable(),
      std::bind(&Component::activations_callback, this, _1));

  // 创建一个名为~/activate的服务，服务类型为bica_msgs::srv::ActivateComponent，回调函数为activate_callback
  activation_service_ = create_service<bica_msgs::srv::ActivateComponent>(
      "~/activate", std::bind(&Component::activate_callback, this, _1, _2, _3));

  // 创建一个名为~/deactivate的服务，服务类型为bica_msgs::srv::DeactivateComponent，回调函数为deactivate_callback
  deactivation_service_ = create_service<bica_msgs::srv::DeactivateComponent>(
      "~/deactivate", std::bind(&Component::deactivate_callback, this, _1, _2, _3));

  // 触发状态转换，将组件的状态从unconfigured转换为inactive
  trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief BICA组件的on_configure回调函数
 * @param state 生命周期状态
 * @details 当组件被配置时，激活发布器并通知启动
 */
CallbackReturnT Component::on_configure(const rclcpp_lifecycle::State& state) {
  RCLCPP_DEBUG(get_logger(), "on_configure start");
  activation_pub_->on_activate();

  notifyStart();

  RCLCPP_DEBUG(get_logger(), "on_configure end");
  return CallbackReturnT::SUCCESS;
}

/**
 * @brief BICA组件的on_activate回调函数
 * @param state 生命周期状态
 * @details 激活依赖项并调用自身的on_activate函数
 */
CallbackReturnT Component::on_activate(const rclcpp_lifecycle::State& state) {
  RCLCPP_DEBUG(get_logger(), "on_activate start");
  activateDependencies();
  this->on_activate();
  RCLCPP_DEBUG(get_logger(), "on_activate end");

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief BICA组件的on_deactivate回调函数
 * @param state 生命周期状态
 * @details 停用依赖项并调用自身的on_deactivate函数
 */
CallbackReturnT Component::on_deactivate(const rclcpp_lifecycle::State& state) {
  RCLCPP_DEBUG(get_logger(), "on_deactivate start");
  deActivateDependencies();
  this->on_deactivate();
  RCLCPP_DEBUG(get_logger(), "on_deactivate end");

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief BICA组件的on_cleanup回调函数
 * @param state 生命周期状态
 * @details 停用依赖项
 */
CallbackReturnT Component::on_cleanup(const rclcpp_lifecycle::State& state) {
  RCLCPP_DEBUG(get_logger(), "on_cleanup start");
  deActivateDependencies();
  RCLCPP_DEBUG(get_logger(), "on_cleanup end");

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief BICA组件的on_shutdown回调函数
 * @param state 生命周期状态
 * @details 停用依赖项
 */
CallbackReturnT Component::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCLCPP_DEBUG(get_logger(), "on_shutdown start");
  deActivateDependencies();
  RCLCPP_DEBUG(get_logger(), "on_shutdown end");

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief BICA组件的on_error回调函数
 * @param state 生命周期状态
 * @details 停用依赖项
 */
CallbackReturnT Component::on_error(const rclcpp_lifecycle::State& state) {
  RCLCPP_DEBUG(get_logger(), "on_error start");
  deActivateDependencies();
  RCLCPP_DEBUG(get_logger(), "on_error end");

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief 添加依赖项
 * @param dep 依赖项名称
 * @details 如果当前状态为PRIMARY_STATE_ACTIVE，则激活该依赖项
 */
void Component::addDependency(const std::string& dep) {
  dependencies_.insert(dep);
  if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    activateDependency(dep);
  }
}

/**
 * @brief 移除依赖项
 * @param dep 依赖项名称
 * @details 如果当前状态为PRIMARY_STATE_ACTIVE，则停用该依赖项
 */
void Component::removeDependency(const std::string& dep) {
  dependencies_.erase(dep);
  if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    deactivateDependency(dep);
  }
}

/**
 * @brief 判断是否为依赖项
 * @param dep 依赖项名称
 * @return bool 是否为依赖项
 */
bool Component::isDependency(const std::string& dep) {
  return dependencies_.find(dep) != dependencies_.end();
}

/**
 * @brief 通知启动
 * @details 发布启动消息
 */
void Component::notifyStart() {
  RCLCPP_DEBUG(get_logger(), "notifyStart start");
  std_msgs::msg::String msg;
  msg.data = get_name();

  activation_pub_->publish(msg);
  RCLCPP_DEBUG(get_logger(), "notifyStart end");
}

/*
  该代码段是 BICA 组件中用于激活和停用组件的回调函数。其中，activate_callback
  函数用于激活组件，deactivate_callback 函数用于停用组件，removeActivator 函数用于移除激活器。

  在 activate_callback 函数中，当组件处于非激活状态时，将请求的激活器添加到激活器列表中，并触发
  TRANSITION_ACTIVATE 过渡状态。如果组件已经处于激活状态或正在激活过程中，则不做任何操作。

  在 removeActivator
  函数中，如果指定的激活器不存在于激活器列表中，则输出警告信息。如果激活器列表为空，则触发
  TRANSITION_DEACTIVATE 过渡状态。

  在 deactivate_callback 函数中，当组件处于激活状态时，将请求的停用器从激活器列表中移除，并触发
  TRANSITION_DEACTIVATE 过渡状态。如果组件不处于激活状态，则不做任何操作。
*/

/**
 * @brief 激活回调函数，用于激活组件
 * @param request_header 请求头指针
 * @param request 请求指针，包含请求的数据
 * @param response 响应指针，用于返回响应的数据
 * @details 当组件处于非激活状态时，将请求的激活器添加到激活器列表中，并触发 TRANSITION_ACTIVATE
 * 过渡状态。 如果组件已经处于激活状态或正在激活过程中，则不做任何操作。
 */
void Component::activate_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<bica_msgs::srv::ActivateComponent::Request> request,
    const std::shared_ptr<bica_msgs::srv::ActivateComponent::Response> response) {
  RCLCPP_DEBUG(
      get_logger(), "activate_callback start (in state %s)",
      this->get_current_state().label().c_str());

  activators_.insert(request->activator);
  if ((this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) &&
      (this->get_current_state().id() != lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING)) {
    RCLCPP_DEBUG(get_logger(), "notifyStart triggering");

    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }
  RCLCPP_DEBUG(get_logger(), "activate_callback end");
}

/**
 * @brief 移除激活器函数，用于移除激活器
 * @param activator 激活器名称
 * @details 如果指定的激活器不存在于激活器列表中，则输出警告信息。
 *          如果激活器列表为空，则触发 TRANSITION_DEACTIVATE 过渡状态。
 */
void Component::removeActivator(const std::string& activator) {
  if (activators_.erase(activator) == 0) {
    RCLCPP_WARN(
        get_logger(), "Request for deactivation from a non valid activator [%s]",
        activator.c_str());
  }

  if (activators_.empty()) {
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }
}

/**
 * @brief 停用回调函数，用于停用组件
 * @param request_header 请求头指针
 * @param request 请求指针，包含请求的数据
 * @param response 响应指针，用于返回响应的数据
 * @details 当组件处于激活状态时，将请求的停用器从激活器列表中移除，并触发 TRANSITION_DEACTIVATE
 * 过渡状态。 如果组件不处于激活状态，则不做任何操作。
 */
void Component::deactivate_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<bica_msgs::srv::DeactivateComponent::Request> request,
    const std::shared_ptr<bica_msgs::srv::DeactivateComponent::Response> response) {
  RCLCPP_DEBUG(
      get_logger(), "deactivate_callback start (in state %s)",
      this->get_current_state().label().c_str());

  if ((this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)) {
    RCLCPP_DEBUG(get_logger(), "deactivate_callback triggering");

    removeActivator(request->deactivator);
  }
  RCLCPP_DEBUG(get_logger(), "deactivate_callback end");
}

/**
 * @brief 激活依赖组件
 * @param dep 依赖组件名称
 * @details 创建一个客户端，向指定的服务请求激活依赖组件。如果服务不可用，则返回错误信息。
 */
void Component::activateDependency(const std::string& dep) {
  RCLCPP_DEBUG(get_logger(), "activateDependency start --> %s", dep.c_str());
  auto client = this->create_client<bica_msgs::srv::ActivateComponent>("/" + dep + "/activate");

  if (!client->wait_for_service(100ms)) {
    RCLCPP_ERROR(get_logger(), "Service %s is not available.", client->get_service_name());
    return;
  }

  auto request = std::make_shared<bica_msgs::srv::ActivateComponent::Request>();
  request->activator = get_name();

  auto future = client->async_send_request(request);
  pending_act_futures_.push_back(ActivationFuture{future, dep});

  RCLCPP_DEBUG(get_logger(), "activateDependency end");
}

/**
 * @brief 停用依赖组件
 * @param dep 依赖组件名称
 * @details 创建一个客户端，向指定的服务请求停用依赖组件。如果服务不可用，则返回错误信息。
 */
void Component::deactivateDependency(const std::string& dep) {
  RCLCPP_DEBUG(get_logger(), "deactivateDependency start --> %s", dep.c_str());

  auto client = this->create_client<bica_msgs::srv::DeactivateComponent>("/" + dep + "/deactivate");

  if (!client->wait_for_service(100ms)) {
    RCLCPP_ERROR(get_logger(), "Service %s is not available.", client->get_service_name());
    return;
  }

  auto request = std::make_shared<bica_msgs::srv::DeactivateComponent::Request>();
  request->deactivator = get_name();

  auto future = client->async_send_request(request);

  pending_deact_futures_.push_back(DeactivationFuture{future, dep});

  RCLCPP_DEBUG(get_logger(), "deactivateDependency end");
}

/*
  这段代码是 BICA（Behavior-based Iterative Component
  Architecture）组件中的相关代码。其中包含三个函数：

  activateDependencies()：激活组件的依赖项。遍历所有依赖项，逐一激活。
  deActivateDependencies()：停用组件的依赖项。遍历所有依赖项，逐一停用。
  activations_callback(const std_msgs::msg::String::SharedPtr
  msg)：组件激活回调函数。当组件处于PRIMARY_STATE_ACTIVE状态且收到的消息是其依赖项时，激活该依赖项。
  其中，activateDependency()和deactivateDependency()函数在此处未给出，但可以猜测它们分别用于激活和停用单个依赖项。代码中还使用了ROS2的消息机制，通过订阅std_msgs::msg::String类型的消息来实现组件激活回调函数。
*/

/**
 * @brief 激活组件的依赖项
 * @param 无
 * @details 遍历所有依赖项，逐一激活
 */
void Component::activateDependencies() {
  for (const auto& dep : dependencies_) {
    activateDependency(dep);
  }
}

/**
 * @brief 停用组件的依赖项
 * @param 无
 * @details 遍历所有依赖项，逐一停用
 */
void Component::deActivateDependencies() {
  for (const auto& dep : dependencies_) {
    deactivateDependency(dep);
  }
}

/**
 * @brief 组件激活回调函数
 * @param msg std_msgs::msg::String类型的ROS2消息指针
 * @details 当组件处于PRIMARY_STATE_ACTIVE状态且收到的消息是其依赖项时，激活该依赖项
 */
void Component::activations_callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_DEBUG(get_logger(), "activations_callback start <-- %s", msg->data.c_str());
  if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
      isDependency(msg->data)) {
    activateDependency(msg->data);
  }
  RCLCPP_DEBUG(get_logger(), "activations_callback end");
}

/*
  功能总结：用于判断ROS2节点是否已经启动，并等待激活和停用操作完成。具体实现过程如下：

  - 判断ROS2节点是否已经启动；
  - 如果ROS2节点已经启动，则获取ROS2节点名称列表和组件的激活器列表；
  - 遍历激活器列表，如果激活器不在节点列表中，则从激活器列表中删除该激活器；
  - 循环处理未完成的激活操作，等待激活操作完成；
  - 循环处理未完成的停用操作，等待停用操作完成；
  - 返回ROS2节点是否已经启动。
*/

/**
 * @brief 判断ROS2节点是否已经启动
 * @details
 * 如果ROS2节点已经启动，则检查组件的激活器是否存在于节点中，如果不存在则从激活器列表中删除。然后等待激活和停用操作完成。
 * @return 返回一个bool值，表示ROS2节点是否已经启动
 */
bool Component::ok() {
  if (rclcpp::ok()) {  // 判断ROS2节点是否已经启动
    auto nodes = this->get_node_graph_interface()->get_node_names();  // 获取ROS2节点名称列表
    auto check_activators = activators_;  // 获取组件的激活器列表
    // 遍历激活器列表
    for (auto const& activator : check_activators) {
      if (std::find(nodes.begin(), nodes.end(), "/" + activator) ==
          nodes.end()) {  // 如果激活器不在节点列表中
        RCLCPP_DEBUG(
            get_logger(),
            "Activator %s is not longer present, removing from activators");  // 输出调试信息
        removeActivator(activator);  // 从激活器列表中删除该激活器
      }
    }

    // 循环处理未完成的激活操作
    while (!pending_act_futures_.empty()) {
      auto pending_future = pending_act_futures_.back();  // 获取最后一个未完成的激活操作
      if ((rclcpp::spin_until_future_complete(
              this->get_node_base_interface(), pending_future.future)) !=
          rclcpp::executor::FutureReturnCode::SUCCESS) {  // 等待激活操作完成
        RCLCPP_WARN(
            get_logger(), "Component [%s] failed to activate",
            pending_future.component.c_str());  // 输出警告信息
      }
      pending_act_futures_.pop_back();  // 从未完成的激活操作列表中删除该操作
    }

    // 循环处理未完成的停用操作
    while (!pending_deact_futures_.empty()) {
      auto pending_future = pending_deact_futures_.back();  // 获取最后一个未完成的停用操作
      if ((rclcpp::spin_until_future_complete(
              this->get_node_base_interface(), pending_future.future)) !=
          rclcpp::executor::FutureReturnCode::SUCCESS) {  // 等待停用操作完成
        RCLCPP_WARN(
            get_logger(), "Component [%s] failed to deactivate",
            pending_future.component.c_str());  // 输出警告信息
      }
      pending_deact_futures_.pop_back();  // 从未完成的停用操作列表中删除该操作
    }
  }

  return rclcpp::ok();  // 返回ROS2节点是否已经启动
}

/**
 * @brief Component::execute()函数是BICA组件的执行函数，用于循环执行组件的step()函数。
 * @param 无参数
 * @details
 * 当组件处于PRIMARY_STATE_ACTIVE状态时，调用组件的step()函数。然后使用rclcpp::spin_some()函数等待回调函数被调用，以便处理接收到的消息。最后通过rate_控制循环的频率。
 */
void Component::execute() {
  while (this->ok()) {
    RCLCPP_DEBUG(get_logger(), "state: %s", this->get_current_state().label().c_str());
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      step();
    }

    // 等待回调函数被调用，以便处理接收到的消息
    rclcpp::spin_some(this->get_node_base_interface());
    rate_.sleep();
  }
}

/**
 * @brief Component::execute_once(bool
 * spin)函数是BICA组件的单次执行函数，用于执行一次组件的step()函数。
 * @param spin：bool类型，表示是否需要等待回调函数被调用，以便处理接收到的消息。
 * @details
 * 当组件处于PRIMARY_STATE_ACTIVE状态时，调用组件的step()函数。如果spin为true，则使用rclcpp::spin_some()函数等待回调函数被调用，以便处理接收到的消息。最后通过rate_控制循环的频率。
 */
void Component::execute_once(bool spin) {
  if (this->ok()) {
    RCLCPP_DEBUG(get_logger(), "state: %s", this->get_current_state().label().c_str());
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      step();
    }

    // 如果spin为true，则等待回调函数被调用，以便处理接收到的消息
    if (spin) {
      rclcpp::spin_some(this->get_node_base_interface());
      rate_.sleep();
    }
  }
}

}  // namespace bica
