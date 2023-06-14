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

#ifndef BICA__COMPONENT_HPP_
#define BICA__COMPONENT_HPP_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "bica/Utils.hpp"
#include "bica_msgs/srv/activate_component.hpp"
#include "bica_msgs/srv/deactivate_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace bica {

/**
 * @brief 激活组件响应结构体
 * @param ActivateComponent_Response 组件激活响应类型
 * @details 包含一个共享的激活组件响应指针和组件名称
 */
struct ActivationFuture {
  using ActivateComponent_Response =
      std::shared_ptr<bica_msgs::srv::ActivateComponent_Response_<std::allocator<void>>>;

  std::shared_future<ActivateComponent_Response> future;  // 共享的激活组件响应指针
  std::string component;                                  // 组件名称
};

/**
 * @brief 停用组件响应结构体
 * @param DeactivateComponent_Response 组件停用响应类型
 * @details 包含一个共享的停用组件响应指针和组件名称
 */
struct DeactivationFuture {
  using DeactivateComponent_Response =
      std::shared_ptr<bica_msgs::srv::DeactivateComponent_Response_<std::allocator<void>>>;

  std::shared_future<DeactivateComponent_Response> future;  // 共享的停用组件响应指针
  std::string component;                                    // 组件名称
};

/*
  这段代码是一个BICA组件类，继承自rclcpp_lifecycle::LifecycleNode。它包含了控制循环的方法、配置域的方法、激活、停用、清理、关闭和错误处理的方法。此外，它还包含了检查rclcpp是否正常并执行周期任务的方法。
  get_rate()方法返回配置的速率（作为rclcpp::Rate），而get_rate_as_freq()方法返回配置的速率（作为浮点数）。
*/

/**
 * @brief BICA组件类，继承自rclcpp_lifecycle::LifecycleNode
 */
class Component : public rclcpp_lifecycle::LifecycleNode {
public:
  /// 创建一个BICA组件
  /**
   * \param[in] id 组件的名称
   * \param[in] rate 组件的执行频率，默认为1.0
   */
  explicit Component(const std::string& id, float rate = 1.0);
  virtual ~Component() {}

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// 此方法包含控制循环。它是阻塞的
  virtual void execute();

  /// 此方法包含控制循环的一步
  /**
   * \param[in] spin 如果为true，则调用rclcpp :: spin_once并休眠以实现执行速率。
   *            如果为False，则用户应该调用控制频率并旋转它。
   */
  virtual void execute_once(bool spin = true);

  /// 通过创建DomainExpert对象配置域
  /**
   * \param[in] state 生命周期节点的状态
   * \return 成功或失败
   */
  virtual CallbackReturnT on_configure(const rclcpp_lifecycle::State& state);

  /// 激活节点
  /**
   * \param[in] state 生命周期节点的状态
   * \return 成功或失败
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State& state);

  /// 停用节点
  /**
   * \param[in] state 生命周期节点的状态
   * \return 成功或失败
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State& state);

  /// 清理节点
  /**
   * \param[in] state 生命周期节点的状态
   * \return 成功或失败
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State& state);

  /// 关闭节点
  /**
   * \param[in] state 生命周期节点的状态
   * \return 成功或失败
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State& state);

  /// 管理节点中的错误
  /**
   * \param[in] state 生命周期节点的状态
   * \return 成功或失败
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State& state);

  /// 检查rclcpp是否正常，并执行周期任务
  /**
   * \return rclcpp :: ok();
   */
  bool ok();

  /// 包含进程功能
  virtual void step() {}

  /// 返回此组件配置的速率（作为rclcpp：Rate）
  /**
   * \return 配置的速率
   */
  rclcpp::Rate& get_rate() { return rate_; }

  /// 返回此组件配置的速率（作为浮点数）
  /**
   * \return 配置的速率
   */
  float get_rate_as_freq() { return rate_freq_; }

private:
  rclcpp::Rate rate_;
  float rate_freq_;

  void addDependency(const std::string& dep);
  void removeDependency(const std::string& dep);

protected:
  virtual void on_activate() {}
  virtual void on_deactivate() {}

private:
  rclcpp::Rate rate_;
  float rate_freq_;

  std::set<std::string> dependencies_;
  std::set<std::string> activators_;

  std::vector<ActivationFuture> pending_act_futures_;
  std::vector<DeactivationFuture> pending_deact_futures_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr activation_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr activation_sub_;
  rclcpp::Service<bica_msgs::srv::ActivateComponent>::SharedPtr activation_service_;
  rclcpp::Service<bica_msgs::srv::DeactivateComponent>::SharedPtr deactivation_service_;

  void notifyStart();
  bool isDependency(const std::string& dep);
  void activateDependency(const std::string& dep);
  void deactivateDependency(const std::string& dep);
  void removeActivator(const std::string& activator);

  void activateDependencies();
  void deActivateDependencies();

  void activations_callback(const std_msgs::msg::String::SharedPtr msg);
  void activate_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<bica_msgs::srv::ActivateComponent::Request> request,
      const std::shared_ptr<bica_msgs::srv::ActivateComponent::Response> response);
  void deactivate_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<bica_msgs::srv::DeactivateComponent::Request> request,
      const std::shared_ptr<bica_msgs::srv::DeactivateComponent::Response> response);
};

}  // namespace bica

#endif  // BICA__COMPONENT_HPP_
