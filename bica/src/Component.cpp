/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */
/*
 * Component.cpp
 *
 *  Created on: 11/05/2016
 *      Author: paco
 */

#include <bica/Component.h>

#include <list>
#include <string>

namespace bica {

Component::Component() : nh_(), active_(false), root_(true) {
  // 声明并初始化服务端对象activation_srv_和deactivation_srv_
  // 用于接收来自其他节点的激活和去活请求
  // ros::this_node::getName()获取当前节点名称
  // &Component::activateCallback, this是回调函数和回调函数所属对象
  activation_srv_ = nh_.advertiseService(
      ros::this_node::getName() + "/activate", &Component::activateCallback, this);
  deactivation_srv_ = nh_.advertiseService(
      ros::this_node::getName() + "/deactivate", &Component::deActivateCallback, this);

  // 声明并初始化发布者对象alive_pub_
  // 发布当前节点的状态信息，表示节点处于激活状态
  alive_pub_ = nh_.advertise<std_msgs::Empty>(ros::this_node::getName() + "/active", 1, false);
}

// 析构函数
Component::~Component() { ROS_DEBUG("Killed destructor"); }

// 设置节点是否处于激活状态
void Component::setActive(bool act) {
  // 如果act为true，则执行激活操作，并输出日志信息
  if (act) {
    activateCode();
    ROS_DEBUG("[%s] Activation", ros::this_node::getName().c_str());
  } else {
    // 如果act为false，则执行去活操作，并输出日志信息
    if (root_) deActivateCode();
    ROS_DEBUG("[%s] Deactivation", ros::this_node::getName().c_str());
  }
  active_ = act;
}

// 激活回调函数
bool Component::activateCallback(
    ros::ServiceEvent<std_srvs::Empty::Request, std_srvs::Empty::Response>& call) {
  // 输出日志信息，表示收到了来自调用方的请求
  ROS_INFO("[%s] start from [%s]", ros::this_node::getName().c_str(), call.getCallerName().c_str());
  root_ = false;
  // 获取调用方名称
  std::string caller = call.getCallerName().substr(1, call.getCallerName().length());

  // 在激活列表中查找是否已存在该调用方的激活信息
  std::list<Activation>::iterator it = activations_.begin();
  while ((it != activations_.end()) && (it->id != caller)) ++it;

  // 如果激活列表为空或者未找到该调用方的激活信息，则添加新的激活信息
  if (activations_.empty() || it == activations_.end()) {
    Activation newact(caller, true, true);

    // 如果当前节点处于非激活状态，则执行激活操作
    if (!active_) activateCode();

    active_ = true;
    activations_.push_back(newact);

    // 输出日志信息，表示添加了新的激活信息
    ROS_INFO(
        "[%s] Added an activation from [%s]", ros::this_node::getName().c_str(), caller.c_str());
  } else
    // 如果已存在该调用方的激活信息，则输出警告信息
    ROS_WARN(
        "[%s] multiple activations from [%s]", ros::this_node::getName().c_str(), caller.c_str());

  return true;
}

/*
  包含了三个函数：deActivateCallback()、activateCode() 和
  deActivateCode()，以及一个激活组件的函数 activate()。

  其中，deActivateCallback() 函数是一个回调函数，用于停止组件。当有服务请求来自 activator
  时，会调用此函数。函数首先获取请求者的名称，然后在 activations_
  列表中查找该请求者是否存在。如果存在，则将其从列表中删除；否则输出错误信息。最后，如果
  activations_ 列表为空，则设置 active_ 标志为 false，并调用 deActivateCode() 函数。

  activateCode() 函数是一个启动组件的函数，当组件被激活时，会调用此函数。函数输出启动信息。

  deActivateCode() 函数是一个停止组件的函数，当组件被停止时，会调用此函数。函数输出停止信息。

  activate() 函数是一个激活组件的函数，当有服务请求来自 activator
  时，会调用此函数。函数首先获取本节点的名称，然后判断请求者是否为本节点。如果是，则直接返回；否则判断该请求者是否为依赖项。如果不是，则将其添加到依赖项列表中；否则输出警告信息。
*/

/**
 * @brief 用于停止组件。
 * @param call 服务事件对象，包含请求和响应。
 * @details 当有服务请求来自 activator 时，会调用此函数。函数首先获取请求者的名称，然后在
 * activations_
 * 列表中查找该请求者是否存在。如果存在，则将其从列表中删除；否则输出错误信息。最后，如果
 * activations_ 列表为空，则设置 active_ 标志为 false，并调用 deActivateCode() 函数。
 */
bool Component::deActivateCallback(
    ros::ServiceEvent<std_srvs::Empty::Request, std_srvs::Empty::Response>& call) {
  ROS_DEBUG("[%s] stop from [%s]", ros::this_node::getName().c_str(), call.getCallerName().c_str());

  std::string caller = call.getCallerName().substr(1, call.getCallerName().length());

  // 在 activations_ 列表中查找请求者
  std::list<Activation>::iterator it = activations_.begin();
  while (it != activations_.end() && (it->id != caller)) ++it;

  // 如果请求者存在，则将其从列表中删除；否则输出错误信息
  if (it != activations_.end()) {
    ROS_DEBUG("[%s] removing activation [%s]", ros::this_node::getName().c_str(), caller.c_str());
    it = activations_.erase(it);
  } else
    ROS_ERROR("[%s] from a non activator [%s]", ros::this_node::getName().c_str(), caller.c_str());

  // 如果 activations_ 列表为空，则设置 active_ 标志为 false，并调用 deActivateCode() 函数
  if (activations_.empty()) {
    active_ = false;
    deActivateCode();
  }

  return true;
}

/**
 * @brief 用于启动组件。
 * @details 当组件被激活时，会调用此函数。函数输出启动信息。
 */
void Component::activateCode() { ROS_INFO("[%s] start code", ros::this_node::getName().c_str()); }

/**
 * @brief 用于停止组件。
 * @details 当组件被停止时，会调用此函数。函数输出停止信息。
 */
void Component::deActivateCode() {
  ROS_INFO("[%s] Component stop code", ros::this_node::getName().c_str());
}

/**
 * @brief 用于激活组件。
 * @param id 请求者的名称。
 * @details 当有服务请求来自 activator
 * 时，会调用此函数。函数首先获取本节点的名称，然后判断请求者是否为本节点。如果是，则直接返回；否则判断该请求者是否为依赖项。如果不是，则将其添加到依赖项列表中；否则输出警告信息。
 */
void Component::activate(const std::string& id) {
  std::string caller = ros::this_node::getName();

  // 判断请求者是否为本节点，如果是，则直接返回
  if (id == caller) return;

  // 判断该请求者是否为依赖项。如果不是，则将其添加到依赖项列表中；否则输出警告信息。
  if (!isDependency(id)) {
    Dependency newdep;
    newdep.id = id;
    newdep.activated = false;
  } else
    ROS_WARN(
        "[%s] trying to activate [%s], which is not a dependency. Add it first.",
        ros::this_node::getName().c_str(), id.c_str());
}

/*
-
deActivate函数用于停用指定id的依赖项。首先构造服务名，然后遍历依赖项列表，找到要停用的依赖项，调用其对应的服务进行停用操作。
-
isDependency函数用于判断给定的id是否为依赖项。首先判断依赖项列表是否为空，如果为空则返回false；否则遍历依赖项列表，查找是否存在与给定id相同的依赖项。
-
addDependency函数用于添加一个依赖项。如果依赖项不存在，则将其添加到依赖项列表中；否则输出警告信息。
*/

/**
 * @brief 组件的deActivate函数，用于停用指定id的依赖项
 * @param id 要停用的依赖项的id
 * @details 遍历依赖项列表，找到要停用的依赖项，调用其对应的服务进行停用操作
 */
void Component::deActivate(const std::string& id) {
  std::string caller = ros::this_node::getName();

  // 构造服务名
  std::string srv_name = "/" + id + "/deactivate";

  // 查找要停用的依赖项
  std::list<Dependency>::iterator it = dependencies_.begin();
  while (it != dependencies_.end() && it->id != id) ++it;

  // 如果找到了要停用的依赖项，则调用其对应的服务进行停用操作
  if (it != dependencies_.end()) {
    ros::ServiceClient auxclient = nh_.serviceClient<std_srvs::Empty>(srv_name);
    std_srvs::Empty srv;

    // 调用停用服务
    it->activated = !auxclient.call(srv);
  }
}

/**
 * @brief 判断给定的id是否为依赖项
 * @param dep 要判断的id
 * @return bool 是否为依赖项
 * @details 遍历依赖项列表，查找是否存在与给定id相同的依赖项
 */
bool Component::isDependency(const std::string& dep) {
  ROS_DEBUG(
      "[%s] checking for dependency in %zu", ros::this_node::getName().c_str(),
      dependencies_.size());

  // 如果依赖项列表为空，则返回false
  if (dependencies_.empty()) return false;

  // 遍历依赖项列表，查找是否存在与给定id相同的依赖项
  std::list<Dependency>::iterator it = dependencies_.begin();
  ROS_DEBUG("\tF [%s][%s]", it->id.c_str(), dep.c_str());
  while (it != dependencies_.end()) {
    bool val = it->id == dep;
    if (val) {
      ROS_DEBUG("[%s] == [%s]", it->id.c_str(), dep.c_str());
      break;
    } else {
      ROS_DEBUG("[%s] != [%s]", it->id.c_str(), dep.c_str());
      ++it;
    }
  }

  return it != dependencies_.end();
}

/**
 * @brief 添加一个依赖项
 * @param dep 要添加的依赖项
 * @details 如果依赖项不存在，则将其添加到依赖项列表中；否则输出警告信息
 */
void Component::addDependency(const Dependency& dep) {
  // 如果依赖项不存在，则将其添加到依赖项列表中
  if (!isDependency(dep.id)) {
    ROS_DEBUG("[%s] adding dependecy [%s]", ros::this_node::getName().c_str(), dep.id.c_str());
    dependencies_.push_back(dep);
  } else
    // 否则输出警告信息
    ROS_WARN("[%s] NOT adding dependecy [%s]", ros::this_node::getName().c_str(), dep.id.c_str());
}

/*
  其中包含了向组件添加依赖项、从组件中删除依赖项以及激活组件的依赖项等功能。

  具体而言，这段代码实现了三个函数：

  1. `addDependency`：向组件添加依赖项，创建一个新的依赖项并将其添加到组件的依赖项列表中。
  2.
  `removeDependency`：从组件中删除依赖项，创建一个新的依赖项并将其从组件的依赖项列表中删除。如果该依赖项处于激活状态，则调用`deActivate`函数将其停用。
  3.
  `activateDependencies`：激活组件的依赖项，遍历组件的依赖项列表，对于每个未激活且处于活动状态的依赖项，调用其`activate`服务进行激活。如果激活成功，则将该依赖项的`activated`标志设置为true。

  主要是在输出一些调试信息，例如“removing dependency”、“is alive”、“is not
  alive”等。除此之外，还有一些变量和函数名也能够提供一定的帮助，例如`dependencies_`表示组件的依赖项列表，`deActivate`函数用于停用依赖项等。
*/

/**
 * @brief 向组件添加依赖项
 * @param dep 依赖项名称
 * @details 创建一个新的依赖项并将其添加到组件的依赖项列表中。
 */
void Component::addDependency(const std::string& dep) {
  Dependency newdep(dep, false);
  addDependency(newdep);
}

/**
 * @brief 从组件中删除依赖项
 * @param dep 依赖项名称
 * @details 创建一个新的依赖项并将其从组件的依赖项列表中删除。
 */
void Component::removeDependency(const std::string& dep) {
  Dependency remdep(dep, false);
  removeDependency(remdep);
}

/**
 * @brief 从组件中删除依赖项
 * @param dep 要删除的依赖项
 * @details
 * 从组件的依赖项列表中查找要删除的依赖项，如果找到则删除它。如果该依赖项处于激活状态，则调用deActivate函数将其停用。
 */
void Component::removeDependency(const Dependency& dep) {
  ROS_DEBUG("[%s] removing dependency [%s]", ros::this_node::getName().c_str(), dep.id.c_str());

  std::list<Dependency>::iterator it = std::find(dependencies_.begin(), dependencies_.end(), dep);
  if (it != dependencies_.end()) {
    if (it->alive) {
      ROS_DEBUG("[%s] is alive", ros::this_node::getName().c_str());
      deActivate(dep.id);
    } else
      ROS_DEBUG("[%s] is not alive", ros::this_node::getName().c_str());

    ROS_DEBUG("[%s] is found", ros::this_node::getName().c_str());
    it = dependencies_.erase(it);
  } else
    ROS_DEBUG(
        "[%s] is not found dependency [%s]", ros::this_node::getName().c_str(), dep.id.c_str());
}

/*
  其中包含了两个函数，分别是activateDependencies和deActivateDependencies。这两个函数都是用于激活或取消激活组件的依赖项。

  在activateDependencies函数中，首先判断组件是否已经处于激活状态，如果未激活则直接返回。然后遍历组件的依赖列表，对于每个依赖项，如果其未被激活且存活，并且不是当前节点本身，则调用其服务进行激活操作。如果激活成功，则将该依赖项标记为已激活。如果激活失败，则记录错误信息。对于当前节点本身的依赖项，直接标记为已激活。

  在deActivateDependencies函数中，同样遍历组件的依赖列表，对于每个依赖项，如果其已被激活且存活，并且不是当前节点本身，则调用其服务进行取消激活操作。如果取消激活成功，则将该依赖项标记为未激活。如果取消激活失败，则记录错误信息。对于当前节点本身的依赖项，直接标记为未激活。
*/

/**
 * @brief 激活组件依赖项，如果组件未激活，则直接返回
 * @param 无
 * @details
 * 遍历组件的依赖列表，对于每个依赖项，如果其未被激活且存活，并且不是当前节点本身，则调用其服务进行激活操作。
 *          如果激活成功，则将该依赖项标记为已激活。如果激活失败，则记录错误信息。
 *          对于当前节点本身的依赖项，直接标记为已激活。
 */
void Component::activateDependencies() {
  if (!active_) return;

  for (std::list<Dependency>::iterator it = dependencies_.begin(); it != dependencies_.end();
       ++it) {
    ROS_DEBUG("[%s] dep [%s]", ros::this_node::getName().c_str(), it->id.c_str());

    if (!it->activated && it->alive && it->id != ros::this_node::getName()) {
      std::string srv_name = "/" + it->id + "/activate";
      ros::ServiceClient auxclient = nh_.serviceClient<std_srvs::Empty>(srv_name);
      std_srvs::Empty srv;

      it->activated = auxclient.call(srv);
      if (it->activated)
        ROS_DEBUG("[%s] activated [%s]", ros::this_node::getName().c_str(), it->id.c_str());
      else
        ROS_ERROR(
            "[%s] Failed to activate [%s]", ros::this_node::getName().c_str(), it->id.c_str());
    }

    if (it->id == ros::this_node::getName()) it->activated = true;
  }
}

/**
 * @brief 取消激活组件依赖项
 * @param 无
 * @details
 * 遍历组件的依赖列表，对于每个依赖项，如果其已被激活且存活，并且不是当前节点本身，则调用其服务进行取消激活操作。
 *          如果取消激活成功，则将该依赖项标记为未激活。如果取消激活失败，则记录错误信息。
 *          对于当前节点本身的依赖项，直接标记为未激活。
 */
void Component::deActivateDependencies() {
  for (std::list<Dependency>::iterator it = dependencies_.begin(); it != dependencies_.end();
       ++it) {
    if (it->activated && it->alive && it->id != ros::this_node::getName()) {
      std::string srv_name = "/" + it->id + "/deactivate";
      ros::ServiceClient auxclient = nh_.serviceClient<std_srvs::Empty>(srv_name);
      std_srvs::Empty srv;

      it->activated = !auxclient.call(srv);
      if (it->activated)
        ROS_DEBUG("[%s] deactivated [%s]", ros::this_node::getName().c_str(), it->id.c_str());
      else
        ROS_ERROR(
            "[%s] Failed to deactivate [%s]", ros::this_node::getName().c_str(), it->id.c_str());

      if (it->id == ros::this_node::getName()) it->activated = false;
    }
  }
}

/**
 * @brief 打印组件的依赖和激活状态
 * @param 无
 * @details 遍历依赖列表和激活列表，打印每个依赖和激活的状态信息
 * 该函数用于打印组件的依赖和激活状态。首先遍历依赖列表并打印每个依赖的状态信息，然后遍历激活列表并打印每个激活的状态信息。其中，依赖和激活都是由自定义的结构体
 * Dependency 和 Activation 组成的，它们包含了各自的 id、activated、alive 等信息。函数中使用
 * ROS_DEBUG 宏来打印调试信息，其中 %s 表示字符串类型，? 表示布尔类型，\t 表示制表符。
 */
void Component::printStatus() {
  ROS_DEBUG("[%s] Dependencies", ros::this_node::getName().c_str());

  // 遍历依赖列表并打印状态信息
  for (std::list<Dependency>::iterator it = dependencies_.begin(); it != dependencies_.end(); ++it)
    ROS_DEBUG(
        "\t[%s] activated: %s\talive: %s", it->id.c_str(), it->activated ? "true" : "false",
        it->alive ? "true" : "false");
  ROS_DEBUG("[%s] Activations", ros::this_node::getName().c_str());

  // 遍历激活列表并打印状态信息
  for (std::list<Activation>::iterator it = activations_.begin(); it != activations_.end(); ++it)
    ROS_DEBUG(
        "\t[%s] active: %s\talive: %s", it->id.c_str(), it->active ? "true" : "false",
        it->alive ? "true"
                  : "fals"
                    "e");
}

/**
 * @brief 检查组件依赖的生命周期状态是否为激活状态
 * @details
 * 遍历组件依赖列表，检查每个依赖的生命周期状态是否为激活状态，如果不是则将其激活标志设置为false
 */
void Component::checkDependenciesAlive() {
  if (!active_) return;

  for (std::list<Dependency>::iterator it = dependencies_.begin(); it != dependencies_.end();
       ++it) {
    it->checkAlive();
    if (!it->alive) it->activated = false;
  }
}

/**
 * @brief 检查激活器的生命周期状态是否为激活状态
 * @details 遍历激活器列表，检查每个激活器的生命周期状态是否为激活状态，如果不是则将其从列表中删除
 */
void Component::checkActivatorsAlive() {
  if (!active_) return;

  std::list<Activation>::iterator it = activations_.begin();

  while (it != activations_.end()) {
    it->checkAlive();
    if (!it->alive) {
      it = activations_.erase(it);
    } else
      ++it;
  }

  /**
   * @brief 如果没有激活器，则将组件的激活状态设置为false，并执行反激活代码
   * @details 如果激活器列表为空，则将组件的激活状态设置为false，并执行反激活代码
   */
  if (activations_.empty()) {
    active_ = false;
    deActivateCode();
  }
}

/**
 * @brief 检查组件是否处于激活状态
 * @details 如果组件处于激活状态，则执行激活依赖、步进和发布消息等操作；否则执行反激活依赖操作
 */
bool Component::ok() {
  // printStatus();
  if (!root_) checkActivatorsAlive();
  checkDependenciesAlive();

  if (active_) {
    ROS_DEBUG("[%s] Active", ros::this_node::getName().c_str());
    activateDependencies();

    step();

    alive_pub_.publish(std_msgs::Empty());
  } else {
    ROS_DEBUG("[%s] Not active", ros::this_node::getName().c_str());
    deActivateDependencies();
  }

  return ros::ok();
}

} /* namespace bica */
