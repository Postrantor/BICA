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
 * Activation.cpp
 *
 *  Created on: 12/05/2016
 *      Author: paco
 */

#include <bica/Activation.h>

#include <string>
#include <vector>

namespace bica {

/*
  其中，Activation类封装了节点的激活状态和存活状态，并提供了检查节点是否存活的方法checkAlive()和激活回调函数activeCB()。
  在构造函数中，订阅了话题"/" + id + "/active"，
  并将回调函数设置为&Activation::activeCB，this指针指向当前对象。同时，初始化last_heartbeat_为ros::Time::now()。在checkAlive()方法中，获取ROS网络中所有节点的名称，并在其中查找名字为
  "/" + id的节点。
  如果找到该节点，则alive为true；否则，alive为false。同时，检查最后一次心跳时间是否超过了DEAD_T（一个预设的常量），如果超过了，则check_hb为true；否则，check_hb为false。在activeCB()方法中，更新最后一次心跳时间为当前时间。
*/

/**
 * @brief Activation类的默认构造函数
 * @param 无
 * @details
 * 初始化Activation对象的私有成员变量nh_、id、active和alive，其中id、active和alive的值分别为默认值""、false和false。
 */
Activation::Activation() : nh_(), id(""), active(false), alive(false) {}

/**
 * @brief Activation类的带参构造函数
 * @param[in] id_i 用于初始化Activation对象的id成员变量
 * @param[in] active_i 用于初始化Activation对象的active成员变量
 * @param[in] alive_i 用于初始化Activation对象的alive成员变量
 * @details
 * 初始化Activation对象的私有成员变量id、active和alive，其中id、active和alive的值分别为传入参数id_i、active_i和alive_i的值。同时，订阅了话题"/"
 * + id +
 * "/active"，并将回调函数设置为&Activation::activeCB，this指针指向当前对象。最后，初始化last_heartbeat_为ros::Time::now()。
 */
Activation::Activation(std::string id_i, bool active_i, bool alive_i)
    : id(id_i), active(active_i), alive(alive_i) {
  sub_ = nh_.subscribe("/" + id + "/active", 1, &Activation::activeCB, this);
  last_heartbeat_ = ros::Time::now();
}

/**
 * @brief Activation类的拷贝构造函数
 * @param[in] other 用于初始化Activation对象的另一个Activation对象
 * @details
 * 将other对象的私有成员变量id、active和alive的值分别赋值给当前对象的对应成员变量。同时，订阅了话题"/"
 * + id +
 * "/active"，并将回调函数设置为&Activation::activeCB，this指针指向当前对象。最后，初始化last_heartbeat_为ros::Time::now()。
 */
Activation::Activation(const Activation &other) {
  id = other.id;
  active = other.active;
  alive = other.alive;

  sub_ = nh_.subscribe("/" + id + "/active", 1, &Activation::activeCB, this);
  last_heartbeat_ = ros::Time::now();
}

/**
 * @brief Activation类的析构函数
 * @param 无
 * @details 释放Activation对象所占用的资源。
 */
Activation::~Activation() {}

/**
 * @brief 检查节点是否存活
 * @param 无
 * @details 获取ROS网络中所有节点的名称，并在其中查找名字为"/" +
 * id的节点。如果找到该节点，则alive为true；否则，alive为false。同时，检查最后一次心跳时间是否超过了DEAD_T（一个预设的常量），如果超过了，则check_hb为true；否则，check_hb为false。
 */
void Activation::checkAlive() {
  std::vector<std::string> nodes;
  if (!ros::master::getNodes(nodes)) ROS_WARN("failed to get list of nodes");

  std::vector<std::string>::iterator nodeit = std::find(nodes.begin(), nodes.end(), "/" + id);

  bool check_hb = ((ros::Time::now() - last_heartbeat_).toSec() > DEAD_T);
  alive = !(nodeit == nodes.end());
}

/**
 * @brief 激活回调函数
 * @param[in] msg 用于激活的消息
 * @details 更新最后一次心跳时间为当前时间。
 */
void Activation::activeCB(const std_msgs::Empty::ConstPtr &msg) {
  last_heartbeat_ = ros::Time::now();
}

} /* namespace bica */
