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
 * Dependency.cpp
 *
 *  Created on: 12/05/2016
 *      Author: paco
 */

#include <bica/Dependency.h>

#include <string>
#include <vector>

namespace bica {
/**
 * @brief Dependency类的默认构造函数
 * @param 无
 * @details 初始化id为""，activated为false，alive为false
 */
Dependency::Dependency() : id(""), activated(false), alive(false) {}

/**
 * @brief Dependency类的带参数构造函数
 * @param id_i 依赖项的ID
 * @param activated_i 依赖项是否已被激活
 * @param alive_i 依赖项是否存活
 * @details 根据传入的参数初始化id、activated和alive
 */
Dependency::Dependency(std::string id_i, bool activated_i, bool alive_i)
    : id(id_i), activated(activated_i), alive(alive_i) {}

/**
 * @brief Dependency类的拷贝构造函数
 * @param other 被拷贝的Dependency对象
 * @details 将other的id、activated和alive值赋给当前对象
 */
Dependency::Dependency(const Dependency& other) {
  id = other.id;
  activated = other.activated;
  alive = other.alive;
}

/**
 * @brief Dependency类的析构函数
 * @param 无
 * @details 待实现
 */
Dependency::~Dependency() {
  // TODO(fmrico): Auto-generated destructor stub
}

/**
 * @brief 检查依赖项是否存活
 * @param 无
 * @details
 * 获取ROS系统中所有节点的名称列表，如果当前依赖项的ID在其中，则说明该依赖项存活，否则不存活
 */
void Dependency::checkAlive() {
  std::vector<std::string> nodes;
  if (!ros::master::getNodes(nodes)) ROS_WARN("failed to get list of nodes");

  std::vector<std::string>::iterator nodeit = std::find(nodes.begin(), nodes.end(), "/" + id);

  alive = !(nodeit == nodes.end());
}

} /* namespace bica */
