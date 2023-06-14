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

#include <memory>

#include "bica/Component.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief BICA组件中的CompA类，继承自bica::Component
 * @param 无
 * @details CompA类在构造函数中添加了"B"和"C"两个依赖项，并实现了step()函数。
 */
class CompA : public bica::Component {
public:
  /**
   * @brief 构造函数，初始化CompA对象
   * @param 无
   * @details
   * 在构造函数中调用父类构造函数，设置组件名称为"A"，优先级为1，然后添加"B"和"C"两个依赖项。
   */
  CompA() : bica::Component("A", 1) {
    addDependency("B");
    addDependency("C");
  }

  /**
   * @brief 组件的执行函数
   * @param 无
   * @details 打印出"CompA::step()"信息。
   */
  void step() { RCLCPP_INFO(get_logger(), "CompA::step()"); }
};

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @details
 * 在主函数中，首先初始化ROS2节点，然后创建一个CompA对象，执行该对象的execute()函数，最后关闭ROS2节点并返回0。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto component = std::make_shared<CompA>();
  component->execute();
  rclcpp::shutdown();
  return 0;
}