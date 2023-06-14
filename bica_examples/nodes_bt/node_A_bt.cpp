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
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "bica/Component.hpp"
#include "bica_behavior_tree/BicaTree.hpp"
#include "bica_behavior_tree/action/activation_action.hpp"
#include "rclcpp/rclcpp.hpp"

/*
  以上代码定义了三个类 Phase1、Phase2 和 Phase3，它们都继承自 ActivationActionNode 类，并重写了
  tick() 函数。每个类的构造函数需要传入一个字符串作为节点的名称。在 tick()
  函数中，会输出计数器的值并将其加一，当计数器大于等于 5 时返回 BT::NodeStatus::SUCCESS，否则返回
  BT::NodeStatus::RUNNING。这些类主要用于 BICA 组件中 behavior_tree 的相关操作。
*/

/**
 * @brief Phase1 类，继承自 ActivationActionNode 类
 * @param name 该节点的名称
 * @details 该类重写了 tick() 函数，每次调用时输出计数器的值并将其加一，当计数器大于等于 5 时返回
 * SUCCESS，否则返回 RUNNING。
 */
class Phase1 : public bica_behavior_tree::ActivationActionNode {
public:
  explicit Phase1(const std::string& name)
      : bica_behavior_tree::ActivationActionNode(name), counter_(0) {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override {
    std::cerr << "CompA::Phase1 tick() " << counter_ << std::endl;

    if (counter_++ < 5) {
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  int counter_;
};

/**
 * @brief Phase2 类，继承自 ActivationActionNode 类
 * @param name 该节点的名称
 * @details 该类重写了 tick() 函数，每次调用时输出计数器的值并将其加一，当计数器大于等于 5 时返回
 * SUCCESS，否则返回 RUNNING。
 */
class Phase2 : public bica_behavior_tree::ActivationActionNode {
public:
  explicit Phase2(const std::string& name)
      : bica_behavior_tree::ActivationActionNode(name), counter_(0) {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override {
    std::cerr << "CompA::Phase2 tick() " << counter_ << std::endl;

    if (counter_++ < 5) {
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  int counter_;
};

/**
 * @brief Phase3 类，继承自 ActivationActionNode 类
 * @param name 该节点的名称
 * @details 该类重写了 tick() 函数，每次调用时输出计数器的值并将其加一，当计数器大于等于 5 时返回
 * SUCCESS，否则返回 RUNNING。
 */
class Phase3 : public bica_behavior_tree::ActivationActionNode {
public:
  explicit Phase3(const std::string& name)
      : bica_behavior_tree::ActivationActionNode(name), counter_(0) {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override {
    std::cerr << "CompA::Phase3 tick() " << counter_ << std::endl;

    if (counter_++ < 5) {
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  int counter_;
};

/**
 * @brief CompA 类，继承自 bica::Component 类
 * @param 无
 * @details 定义了 CompA 的构造函数、on_activate() 函数和 step() 函数
 */
class CompA : public bica::Component {
public:
  /**
   * @brief CompA 构造函数
   * @param 无
   * @details 调用了父类 bica::Component 的构造函数，传入参数 "A" 和 1，并初始化 finished_ 为
   * false。同时注册了 Phase1、Phase2 和 Phase3 三个节点类型，以及两个插件
   * activation_sequence_bt_node 和 activation_action_bt_node。
   */
  CompA() : bica::Component("A", 1), finished_(false) {
    factory_.registerNodeType<Phase1>("Phase1");
    factory_.registerNodeType<Phase2>("Phase2");
    factory_.registerNodeType<Phase3>("Phase3");
    factory_.registerFromPlugin(BT::SharedLibrary().getOSName("activation_sequence_bt_node"));
    factory_.registerFromPlugin(BT::SharedLibrary().getOSName("activation_action_bt_node"));
  }

  /**
   * @brief on_activate() 函数
   * @param 无
   * @details 获取 bica_examples 包的路径，读取 nodes_bt 文件夹下的 node_A_tree.xml 文件，并使用
   * BehaviorTreeFactory 创建行为树 tree_。然后对 Phase1、Phase2 和 Phase3
   * 三个节点进行配置，设置它们的激活条件和依赖关系。
   */
  void on_activate() {
    std::string pkgpath = ament_index_cpp::get_package_share_directory("bica_examples");
    std::string xml_file = pkgpath + "/nodes_bt/node_A_tree.xml";

    tree_ = factory_.createTreeFromFile(xml_file);
    tree_.configureActivations<Phase1>("Phase1", this, {"B"});
    tree_.configureActivations<Phase2>("Phase2", this, {"D", "C"});
    tree_.configureActivations<Phase3>("Phase3", this, {"C"});
  }

  /**
   * @brief step() 函数
   * @param 无
   * @details 如果 finished_ 为 false，则执行行为树的根节点，直到返回状态为
   * BT::NodeStatus::SUCCESS。然后将 finished_ 设置为 true。
   */
  void step() {
    if (!finished_) {
      finished_ = tree_.root_node->executeTick() == BT::NodeStatus::SUCCESS;
    }
  }

private:
  BT::BehaviorTreeFactory factory_;    // 行为树工厂类对象
  bica_behavior_tree::BicaTree tree_;  // BICA 行为树对象
  bool finished_;                      // 标记行为树是否执行完成
};

/**
 * @brief 主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @details 初始化 ROS 2 节点，创建 CompA 类对象 component 并执行其 execute() 函数，最后关闭 ROS 2
 * 节点并返回 0。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto component = std::make_shared<CompA>();
  component->execute();
  rclcpp::shutdown();

  return 0;
}
