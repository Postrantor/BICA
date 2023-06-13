/*
 * test_b_node.cpp
 *
 *  Created on: 11/05/2016
 *      Author: paco
 */

#include <bica/Component.h>
#include <ros/console.h>
#include <ros/ros.h>

class TestD : public bica::Component {
public:
  TestD() {}

  ~TestD() {}

  void step() {
    if (!isActive()) return;

    ROS_INFO("[%s] step", ros::this_node::getName().c_str());
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "node_D");

  TestD test_a;

  ros::Rate loop_rate(7);
  while (test_a.ok()) {
    test_a.step();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
