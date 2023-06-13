---
tip: translate by openai@2023-06-13 15:06:27
...

# BICA

[![Build Status](https://travis-ci.com/IntelligentRoboticsLabs/BICA.svg?branch=master)](https://travis-ci.com/IntelligentRoboticsLabs/BICA)

> [](https://travis-ci.com/IntelligentRoboticsLabs/BICA)

**BICA (Behavior-based Iterative Component Architecture) provides a way to create ROS applications that implements complex behaviors in robots.**

> **BICA(基于行为的迭代组件体系结构)提供了一种创建 ROS 应用程序的方法，该应用程序在机器人中实现复杂的行为**。

BICA has a long history. It was designed and implemented in 2008 for the RoboCup SPL competition, being the behavioral architecture that allowed a Nao robot to play soccer in this competition. It was executed inside a NaoQi module and allowed to implement behaviors that arose from the concurrent execution of many components. There were perceptual components (image processing, sound ...), acting (walk, kick, leds, attention ...) or reasoning. Component hierarchies were created, where some components explicitly activated others, avoiding synchronization problems and saving computation time.

> BICA 有着悠久的历史。它于 2008 年为 RoboCup SPL 比赛设计和实现，成为允许 Nao 机器人参加比赛的行为架构。它在 NaoQi 模块中执行，**允许实现由多个组件并发执行而产生的行为**。有感知组件(图像处理、声音...)、行动(行走、踢球、LED 灯、注意力...)或推理。**创建了组件层次结构**，其中**一些组件明确激活其他组件，避免同步问题，节省计算时间**。

With the arrival of ROS, the implementation changed, with each component being a ROS node that could define what other components needed for its operation. BICA executed the dependencies of each component. With this architecture we have participated in competitions such as RoCKIn (robot MYRAbot), RoboCup @Home (robot RB-1 Robotnik) and we continue using it, whith [ROSPlan](https://github.com/KCL-Planning/ROSPlan), in the RoboCup SSPL with the robot Pepper. We also use it in our industrial projects. It provides a mechanism to create independent components that only consume processing time when a component requires the result of its computation.

> 随着 ROS 的到来，实施方式发生了变化，每个组件都是一个 ROS 节点，可以定义其他组件所需的操作。BICA 执行每个组件的依赖关系。使用这种架构，我们参加了诸如 RoCKIn(机器人 MYRAbot)、RoboCup @Home(机器人 RB-1 Robotnik)等比赛，并继续使用它，使用[ROSPlan](https://github.com/KCL-Planning/ROSPlan)参加 RoboCup SSPL 比赛，使用机器人 Pepper。我们还在工业项目中使用它。它提供了一种机制，可以创建独立的组件，只有在组件需要计算结果时才消耗处理时间。

You can find more references in these papers:

- **"Planning-centered Architecture for RoboCup SSPL @Home". Francisco Martín, Jonathan Ginés, David Vargas, Francisco J. Rodríguez-Lera, Vicente Matellán. WAF2018, publisehd in Advances in Intelligent Systems and Computing series. November 2018.**
- "A Simple, Efficient, and Scalable Behavior-based Architecture for Robotic Applications", Francisco Martín Rico, Carlos E. Agüero Durán. ROBOT'2015 Second Iberian Robotics Conference.
- "Humanoid Soccer Player Design", Francisco Martín, Carlos Agüero, José María Cañas y Eduardo Perdices. Robot Soccer. Ed: Vladan Papic, pp 67-100. IN-TECH, ISBN, 978-953-307-036-0. 2010.
- "Behavior-based Iterative Component Architecture for soccer applications with the Nao humanoid". Carlos E. Agüero, Jose M. Canas, Francisco Martin and Eduardo Perdices 5Th Workshop on Humanoids Soccer Robots. Nashville, TN, USA. Dic.

Now we have released it so that anyone can take advantage of its robotic applications.

> 现在我们已经发布了它，以便任何人都可以利用它的机器人应用程序。

## How it works

Each bica component is executed in a ROS node. Its life cycle is as follows:

> 每个 bica 组件都在 ROS 节点中执行。它的生命周期如下：

- Each component has its own frequency of execution.
- When you launch your application, your nodes will be running, but they will be inactive until some other component activates them. Thus hierarchies of components can be formed.
- A component declares what other components need to be running when it is active. They are its dependecies.
- Un componente se activa cuando algún componente que lo declaró como depencia se activa. También se activa si este componente se declara como raiz de una jerarquía de ejecución.
- A component is deactivated when there is no active component that has declared it as a dependency.

> - 每个组件都有自己的执行频率
> - 当您启动应用程序时，您的节点将运行，但在其他组件激活它们之前，它们将处于非活动状态。这样就可以**形成组件的层次结构**。
> - 组件声明当它处于活动状态时需要运行的其他组件。他们是它的依赖者。
> - 行动的组成部分是行动的一部分。También 的活动是声明的组成部分
> - 当没有活动组件将组件声明为依赖项时，该组件将被停用。

Example:

In this video we can show the execution of the nodes in bica_examples folder of this repo.

> 在这个视频中，我们可以展示这个仓库中 bica_examples 文件夹中节点的执行情况。

- Component A depends on B and C
- Component C depends con D
- Components log a message when they are active.

In this video we will:

1. Activate A, so B,C and D will be also active. For activating A we will usa a bica tool called launcher by executing `rosrun bica launcher node_A`

> 激活 A，因此 B、C 和 D 也将被激活。为了激活 A，我们将使用一个叫做 launcher 的 bica 工具，通过执行`rosrun bica launcher node_A`来实现。 2. Activate C, so D will be also active. 3. Activate B.

4. Activate A, so B,C and D will be also active. Then we will close C, so D will be inactive until we execute C again, which inmediatelly is active.

> 4.激活 A，因此 B、C 和 D 也将被激活。然后我们将关闭 C，因此 D 在我们再次执行 C 之前将处于非活动状态，一旦执行 C 即可立即激活。

[![BICA Example](https://img.youtube.com/vi/ozYrQdCbGA4/0.jpg)](https://www.youtube.com/watch?v=ozYrQdCbGA4)

> [![BICA示例](https://img.youtube.com/vi/ozYrQdCbGA4/0.jpg)](https://www.youtube.com/watch?v=ozYrQdCbGA4)

The code of a node that implements a BICA component is very simple:

> 节点实现 BICA 组件的代码非常简单：

```cpp
#include <ros/ros.h>
#include <ros/console.h>

#include <bica/Component.h>

class TestA: public bica::Component
{
public:
	TestA()
	{
    addDependency("node_B");
    addDependency("node_C");
	}

	void step()
	{
		if(!isActive()) return;

		ROS_INFO("[%s] step", ros::this_node::getName().c_str());
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_A");

	TestA test_a;

	ros::Rate loop_rate(10);
	while(test_a.ok())
	{
		test_a.step();

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
```

- A BICA component inherits from bica :: Component
- Declare its dependencies with `addDependency (id)`. `id` is the name of the node that implements this component (you can match it with the name of your component for clarity)
- In your `main()`, create an instance of this class and call the frequency you want to your step () method. The first thing you do in this method is to check if it is active.

> - BICA 组件继承自 BICA::component
> - 用`addDependency（id）`声明其依赖项`id` 是实现该组件的节点的名称（为了清晰起见，您可以将其与组件的名称相匹配）
> - 在您的 main（）中，创建该类的一个实例，并调用您想要的 step（）方法的 frequency。在这个方法中，您要做的第一件事是检查它是否处于活动状态。

You can also implement your components in Python:

> 你也可以用 Python 实现你的组件。

```python
#!/usr/bin/env python

import rospy
from bica.bica import Component

class TestP(Component):
  def step(self):
    rospy.loginfo("[" +  rospy.get_name() + "] step")

if __name__ == '__main__':
  rospy.init_node('node_P', anonymous=False)
  rate = rospy.Rate(10) # 10hz

  test_p = TestP(10)

  while test_p.ok():
    pass
```

## FSM and BICA-GUI

Any of your components can be a finite state machine, even creating hierarchical state machines. We have developed a tool to create these components graphically. This tool automatically generates the C ++ code.

> 任何你的组件都可以是有限状态机，甚至可以**创建分层状态机**。我们已经开发了一个图形化工具来创建这些组件。该工具可以自动生成 C ++代码。

![](https://raw.githubusercontent.com/IntelligentRoboticsLabs/BICA/images/images/hfsm.png)

You can define states, transitions and what components must be active in each state. You can even debug the active state at runtime.

> 你可以定义状态、转换以及每个状态中哪些组件必须处于活动状态。甚至可以在运行时调试活动状态。

[![BICA Example](https://img.youtube.com/vi/ImnmOF_CO1E/0.jpg)](https://www.youtube.com/watch?v=ImnmOF_CO1E)

> [![BICA示例](https://img.youtube.com/vi/ImnmOF_CO1E/0.jpg)](https://www.youtube.com/watch?v=ImnmOF_CO1E)

BICA GUI generates a class in C ++ from which you can inherit to create your component. You can implement the methods that you need:

> BICA GUI 生成一个 C++类，您可以从中继承以创建您的组件。您可以实现您需要的方法：

- Each state generates two methods `*_iterative` and `*_once`. The first is a method that will be called continually while the component is in that state. The second is called only once when it is transited to this state. In the base class they are empty. Redefine them in your component.
- Redefines the transitions and returns `true` when the transition has to be made.

> - 每个状态生成两个方法`*_iterive`和`*_once`。第一种是在组件处于该状态时将被连续调用的方法。第二个在转换到此状态时只调用一次。在基类中，它们是空的。在组件中重新定义它们。
> - 重新定义转换，并在必须进行转换时返回`true`。

Example: (Complete example y bica_examples)

```cpp
#include <ros/ros.h>
#include "test_M.h"


class test_M_impl: public bica::test_M
{

  void State_B_code_once()
  {
    ROS_INFO("[%s] State B ", ros::this_node::getName().c_str());
  }

  void State_C_code_iterative()
  {
    ROS_INFO("[%s] State C ", ros::this_node::getName().c_str());
  }

  void State_A_code_once()
  {
    ROS_INFO("[%s] State A ", ros::this_node::getName().c_str());
  }

  bool State_A_2_State_B()
  {
    return (ros::Time::now() - state_ts_).toSec() > 5.0;
  }

  bool State_B_2_State_C()
  {
    return (ros::Time::now() - state_ts_).toSec() > 5.0;
  }

  bool State_C_2_State_A()
  {
    return (ros::Time::now() - state_ts_).toSec() > 5.0;
  }

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_M");

	test_M_impl test_m;

	ros::Rate loop_rate(7);
	while(test_m.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
```

## Contributing

Contributions are welcome. Don't forger to run `catkin roslint` before.

> 欢迎提出贡献。在此之前不要忘记运行`catkin roslint`。
