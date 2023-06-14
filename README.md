---
tip: translate by openai@2023-06-13 17:05:46
...

# BICA for ROS2

[](https://travis-ci.com/IntelligentRoboticsLabs/BICA)

**BICA (Behavior-based Iterative Component Architecture) provides a way to create ROS applications that implements complex behaviors in robots.**

BICA has a long history. It was designed and implemented in 2008 for the RoboCup SPL competition, being the behavioral architecture that allowed a Nao robot to play soccer in this competition. It was executed inside a NaoQi module and allowed to implement behaviors that arose from the concurrent execution of many components. There were perceptual components (image processing, sound ...), acting (walk, kick, leds, attention ...) or reasoning. Component hierarchies were created, where some components explicitly activated others, avoiding synchronization problems and saving computation time.

> BICA 有着悠久的历史。它于 2008 年为 RoboCup SPL 比赛设计并实施，是使 Nao 机器人参加这项比赛的行为架构。它在 NaoQi 模块中执行，允许实现由许多组件并发执行而产生的行为。有感知组件(图像处理、声音等)、行动(行走、踢球、LED、注意力等)或推理。创建了组件层次结构，其中一些组件明确激活其他组件，避免同步问题，节省计算时间。

With the arrival of ROS, the implementation changed, with each component being a ROS node that could define what other components needed for its operation. BICA executed the dependencies of each component. With this architecture we have participated in competitions such as RoCKIn (robot MYRAbot), RoboCup @Home (robot RB-1 Robotnik) and we continue using it, whith [ROSPlan](https://github.com/KCL-Planning/ROSPlan), in the RoboCup SSPL with the robot Pepper. We also use it in our industrial projects. It provides a mechanism to create independent components that only consume processing time when a component requires the result of its computation.

> 随着 ROS 的到来，实施方式发生了变化，每个组件都是一个 ROS 节点，可以定义其他组件所需的操作。 BICA 执行了每个组件的依赖关系。使用这种体系结构，我们参加了诸如 RoCKIn(机器人 MYRAbot)、RoboCup @Home(机器人 RB-1 Robotnik)等比赛，并继续使用它，使用[ROSPlan](https://github.com/KCL-Planning/ROSPlan)，在 RoboCup SSPL 上使用机器人 Pepper。我们也在我们的工业项目中使用它。它提供了一种机制，可以创建独立的组件，只有在组件需要其计算结果时才消耗处理时间。

You can find more references in these papers:

- **"Planning-centered Architecture for RoboCup SSPL @Home". Francisco Martín, Jonathan Ginés, David Vargas, Francisco J. Rodríguez-Lera, Vicente Matellán. WAF2018, publisehd in Advances in Intelligent Systems and Computing series. November 2018.**

> 计划中心的 RoboCup SSPL @Home 建筑。弗朗西斯科·马丁，乔纳森·吉纳斯，大卫·瓦加斯，弗朗西斯科·J·罗德里亚·勒拉，维森特·马特兰。WAF2018，发表于智能系统与计算系列。2018 年 11 月。

- "A Simple, Efficient, and Scalable Behavior-based Architecture for Robotic Applications", Francisco Martín Rico, Carlos E. Agüero Durán. ROBOT'2015 Second Iberian Robotics Conference.

> 一种简单、高效且可扩展的基于行为的机器人应用架构，Francisco Martín Rico, Carlos E. Agüero Durán. ROBOT'2015 第二届伊比利亚机器人会议。

- "Humanoid Soccer Player Design", Francisco Martín, Carlos Agüero, José María Cañas y Eduardo Perdices. Robot Soccer. Ed: Vladan Papic, pp 67-100. IN-TECH, ISBN, 978-953-307-036-0. 2010.

> 《人形足球运动员设计》，Francisco Martín，Carlos Agüero，José María Cañas 和 Eduardo Perdices 编著，机器人足球，Vladan Papic 编辑，67-100 页，IN-TECH 出版，ISBN 978-953-307-036-0，2010 年。

- "Behavior-based Iterative Component Architecture for soccer applications with the Nao humanoid". Carlos E. Agüero, Jose M. Canas, Francisco Martin and Eduardo Perdices 5Th Workshop on Humanoids Soccer Robots. Nashville, TN, USA. Dic.

> 行为基础的迭代组件架构，用于 Nao 人形机器人的足球应用。Carlos E. Agüero, Jose M. Canas, Francisco Martin 和 Eduardo Perdices 第五届人形机器人足球研讨会。Nashville, TN, USA. Dic.

Now we have released it so that anyone can take advantage of its robotic applications.

> 现在我们已经发布它，以便任何人都可以利用它的机器人应用程序。

## How it works

Each bica component is executed in a ROS node. Its life cycle is as follows:

> 每个 bica 组件都在 ROS 节点中执行。它的生命周期如下：

- Each component has its own frequency of execution.

- When you launch your application, your nodes will be running, but they will be inactive until some other component activates them. Thus hierarchies of components can be formed.

> 当你启动你的应用程序时，你的节点将会运行，但是它们会处于非活动状态，直到其他组件激活它们。因此可以形成组件的层次结构。

- A component declares what other components need to be running when it is active. They are its dependecies.

> 一个组件声明当它处于活动状态时，需要运行哪些其他组件。它们是它的依赖项。

- Un componente se activa cuando algún componente que lo declaró como depencia se activa. También se activa si este componente se declara como raiz de una jerarquía de ejecución.

> 一个组件在宣布它为依赖项的其他组件激活时被激活。如果将其声明为执行层次结构的根，也会激活它。

- A component is deactivated when there is no active component that has declared it as a dependency.

> 一个组件在没有活动组件声明它为依赖项时会被停用。

## Contributing

Contributions are welcome. Don't forger to run `colcon test` before.

> 欢迎提供贡献。在此之前不要忘记运行`colcon test`。
