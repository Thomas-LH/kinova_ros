## gazebo simulation
launch kinova_gazebo/launch/robot_launch.launch 文件　加载环境模型和虚拟控制器  
发布 jointTrajectory 命令到 /robot_name/effort_joint_trajectory_controller/command 话题控制模型  
**problem**:  
1. jointTrajectory 数据结构中　header.stamp 意义？
>  jointTrajectory.header.stamp 指定数据相关的时间　jointTrajectory.jointTrajectoryPoint.time_from_start 指该点相应的执行时刻(按照代码中的意义解读，目前未找到明确的词义解释)，此时间是相对于头文件的时间戳。轨迹上相邻两点之间的时间不应一致。
2. 连续发布命令到此话题不能正常执行后来发布的命令？
>  使用轨迹点阵一次将路径发送完毕，不要多次发送单点
3. 轨迹点阵使用方法？
>  经过尝试，直接将jointTrajectory.points设置为多个点即可，即jointTrajectory.points = [point1, point2, ...　]。
4. moveit and rviz(似乎，gazebo不是必须的，使用rviz观察运动即可)
>  rviz可以看到规划的轨迹和执行动作，而gazebo更注重真实的物理环境模拟（摩擦力等等）
5. wiki指出使用Action要比直接发送数据给joint要更加灵活，需要了解Action相关
>  Action是一种带有过程反馈的服务


kinova 7dof 机械臂 与　智能底座
使用ros进行开发：ros是一个开源的机器人操作系统，它包含许多可供使用的工具和算法来帮助我们更好地开发机器人系统，一些优秀的软件包可针对特定的问题给出较好的结果。
ros通讯:话题（topic）,服务（server）,动作（action）
机械臂控制可使用moveit！软件包，在建立好实际机器人的模型后，在moveit！中可配置相关的参数（组，末端执行器等），之后可配置使用图形界面对机器人进行控制或使用脚本进行控制。图形界面使用rviz插件，脚本控制需使用move_group接口。

## grasp demo

1. 参考系的选取要注意，抓取的方向向量会受影响
2. 抓取位姿预设值要注意末端执行器的方向
>  面向坐标系，逆时针旋转为正
3. 目标位姿（抓取、放置）中最小距离，期望距离具体意义
>  指预接近（或离开）目标位姿的期望距离和最小距离
4. 抓取时，角度设置是指末端执行器的方向设置，而放置时角度设置是指所抓取的物体的方向设置。
5. pick and place 函数加入路径限制无效

### 场景限制
1. 系统服务面向双腿瘫痪或因事故不能行走的病人，目的在于扩展其运动范围。
此类病人双手可以正常移动，可以自主完成一些日常活动，并且他们可以使用传统的轮椅，因此我们的系统提供的是***室内自主导航以及抓取物品范围的扩展***。室内自动导航可以减轻病人的控制负担（平时控制仅需使用Hololens和脑电设定目标点即可），室外手动控制移动等同于现在的电动轮椅，抓取范围的扩展可以便利病人的日常生活（病人坐在任何轮椅上都会限制其抓取物品的空间范围，我们的系统由于增加机械臂的运动范围可以改善此方面的问题）。

