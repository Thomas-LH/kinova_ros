### gazebo simulation
launch kinova_gazebo/launch/robot_launch.launch 文件　加载环境模型和虚拟控制器  
发布 jointTrajectory 命令到 /robot_name/effort_joint_trajectory_controller/command 话题控制模型  
**problem**:  
1. jointTrajectory 数据结构中　header.stamp 意义？
>  jointTrajectory.header.stamp 指定数据相关的时间　jointTrajectory.jointTrajectoryPoint.time_from_start 指该点相应的执行时刻(按照代码中的意义解读，目前未找到明确的词义解释)，此时间是相对于头文件的时间戳。轨迹上相邻两点之间的时间不应一致。
2. 连续发布命令到此话题不能正常执行后来发布的命令？
3. 轨迹点阵使用方法？
>  经过尝试，直接将jointTrajectory.points设置为多个点即可，即jointTrajectory.points = [point1, point2, ...　]。

