
#ifndef MOVEIT_SIMPLE_GRASPS__GRASP_DATA_H_
#define MOVEIT_SIMPLE_GRASPS__GRASP_DATA_H_

// Ros
#include <ros/node_handle.h>

// Msgs
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_state/robot_state.h>

namespace moveit_simple_grasps
{

class GraspData
{
public:
  geometry_msgs::Pose grasp_pose_to_eef_pose_; // 将通用抓握姿势转换为此末端执行器的参考系
  trajectory_msgs::JointTrajectory pre_grasp_posture_; // 当末端执行器处于“打开”位置时
  trajectory_msgs::JointTrajectory grasp_posture_; // 当末端执行器处于“关闭”位置时
  std::string base_link_; // 指向z的全局框架的名称
  std::string ee_parent_link_; // 运动链中末端执行器之前的最后一个链接，例如 “ / gripper_roll_link”
  std::string ee_group_; // 末端执行器名称
  double grasp_depth_; // 物体中心点到末端执行器的距离
  int angle_resolution_; // 以PI / angle_resolution增量生成抓取
  double approach_retreat_desired_dist_; // 预抓紧阶段应距抓握位置多远
  double approach_retreat_min_dist_; // 预抓取阶段应距离抓握位置多远
  double object_size_; // 可视化

public:

  /**
   * \brief Constructor
   */
  GraspData();

  /**
   * \brief 从Yaml文件加载抓取数据（从roslaunch加载）
   * \param node handle - 允许命名空间
   * \param end effector name - 双手机器人的哪一侧要加载数据。 应与SRDF EE名称相对应
   * \return true on success
   */
  bool loadRobotGraspData(const ros::NodeHandle& nh, const std::string& end_effector);

  /**
   * \brief 更改机器人状态，以使与此抓取数据对应的末端执行器处于预抓握状态（打开）
   * \param joint state of robot
   * \return true on success
   */
  bool setRobotStatePreGrasp( robot_state::RobotStatePtr &robot_state );

  /**
   * \brief 更改机器人状态，以使与此抓取数据对应的末端执行器处于抓握状态（关闭）
   * \param joint state of robot
   * \return true on success
   */
  bool setRobotStateGrasp( robot_state::RobotStatePtr &robot_state );

  /**
   * \brief 更改机器人状态，以使与此抓取数据对应的末端执行器处于抓握姿势
   * \param joint state of robot
   * \param posture - 设置末端执行器的状态
   * \return true on success
   */
  bool setRobotState( robot_state::RobotStatePtr &robot_state, const trajectory_msgs::JointTrajectory &posture );

  /**
   * \brief 调试数据到控制台
   */
  void print();
};

} // namespace

#endif
