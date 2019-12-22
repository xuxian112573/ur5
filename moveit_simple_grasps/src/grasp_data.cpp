
#include <moveit_simple_grasps/grasp_data.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// C++
#include <math.h>
#define _USE_MATH_DEFINES

namespace moveit_simple_grasps
{
GraspData::GraspData() :
  // Fill in default values where possible:
  base_link_("/base_link"),
  grasp_depth_(0.12),//物体中心点到末端执行器的距离
  angle_resolution_(16),// 以PI / angle_resolution增量生成抓取
  approach_retreat_desired_dist_(0.6),// 预抓紧阶段应距抓握位置多远
  approach_retreat_min_dist_(0.4),// 预抓取阶段应距离抓握位置多远
  object_size_(0.04)//障碍物的尺寸
{}
  /**
   * \brief 从Yaml文件加载抓取数据（从roslaunch加载）
   * \param node handle - 允许命名空间
   * \param end effector name - 双手机器人的哪一侧要加载数据。 应与SRDF EE名称相对应
   * \return true on success
   */
bool GraspData::loadRobotGraspData(const ros::NodeHandle& nh, const std::string& end_effector)
{
  std::vector<std::string> joint_names;//关节名称
  std::vector<double> pre_grasp_posture; // todo: remove all underscore post-fixes当末端执行器处于“打开”位置时
  std::vector<double> grasp_posture;//当末端执行器处于“关闭”位置时
  std::vector<double> grasp_pose_to_eef;//
  std::vector<double> grasp_pose_to_eef_rotation;//
  double pregrasp_time_from_start;//
  double grasp_time_from_start;//
  std::string end_effector_name;//
  std::string end_effector_parent_link;//

  // 加载参数
  if (!nh.hasParam("base_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `base_link` missing from rosparam server. Did you load your end effector's configuration yaml file? Searching in namespace: " << nh.getNamespace());//rosparam服务器缺少配置参数`base_link`。 您是否加载了末端执行器的配置yaml文件？ 在名称空间中搜索：
    return false;
  }
  nh.getParam("base_link", base_link_);

  // Search within the sub-namespace of this end effector name在此末端执行器名称的子命名空间中搜索
  ros::NodeHandle child_nh(nh, end_effector);

  // 加载参数
  if (!child_nh.hasParam("pregrasp_time_from_start"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `pregrasp_time_from_start` missing from rosparam server. Did you load your end effector's configuration yaml file? Searching in namespace: " << child_nh.getNamespace());
    return false;
  }
  child_nh.getParam("pregrasp_time_from_start", pregrasp_time_from_start);

  // 加载参数
  if (!child_nh.hasParam("grasp_time_from_start"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `grasp_time_from_start` missing from rosparam server. Did you load your end effector's configuration yaml file?");
    return false;
  }
  child_nh.getParam("grasp_time_from_start", grasp_time_from_start);

  // 加载参数
  if (!child_nh.hasParam("end_effector_name"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `end_effector_name` missing from rosparam server. Did you load your end effector's configuration yaml file?");
    return false;
  }
  child_nh.getParam("end_effector_name", end_effector_name);

  // 加载参数
  if (!child_nh.hasParam("end_effector_parent_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `end_effector_parent_link` missing from rosparam server. Did you load your end effector's configuration yaml file?");
    return false;//rosparam服务器缺少配置参数`end_effector_parent_link`。 您是否加载了末端执行器的配置yaml文件？
  }
  child_nh.getParam("end_effector_parent_link", end_effector_parent_link);

  // Load a param
  if (!child_nh.hasParam("joints"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `joints` missing from rosparam server. Did you load your end effector's configuration yaml file?");
    return false;
  }
  XmlRpc::XmlRpcValue joint_list;
  child_nh.getParam("joints", joint_list);
  if (joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    for (int32_t i = 0; i < joint_list.size(); ++i)
    {
      ROS_ASSERT(joint_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      joint_names.push_back(static_cast<std::string>(joint_list[i]));
    }
  else
    ROS_ERROR_STREAM_NAMED("temp","joint list type is not type array???");

  if(child_nh.hasParam("pregrasp_posture"))
  {
    XmlRpc::XmlRpcValue preg_posture_list;
    child_nh.getParam("pregrasp_posture", preg_posture_list);
    ROS_ASSERT(preg_posture_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < preg_posture_list.size(); ++i)
    {
      ROS_ASSERT(preg_posture_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      pre_grasp_posture.push_back(static_cast<double>(preg_posture_list[i]));
    }
  }

  ROS_ASSERT(child_nh.hasParam("grasp_posture"));
  XmlRpc::XmlRpcValue grasp_posture_list;
  child_nh.getParam("grasp_posture", grasp_posture_list);
  ROS_ASSERT(grasp_posture_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < grasp_posture_list.size(); ++i)
  {
    ROS_ASSERT(grasp_posture_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    grasp_posture.push_back(static_cast<double>(grasp_posture_list[i]));
  }

  ROS_ASSERT(child_nh.hasParam("grasp_pose_to_eef"));
  XmlRpc::XmlRpcValue g_to_eef_list;
  child_nh.getParam("grasp_pose_to_eef", g_to_eef_list);
  ROS_ASSERT(g_to_eef_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < g_to_eef_list.size(); ++i)
  {
    // Cast to double OR int
    if (g_to_eef_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (g_to_eef_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `grasp_pose_to_eef` wrong data type - int or double required.");
        return false;
      }
      else
        grasp_pose_to_eef.push_back(static_cast<int>(g_to_eef_list[i]));
    }
    else
      grasp_pose_to_eef.push_back(static_cast<double>(g_to_eef_list[i]));
  }

  ROS_ASSERT(child_nh.hasParam("grasp_pose_to_eef_rotation"));
  XmlRpc::XmlRpcValue g_to_eef_rotation_list;
  child_nh.getParam("grasp_pose_to_eef_rotation", g_to_eef_rotation_list);
  ROS_ASSERT(g_to_eef_rotation_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < g_to_eef_rotation_list.size(); ++i)
  {
    // Cast to double OR int
    if (g_to_eef_rotation_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      if (g_to_eef_rotation_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `grasp_pose_to_eef_rotation` wrong data type - int or double required.");
        return false;
      }
      else
        grasp_pose_to_eef_rotation.push_back(static_cast<int>(g_to_eef_rotation_list[i]));
    }
    else
      grasp_pose_to_eef_rotation.push_back(static_cast<double>(g_to_eef_rotation_list[i]));
  }

  // -------------------------------
  // Convert generic grasp pose to this end effector's frame of reference, approach direction for short
//将通用的抓握姿势转换为该末端执行器的参考系，并简化为进近方向
  // Orientation
  ROS_ASSERT(grasp_pose_to_eef_rotation.size() == 3);
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(grasp_pose_to_eef_rotation[1]), Eigen::Vector3d::UnitY())); // turn on Z axis
  // TODO: rotate for roll and yaw also, not just pitch (unit y)
  // but i don't need that feature right now and it might be tricky
  //也可以旋转以进行滚动和偏航，而不仅仅是俯仰（y单位），但我现在不需要该功能，这可能很棘手
  grasp_pose_to_eef_pose_.orientation.x = quat.x();
  grasp_pose_to_eef_pose_.orientation.y = quat.y();
  grasp_pose_to_eef_pose_.orientation.z = quat.z();
  grasp_pose_to_eef_pose_.orientation.w = quat.w();

  // Position // approach vector?
  ROS_ASSERT(grasp_pose_to_eef.size() == 3);
  grasp_pose_to_eef_pose_.position.x = grasp_pose_to_eef[0];
  grasp_pose_to_eef_pose_.position.y = grasp_pose_to_eef[1];
  grasp_pose_to_eef_pose_.position.z = grasp_pose_to_eef[2];

  // -------------------------------
  // 如果指定，请创建抓握前的姿势
  if(!pre_grasp_posture.empty())
  {
    pre_grasp_posture_.header.frame_id = base_link_;
    pre_grasp_posture_.header.stamp = ros::Time::now();
    // Name of joints:
    pre_grasp_posture_.joint_names = joint_names;
    // Position of joints
    pre_grasp_posture_.points.resize(1);
    pre_grasp_posture_.points[0].positions = pre_grasp_posture;
    pre_grasp_posture_.points[0].time_from_start = ros::Duration(pregrasp_time_from_start);
  }
  // -------------------------------
  // 建立抓握姿势
  grasp_posture_.header.frame_id = base_link_;
  grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_posture_.joint_names = joint_names;
  // Position of joints
  grasp_posture_.points.resize(1);
  grasp_posture_.points[0].positions = grasp_posture;
  grasp_posture_.points[0].time_from_start = ros::Duration(grasp_time_from_start);

  // -------------------------------
  // SRDF Info
  ee_parent_link_ = end_effector_parent_link;
  ee_group_ = end_effector_name;

  // -------------------------------
  // Nums
  approach_retreat_desired_dist_ = 0.2; // 0.3;
  approach_retreat_min_dist_ = 0.06;
  // 物体中心点到末端执行器的距离
  grasp_depth_ = 0.06;// 为负数或0时，这将使对象的另一侧抓紧！ （如下面所示）

  // 以PI / angle_resolution增量生成抓取
  angle_resolution_ = 16; //TODO 参数化此参数，或移至操作界面

  // Debug
  //moveit_simple_grasps::SimpleGrasps::printObjectGraspData(grasp_data);

  return true;
}
  /**
   * \brief 更改机器人状态，以使与此抓取数据对应的末端执行器处于预抓握状态（打开）
   * \param joint state of robot
   * \return true on success
   */
bool GraspData::setRobotStatePreGrasp( robot_state::RobotStatePtr &robot_state )
{
  return setRobotState( robot_state, pre_grasp_posture_ );
}
/**
   * \brief 更改机器人状态，以使与此抓取数据对应的末端执行器处于抓握状态（关闭）
   * \param joint state of robot
   * \return true on success
   */
bool GraspData::setRobotStateGrasp( robot_state::RobotStatePtr &robot_state )
{
  return setRobotState( robot_state, grasp_posture_ );
}

  /**
   * \brief 更改机器人状态，以使与此抓取数据对应的末端执行器处于抓握姿势
   * \param joint state of robot
   * \param posture - 设置末端执行器的状态
   * \return true on success
   */
bool GraspData::setRobotState( robot_state::RobotStatePtr &robot_state, const trajectory_msgs::JointTrajectory &posture )
{
  // Do for every joint in end effector
  for (std::size_t i = 0; i < posture.joint_names.size(); ++i)
  {
    // Debug
    std::cout << "Setting joint " << posture.joint_names[i] << " to value " 
              << posture.points[i].positions[0] << std::endl;

    // Set joint position
    robot_state->setJointPositions( posture.joint_names[i],
                                    posture.points[i].positions );
  }
}
//调试数据到控制台
void GraspData::print()
{
  ROS_WARN_STREAM_NAMED("grasp_data","Debug Grasp Data variable values:");
  std::cout << "grasp_pose_to_eef_pose_: \n" <<grasp_pose_to_eef_pose_<<std::endl;
  std::cout << "pre_grasp_posture_: \n" <<pre_grasp_posture_<<std::endl;
  std::cout << "grasp_posture_: \n" <<grasp_posture_<<std::endl;
  std::cout << "base_link_: " <<base_link_<<std::endl;
  std::cout << "ee_parent_link_: " <<ee_parent_link_<<std::endl;
  std::cout << "ee_group_: " <<ee_group_<<std::endl;
  std::cout << "grasp_depth_: " <<grasp_depth_<<std::endl;
  std::cout << "angle_resolution_: " <<angle_resolution_<<std::endl;
  std::cout << "approach_retreat_desired_dist_: " <<approach_retreat_desired_dist_<<std::endl;
  std::cout << "approach_retreat_min_dist_: " <<approach_retreat_min_dist_<<std::endl;
  std::cout << "object_size_: " <<object_size_<<std::endl;
}

} // namespace
