

#ifndef MOVEIT_SIMPLE_GRASPS__MOVEIT_SIMPLE_GRASPS_H_
#define MOVEIT_SIMPLE_GRASPS__MOVEIT_SIMPLE_GRASPS_H_

// ROS
#include <ros/ros.h>

// TF
#include <tf_conversions/tf_eigen.h>

// Msgs
#include <geometry_msgs/PoseArray.h>

// MoveIt
#include <moveit_msgs/Grasp.h>
#include <moveit/macros/deprecation.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

// C++
#include <math.h>
#define _USE_MATH_DEFINES

#include <moveit_simple_grasps/grasp_data.h>

namespace moveit_simple_grasps
{

static const double RAD2DEG = 57.2957795;

// 握轴方向
enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};
enum grasp_direction_t {UP, DOWN};
enum grasp_rotation_t {FULL, HALF};

// Class
class SimpleGrasps
{
private:

  // 用于将内容发布到rviz的类
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // 从框框转换为全局框
  Eigen::Affine3d object_global_transform_;

  // 在控制台和Rviz中显示更多输出（带有箭头和标记）
  bool verbose_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen需要对Eigen :: Vector2d的数组（2个双精度）进行128位对齐。 对于GCC，这是通过属性（（aligned（16）））完成的。

  /**
   * \brief Constructor
   */
  SimpleGrasps(moveit_visual_tools::MoveItVisualToolsPtr rviz_tools, bool verbose = false);

  /**
   * \brief Destructor
   */
  ~SimpleGrasps();

  /**
   * \brief 移动到generateBlockGrasps
   */
  MOVEIT_DEPRECATED bool generateAllGrasps(const geometry_msgs::Pose& object_pose, const GraspData& grasp_data,
    std::vector<moveit_msgs::Grasp>& possible_grasps)
  {
    generateBlockGrasps(object_pose, grasp_data, possible_grasps);

    return true;
  }

  /**
   * \brief 为块创建所有可能的抓取位置
   * \param pose of block, where vector arrow is parallel to table plane块的姿势，矢量箭头与工作台平面平行
   * \param data 描述末端执行器
   * \param resulting generated possible grasps
   * \return true if successful
   */ 
  bool generateBlockGrasps(const geometry_msgs::Pose& object_pose, const GraspData& grasp_data,
    std::vector<moveit_msgs::Grasp>& possible_grasps);

  /**
   * \brief 围绕一个姿势在一个轴上创建抓取位置
   *        Note: to visualize these grasps use moveit_visual_tools.publishAnimatedGrasps() function or
   *        moveit_visual_tools.publishIKSolutions() with the resulting data
   * \param pose - 被抓物体的中心点
   * \param axis - axis relative to object pose to rotate generated grasps around相对于对象姿态的轴旋转生成的抓取
   * \param direction - 平行抓取器通常是对称的，因此可以在180度左右执行相同的抓取。 此选项允许生成翻转的抓握姿势
   * \param rotation - amount to rotate around the object - 180 or 360 degrees围绕物体旋转的量-180或360度
   * \param hand_roll - 握住时相对于对象中心点滚动腕部的弧度（弧度）。 默认使用0
   * \param grasp_data - 特定于机器人几何形状的参数
   * \param possible_grasps - 尝试掌握的输出解向量。 好的，如果预先填充
   * \return true if successful
   */
  bool generateAxisGrasps(
    const geometry_msgs::Pose& object_pose,
    grasp_axis_t axis,
    grasp_direction_t direction,
    grasp_rotation_t rotation,
    double hand_roll,
    const GraspData& grasp_data,
    std::vector<moveit_msgs::Grasp>& possible_grasps);

  /**
   * \brief 使用输入的抓取描述，获得预抓姿势
   * \param grasp description
   * \param name of parent link
   * \return pregrasp pose
   */
  static geometry_msgs::PoseStamped getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link);

  /**
   * \brief Print debug info
   * DEPRECATRED: moved to grasp_data.cpp
   */
  MOVEIT_DEPRECATED static void printObjectGraspData(const GraspData& data)
  {
    ROS_INFO_STREAM_NAMED("grasp","ROBOT GRASP DATA DEBUG OUTPUT ---------------------");
    ROS_INFO_STREAM_NAMED("grasp","Base Link: " << data.base_link_);
    ROS_INFO_STREAM_NAMED("grasp","EE Parent Link: " << data.ee_parent_link_);
    ROS_INFO_STREAM_NAMED("grasp","Grasp Depth: " << data.grasp_depth_);
    ROS_INFO_STREAM_NAMED("grasp","Angle Resolution: " << data.angle_resolution_);
    ROS_INFO_STREAM_NAMED("grasp","Approach Retreat Desired Dist: " << data.approach_retreat_desired_dist_);
    ROS_INFO_STREAM_NAMED("grasp","Approach Retreat Min Dist: " << data.approach_retreat_min_dist_);
    ROS_INFO_STREAM_NAMED("grasp","Pregrasp Posture: \n" << data.pre_grasp_posture_);
    ROS_INFO_STREAM_NAMED("grasp","Grasp Posture: \n" << data.grasp_posture_);
    ROS_INFO_STREAM_NAMED("grasp","---------------------------------------------------\n");
  }

}; // end of class

typedef boost::shared_ptr<SimpleGrasps> SimpleGraspsPtr;
typedef boost::shared_ptr<const SimpleGrasps> SimpleGraspsConstPtr;

} // namespace

#endif
