

#include <moveit_simple_grasps/simple_grasps.h>

namespace moveit_simple_grasps
{

// Constructor
SimpleGrasps::SimpleGrasps(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, bool verbose) :
  visual_tools_(visual_tools),
  verbose_(verbose)
{
  ROS_DEBUG_STREAM_NAMED("grasps","Loaded simple grasp generator");
}

// Deconstructor
SimpleGrasps::~SimpleGrasps()
{
}
/**
   * \brief 为块创建所有可能的抓取位置
   * \param pose of block, where vector arrow is parallel to table plane块的姿势，矢量箭头与工作台平面平行
   * \param data 描述末端执行器
   * \param resulting generated possible grasps
   * \return true if successful
   */ 
// Create all possible grasp positions for a object创建对象的所有可能的抓取位置
bool SimpleGrasps::generateBlockGrasps(const geometry_msgs::Pose& object_pose, const GraspData& grasp_data,
  std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // ---------------------------------------------------------------------------------------------
  // 计算两个方向上两个轴的抓取
  generateAxisGrasps( object_pose, X_AXIS, DOWN, HALF, 0, grasp_data, possible_grasps); // got no grasps with this alone
  generateAxisGrasps( object_pose, X_AXIS, UP,   HALF, 0, grasp_data, possible_grasps); // gives some grasps... looks ugly
  generateAxisGrasps( object_pose, Y_AXIS, DOWN, HALF, 0, grasp_data, possible_grasps); // GOOD ONES!
  generateAxisGrasps( object_pose, Y_AXIS, UP,   HALF, 0, grasp_data, possible_grasps); // gave a grasp from top... bad

  return true;
}

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
// 在一个轴上创建抓取位置
bool SimpleGrasps::generateAxisGrasps(
  const geometry_msgs::Pose& object_pose,//被抓物体的中心点
  grasp_axis_t axis,//相对于对象姿态的轴旋转生成的抓取
  grasp_direction_t direction,//平行抓取器通常是对称的，因此可以在180度左右执行相同的抓取。 此选项允许生成翻转的抓握姿势
  grasp_rotation_t rotation,//围绕物体旋转的量-180或360度
  double hand_roll,//握住时相对于对象中心点滚动腕部的弧度（弧度）。 默认使用0
  const GraspData& grasp_data,//特定于机器人几何形状的参数
  std::vector<moveit_msgs::Grasp>& possible_grasps)//尝试掌握的输出解向量。 好的，如果预先填充
{
  // ---------------------------------------------------------------------------------------------
  // 创建从对象框架（对象中心）到/ base_link的转换
  tf::poseMsgToEigen(object_pose, object_global_transform_);

  // ---------------------------------------------------------------------------------------------
  // 抓握参数

  // 创建可重复使用的逼近运动
  moveit_msgs::GripperTranslation pre_grasp_approach;
  pre_grasp_approach.direction.header.stamp = ros::Time::now();
  pre_grasp_approach.desired_distance = grasp_data.approach_retreat_desired_dist_; // 
  pre_grasp_approach.min_distance = grasp_data.approach_retreat_min_dist_; // half of the desired? Untested.

  // Create re-usable retreat motion创建可重复使用的撤退动作
  moveit_msgs::GripperTranslation post_grasp_retreat;
  post_grasp_retreat.direction.header.stamp = ros::Time::now();
  post_grasp_retreat.desired_distance = grasp_data.approach_retreat_desired_dist_; // 机器人链接原点需要行进的距离
  post_grasp_retreat.min_distance = grasp_data.approach_retreat_min_dist_; // half of the desired? Untested.

  // 创建可重复使用的空白姿势
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // ---------------------------------------------------------------------------------------------
  // Angle calculations
  double radius = grasp_data.grasp_depth_; //0.12
  double xb;
  double yb = 0.0; // 停留在物体的y平面
  double zb;
  double theta1 = 0.0; // 点在对象周围的位置
  double theta2 = 0.0; // UP 'direction'

  // 夹爪方向（上/下）旋转。 UP默认设置
  if( direction == DOWN )
  {
    theta2 = M_PI;
  }

  // ---------------------------------------------------------------------------------------------
  // ---------------------------------------------------------------------------------------------
  // Begin Grasp Generator Loop开始掌握生成器循环
  // ---------------------------------------------------------------------------------------------
  // ---------------------------------------------------------------------------------------------

  /* Developer Note:
   * 以给定的分辨率围绕所选轴创建180度角
    *我们在对象的参考框架中创建抓取，然后将其转换为基本链接
   */
  for(int i = 0; i <= grasp_data.angle_resolution_; ++i)
  {
    // 创建一条抓握消息
    moveit_msgs::Grasp new_grasp;

    // 计算抓握姿势
    xb = radius*cos(theta1);
    zb = radius*sin(theta1);

    Eigen::Affine3d grasp_pose;

    switch(axis)
    {
      case X_AXIS:
        grasp_pose = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

        grasp_pose.translation() = Eigen::Vector3d( yb, xb ,zb);

        break;
      case Y_AXIS:
        grasp_pose =
          Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitY())
          *Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

        grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

        break;
      case Z_AXIS:
        grasp_pose =
                Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitZ())
                *Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

            grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

        break;
    }

    /* 这种把握的成功估计概率，或某种“良好”程度的其他度量。 
    *在此，我们根据手腕距表面的距离来对分数进行偏倚，最好选择更大的距离以防止手腕/末端执行器与桌子碰撞
     */
    double score = sin(theta1);
    new_grasp.grasp_quality = std::max(score,0.1); // 不允许分数降到0.1 b / c以下，所有把握都可以

    // 下次计算theta1
    if (rotation == HALF)
      theta1 += M_PI / grasp_data.angle_resolution_;
    else
    {
      theta1 += 2*M_PI / grasp_data.angle_resolution_;
    }

    // 抓握的名字
    static int grasp_id = 0;
    new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
    ++grasp_id;

    // PreGrasp和Grasp姿势 --------------------------------------------------------------------------

    // 仅用于抓握前的手的内部姿势
    new_grasp.pre_grasp_posture = grasp_data.pre_grasp_posture_;

    // 使用手的内部姿势进行抓握位置和力度
    new_grasp.grasp_posture = grasp_data.grasp_posture_;

    // Grasp ------------------------------------------------------------------------------------------------


    // 调试-在转换为抓取器框架之前显示原始抓取姿势
    if( verbose_ )
    {
      tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
      visual_tools_->publishArrow(grasp_pose_msg.pose, rviz_visual_tools::GREEN);
    }

    // ------------------------------------------------------------------------
    // 可选择相对于物体姿势滚动手腕
    Eigen::Affine3d roll_gripper;
    roll_gripper = Eigen::AngleAxisd(hand_roll, Eigen::Vector3d::UnitX());
    grasp_pose = grasp_pose * roll_gripper;

    // ------------------------------------------------------------------------
    // 更改对此定制末端执行器参考框架的抓握

    // Convert to Eigen
    Eigen::Affine3d eef_conversion_pose;
    tf::poseMsgToEigen(grasp_data.grasp_pose_to_eef_pose_, eef_conversion_pose);

    // 改变抓握姿势
    grasp_pose = grasp_pose * eef_conversion_pose;

    // ------------------------------------------------------------------------
    // 将姿势转换为全局框架（base_link）
    tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);

    // 抓持器的末端执行器相对于参考系的位置（始终在其他位置指定，而不是在此消息中指定）
    new_grasp.grasp_pose = grasp_pose_msg;

    // Other ------------------------------------------------------------------------------------------------

    // 抓握时要使用的最大接触力（<= 0以禁用）
    new_grasp.max_contact_force = 0;

    // -------------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------
    // 进退
    // -------------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------

    // Straight down ---------------------------------------------------------------------------------------
    //关于基本链接/世界框架

    // Approach进
    pre_grasp_approach.direction.header.frame_id = grasp_data.base_link_;
    pre_grasp_approach.direction.vector.x = 0;
    pre_grasp_approach.direction.vector.y = 0;
    pre_grasp_approach.direction.vector.z = -1; // 接近方向（负z轴） // TODO: 记录这个假设
    new_grasp.pre_grasp_approach = pre_grasp_approach;

    // Retreat退
    post_grasp_retreat.direction.header.frame_id = grasp_data.base_link_;
    post_grasp_retreat.direction.vector.x = 0;
    post_grasp_retreat.direction.vector.y = 0;
    post_grasp_retreat.direction.vector.z = 1; // 退避方向（位置z轴）
    new_grasp.post_grasp_retreat = post_grasp_retreat;

    // Add to vector添加到向量
    possible_grasps.push_back(new_grasp);

    // Angled with pose 与姿势成角度-------------------------------------------------------------------------------------
    // Approach with respect to end effector orientation有关末端执行器方向的方法

    // Approach
    pre_grasp_approach.direction.header.frame_id = grasp_data.ee_parent_link_;
    pre_grasp_approach.direction.vector.x = 0;
    pre_grasp_approach.direction.vector.y = 0;
    pre_grasp_approach.direction.vector.z = 1;
    new_grasp.pre_grasp_approach = pre_grasp_approach;

    // Retreat
    post_grasp_retreat.direction.header.frame_id = grasp_data.ee_parent_link_;
    post_grasp_retreat.direction.vector.x = 0;
    post_grasp_retreat.direction.vector.y = 0;
    post_grasp_retreat.direction.vector.z = -1;
    new_grasp.post_grasp_retreat = post_grasp_retreat;

    // Add to vector
    possible_grasps.push_back(new_grasp);

  }
/*
*为调试信息命名
*ROS_INFO_STREAM_NAMED( "named_msg", "INFO named message." );
*表示为这段信息命名，为了更容易知道这段信息来自那段代码．


*/
  ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );

  return true;
}
 /**
   * \brief 使用输入的抓取描述，获得预抓姿势
   * \param grasp description
   * \param name of parent link
   * \return pregrasp pose
   */
geometry_msgs::PoseStamped SimpleGrasps::getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link)
{
  // 抓握姿势变量
  geometry_msgs::PoseStamped grasp_pose = grasp.grasp_pose;
  Eigen::Affine3d grasp_pose_eigen;
  tf::poseMsgToEigen(grasp_pose.pose, grasp_pose_eigen);

  // 首先获取抓握前的姿势
  geometry_msgs::PoseStamped pre_grasp_pose;
  Eigen::Affine3d pre_grasp_pose_eigen = grasp_pose_eigen; // 将原始抓握姿势复制到预抓握姿势

  // 接近方向变量
  Eigen::Vector3d pre_grasp_approach_direction_local;

  // 预抓的方向
   //根据百分比计算当前动画位置
  Eigen::Vector3d pre_grasp_approach_direction = Eigen::Vector3d(
    -1 * grasp.pre_grasp_approach.direction.vector.x * grasp.pre_grasp_approach.desired_distance,
    -1 * grasp.pre_grasp_approach.direction.vector.y * grasp.pre_grasp_approach.desired_distance,
    -1 * grasp.pre_grasp_approach.direction.vector.z * grasp.pre_grasp_approach.desired_distance
  );

  // 决定是否需要将approach_direction更改为末端执行器方向的局部框架
  if( grasp.pre_grasp_approach.direction.header.frame_id == ee_parent_link )
  {
    // 在抓握姿态姿势的局部帧中应用/计算方法向量
    pre_grasp_approach_direction_local = grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
  }
  else
  {
    pre_grasp_approach_direction_local = pre_grasp_approach_direction; //grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
  }

  // 使用新的本地框的方式更新方向的把握矩阵
  pre_grasp_pose_eigen.translation() += pre_grasp_approach_direction_local;

  // 转换征抓前位置回到正常的消息
  tf::poseEigenToMsg(pre_grasp_pose_eigen, pre_grasp_pose.pose);

  //将原始标题复制到新的掌握
  pre_grasp_pose.header = grasp_pose.header;

  return pre_grasp_pose;
}



} // namespace
