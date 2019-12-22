#include<ros/ros.h>
#include<string>
#include<moveit/move_group_interface/move_group_interface.h>
int main(int argc,char** argv)
{
    ros::init(argc,argv,"moveit_ik_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    
    std::string end_effector_link=arm.getEndEffectorLink();
    std::string reference_frame="base_link";
    arm.setPoseReferenceFrame(reference_frame);
    
    arm.allowReplanning(true);
    
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);
    
    arm.setMaxAccelerationScalingFactor(1.5);
    arm.setMaxVelocityScalingFactor(1.5);
    int i=1;
    while(i){
    arm.setNamedTarget("up");
    arm.move();
    sleep(1);
    
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x=0;
    target_pose.orientation.y=0.0;
    target_pose.orientation.z=0.0;
    target_pose.orientation.w=1;
    
    target_pose.position.x=-0.0145692;
    target_pose.position.y=0.356811;
    target_pose.position.z=0.876126;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);
        
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success=arm.plan(plan);
    //std::cout<<"i=: "<<i<<std::endl;
    //ROS_INFO("Plan (pose goal)%d--%d--- %s",i,j,success?" ":"FAILED");
    
    if(success)
        arm.execute(plan);
        sleep(1);
    //std::cout<<"j=: "<<j<<std::endl;
   geometry_msgs::Pose target_pose2;
    target_pose2.orientation.x=-0.00013699;
    target_pose2.orientation.y=-0.000119428;
    target_pose2.orientation.z=-0.283954;
    target_pose2.orientation.w=0.958838;
    
    target_pose2.position.x=0.318786;
    target_pose2.position.y=-0.0120576;
    target_pose2.position.z=0.736996;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose2);
        
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    moveit::planning_interface::MoveItErrorCode success1=arm.plan(plan1);
    //std::cout<<"i=: "<<i<<std::endl;
    //ROS_INFO("Plan (pose goal)%d--%d--- %s",i,j,success?" ":"FAILED");
    
    if(success)
        arm.execute(plan1);
        sleep(1);
    //sleep(1);
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);}
    ros::shutdown();

    return 0;
}
