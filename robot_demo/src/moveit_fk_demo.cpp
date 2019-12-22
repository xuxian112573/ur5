#include<ros/ros.h>
#include<moveit/move_group_interface/move_group_interface.h>
int main(int argc,char** argv)
{
    ros::init(argc,argv,"moveit_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    
    arm.setGoalJointTolerance(0.001);
    
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);
    
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);/**/
    
    double targetPose[6]={0.391410,-0.676384,-0.376217,0.0,1.052834,0.454125};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0]=targetPose[0];
    joint_group_positions[1]=targetPose[1];
    joint_group_positions[2]=targetPose[2];
    joint_group_positions[3]=targetPose[3];
    joint_group_positions[4]=targetPose[4];
    joint_group_positions[5]=targetPose[5];
    
    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    sleep(1);
    
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);
    ros::shutdown();

    return 0;
}
