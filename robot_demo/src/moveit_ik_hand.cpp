#include<string>
#include<ros/ros.h>
#include<moveit/move_group_interface/move_group_interface.h>
int main(int argc,char** argv)
{
    ros::init(argc,argv,"moveit_ik_hand");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    
    std::string end_effector_link=arm.getEndEffectorLink();
    
    std::string reference_frame="base_link";
    arm.setPoseReferenceFrame(reference_frame);
    
    arm.allowReplanning(true);
    
    sleep(1);
    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("close");
    
    gripper.move();
    ROS_INFO("---test1---ok----");
    
    sleep(1);
    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("open");
    
    gripper.move();
    ROS_INFO("---test2---ok----");
    
    
    return 0;
}
