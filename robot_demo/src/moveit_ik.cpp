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
    int i=0;
    while(i<5){
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);
    
    geometry_msgs::Pose target_pose[4];
    target_pose[0].orientation.x=0;
    target_pose[0].orientation.y=0.70729;
    target_pose[0].orientation.z=0.70729;
    target_pose[0].orientation.w=0;
    
    target_pose[0].position.x=0.0611961;
    target_pose[0].position.y=-0.319739;
    target_pose[0].position.z=0.784734;
    
    target_pose[1].orientation.x=0;
    target_pose[1].orientation.y=0.70729;
    target_pose[1].orientation.z=0.70729;
    target_pose[1].orientation.w=0;
    
    target_pose[1].position.x=-0.190107;
    target_pose[1].position.y=0.572039;
    target_pose[1].position.z=0.45035;
    
    target_pose[2].orientation.x=0;
    target_pose[2].orientation.y=0.70729;
    target_pose[2].orientation.z=0.70729;
    target_pose[2].orientation.w=0;
    
    target_pose[2].position.x=-0.47159;
    target_pose[2].position.y=0.0131242;
    target_pose[2].position.z=0.48774;
    
    
    target_pose[3].orientation.x=0;
    target_pose[3].orientation.y=0.70729;
    target_pose[3].orientation.z=0.70729;
    target_pose[3].orientation.w=0;
    
    target_pose[3].position.x=-0.817237;
    target_pose[3].position.y=0.191501;
    target_pose[3].position.z=-0.00543038;
    for(int j=0;j<4;j++)
    {
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose[j]);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success=arm.plan(plan);
    //std::cout<<"i=: "<<i<<std::endl;
    ROS_INFO("Plan (pose goal)%d--%d--- %s",i,j,success?" ":"FAILED");
    
    if(success)
        arm.execute(plan);
        sleep(0.5);
    //std::cout<<"j=: "<<j<<std::endl;
    }
    sleep(1);
    i++;}
    ros::shutdown();

    return 0;
}
