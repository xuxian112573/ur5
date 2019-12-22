#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import rospkg

from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown

from actionlib import SimpleActionClient, GoalStatus

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from moveit_msgs.msg import PickupAction, PickupGoal
from moveit_msgs.msg import PlaceAction, PlaceGoal
from moveit_msgs.msg import PlaceLocation
from moveit_msgs.msg import MoveItErrorCodes
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GraspGeneratorOptions

from tf.transformations import quaternion_from_euler

import sys
import copy
import numpy
import os

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


# Create dict with human readable MoveIt! error codes:用易读的MoveIt创建字典！ 错误代码：
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class Pick_Place:
    def __init__(self,x,y,z):
        self._x=x
        self._y=y
        self._z=z
        
        # Retrieve params:
        self._table_object_name = rospy.get_param('~table_object_name', 'Grasp_Table')
        self._grasp_object_name = rospy.get_param('~grasp_object_name', 'Grasp_Object')

        self._grasp_object_width = rospy.get_param('~grasp_object_width', 0.01)

        self._arm_group     = rospy.get_param('~manipulator', 'manipulator')
        self._gripper_group = rospy.get_param('~gripper', 'gripper')

        self._approach_retreat_desired_dist = rospy.get_param('~approach_retreat_desired_dist', 0.15)
        self._approach_retreat_min_dist = rospy.get_param('~approach_retreat_min_dist', 0.1)

        # Create (debugging) publishers:创建（调试）发布者：
        self._grasps_pub = rospy.Publisher('grasps', PoseArray, queue_size=1, latch=True)
        self._places_pub = rospy.Publisher('places', PoseArray, queue_size=1, latch=True)

        # Create planning scene and robot commander:创建规划场景和机器人命令：
        self._scene = PlanningSceneInterface()#未知
        self._robot = RobotCommander()#未知
        self._pose_coke_can = self._add_grasp_block_(self._grasp_object_name,self._x,self._y,self._z)
        rospy.sleep(0.5)

      

        # Define target place pose:定义目标位置姿势
        self._pose_place = Pose()

        self._pose_place.position.x =-0.75 #self._pose_coke_can.position.x
        self._pose_place.position.y =0.0 #self._pose_coke_can.position.y - 0.20
        self._pose_place.position.z =self._z #self._pose_coke_can.position.z
        self._pose_place.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))

        self._pose_coke_can.position.z = self._pose_coke_can.position.z-0.9#self._pose_coke_can.position.z - 0.9 # base_link is 0.9 above
        self._pose_place.position.z = self._pose_place.position.z + 0.05 # target pose a little above

        # Retrieve groups (arm and gripper):
        self._arm     = self._robot.get_group(self._arm_group)
        self._gripper = self._robot.get_group(self._gripper_group)

        self._arm.set_named_target('up')
        self._arm.go(wait=True)
        print("up pose")
        print("第一部分恢复位置初始")

        # Create grasp generator 'generate' action client:
        # #创建抓取生成器“生成”动作客户端：
        print("开始Action通信")
        self._grasps_ac = SimpleActionClient('/moveit_simple_grasps_server/generate', GenerateGraspsAction)
        if not self._grasps_ac.wait_for_server(rospy.Duration(0.5)):
            rospy.logerr('Grasp generator action client not available!')
            rospy.signal_shutdown('Grasp generator action client not available!')
            return
        print("结束Action通信")
        # Create move group 'pickup' action client:
        # 创建移动组“抓取”操作客户端：
        print("开始pickup通信")
        self._pickup_ac = SimpleActionClient('/pickup', PickupAction)
        if not self._pickup_ac.wait_for_server(rospy.Duration(0.5)):
            rospy.logerr('Pick up action client not available!')
            rospy.signal_shutdown('Pick up action client not available!')
            return
        print("结束pickup通信")
        # Create move group 'place' action client:
        # 创建移动组“放置”动作客户端：
        print("开始place通信")
        self._place_ac = SimpleActionClient('/place', PlaceAction)
        if not self._place_ac.wait_for_server(rospy.Duration(0.5)):
            rospy.logerr('Place action client not available!')
            rospy.signal_shutdown('Place action client not available!')
            return
        print("结束place通信")
        # Pick Coke can object:抓取快
        while not self._pickup(self._arm_group, self._grasp_object_name, self._grasp_object_width):
            rospy.logwarn('Pick up failed! Retrying ...')
            rospy.sleep(0.5)
        print("抓取物体")
        rospy.loginfo('Pick up successfully')
        print("pose_place: ", self._pose_place)

        # Place Coke can object on another place on the support surface (table):
        while not self._place(self._arm_group, self._grasp_object_name, self._pose_place):
            rospy.logwarn('Place failed! Retrying ...')
            rospy.sleep(0.5)

        rospy.loginfo('Place successfully')

        
    
    
    def _generate_grasps(self, pose, width):
        """
        使用抓握生成器生成动作来生成抓握； 基于moveit_simple_grasps pkg上的server_test.py示例。

        生成抓取姿势阵列数据以进行可视化，
         然后将抓取目标发送到抓取服务器.
        """
        self._grasps_ac.wait_for_server()
        rospy.loginfo("Successfully connected!")
        # Create goal:
        goal = GenerateGraspsGoal()

        goal.pose  = pose
        goal.width = width
        
        options = GraspGeneratorOptions()
        # simple_graps.cpp doesn't implement GRASP_AXIS_Z!
        #options.grasp_axis      = GraspGeneratorOptions.GRASP_AXIS_Z
        options.grasp_direction = GraspGeneratorOptions.GRASP_DIRECTION_UP
        options.grasp_rotation  = GraspGeneratorOptions.GRASP_ROTATION_FULL

        # @todo disabled because it works better with the default options
        #goal.options.append(options)

        # Send goal and wait for result:
        # Sends a goal to the ActionServer, waits for the goal to complete, and preempts goal is necessary.
        #
        rospy.loginfo("Sent goal,waiting,目标物体的位置:\n "+str(goal))
        
        #self._grasps_ac.wait_for_result()
        
        #发送目标并等待结果：
        # ＃将目标发送到ActionServer，等待目标完成，然后抢占目标是必要的。
        t_start=rospy.Time.now()
        state = self._grasps_ac.send_goal_and_wait(goal)
        t_end=rospy.Time.now()
        t_toal=t_end-t_start
        rospy.loginfo("发送时间Result received in "+str(t_toal.to_sec()))
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Grasp goal failed!: %s' % self._grasps_ac.get_goal_status_text())
            return None

        grasps = self._grasps_ac.get_result().grasps

        # Publish grasps (for debugging/visualization purposes):
        self._publish_grasps(grasps)
        """
        print("---------grasps--start------------")
        print(grasps)
        
        print("-----------test-grasps-----end-----------------")
        
        
        rospy.sleep(2)
        print("-grasps---sleep----end-----")
        """
        return grasps

    def _generate_places(self, target):
        """
        Generate places (place locations), based on
        https://github.com/davetcoleman/baxter_cpp/blob/hydro-devel/
        baxter_pick_place/src/block_pick_place.cpp

        为该位置的姿势创建姿势数组数据
        """

        # Generate places:
        places = []
        now = rospy.Time.now()
        #print("---------------test3--------------------------")
        for angle in numpy.arange(0.0, numpy.deg2rad(360.0), numpy.deg2rad(1.0)):
            # Create place location:
            place = PlaceLocation()

            place.place_pose.header.stamp = now
            place.place_pose.header.frame_id = self._robot.get_planning_frame()

            # Set target position:
            place.place_pose.pose = copy.deepcopy(target)

            # Generate orientation (wrt Z axis):
            q = quaternion_from_euler(0.0, 0.0, angle )
            place.place_pose.pose.orientation = Quaternion(*q)

            # Generate pre place approach:生成前置位置方法：
            place.pre_place_approach.desired_distance = self._approach_retreat_desired_dist
            place.pre_place_approach.min_distance = self._approach_retreat_min_dist

            place.pre_place_approach.direction.header.stamp = now
            place.pre_place_approach.direction.header.frame_id = self._robot.get_planning_frame()

            place.pre_place_approach.direction.vector.x =  0
            place.pre_place_approach.direction.vector.y =  0
            place.pre_place_approach.direction.vector.z = 0.1
            #print("place1---test=====")
            # Generate post place approach:
            place.post_place_retreat.direction.header.stamp = now
            place.post_place_retreat.direction.header.frame_id = self._robot.get_planning_frame()

            place.post_place_retreat.desired_distance = self._approach_retreat_desired_dist
            place.post_place_retreat.min_distance = self._approach_retreat_min_dist

            place.post_place_retreat.direction.vector.x = 0
            place.post_place_retreat.direction.vector.y = 0
            place.post_place_retreat.direction.vector.z = 0.06
            #print("place2---test=====")
            # Add place:
            places.append(place)

        # Publish places (for debugging/visualization purposes):
        self._publish_places(places)
        """
        print(places)
        print("------test4---------------------------------------")
        """
        return places

    def _create_pickup_goal(self, group, target, grasps):
        """
        Create a MoveIt! PickupGoal
        Create a goal for picking up the grasping object创建一个捡起抓取物体的目标
        """

        # Create goal:
        goal = PickupGoal()

        goal.group_name  = group #规划的组
        goal.target_name = target #要拾取的对象的名称（
#可能使用的抓握列表。 必须至少填写一个掌握
        goal.possible_grasps.extend(grasps)
#障碍物的可选清单，我们掌握有关语义的信息，可以在抓取过程中触摸/推动/移动这些障碍物； 注意：如果使用对象名称“ all”，则在进近和提升过程中禁止与所有对象发生碰撞。
        goal.allowed_touch_objects.append(target) 
#如果没有可用的名称，则在碰撞图中支撑表面（例如桌子）的名称可以留空
        goal.support_surface_name = self._table_object_name

        # Configure goal planning options:配置目标计划选项：
        #运动计划者可以规划的最长时间
        goal.allowed_planning_time = 7.0

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 20
        """
        print("----goal-------test-----")
        print(goal)
        print("----goal---end------test---")
        """
        rospy.sleep(0.5)
        print("-goal---sleep----end-----")
        return goal

    def _create_place_goal(self, group, target, places):
        """
        Create a MoveIt! PlaceGoal
        Create a place goal for MoveIt!
        """

        # Create goal:
        goal = PlaceGoal()

        goal.group_name           = group
        goal.attached_object_name = target

        goal.place_locations.extend(places)

        # Configure goal planning options:
        goal.allowed_planning_time = 7.0

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 20

        return goal
    
    def _add_grasp_block_(self, name,_x,_y,_z):
        """
        Create and add block to the scene创建场景并将其添加到场景
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = _x
        p.pose.position.y = _y
        p.pose.position.z = _z

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = Quaternion(*q)

        # Coke can size from ~/.gazebo/models/coke_can/meshes/coke_can.dae,
        # using the measure tape tool from meshlab.
        # The box is the bounding box of the coke cylinder.
        # The values are taken from the cylinder base diameter and height.
        #self._scene.add_box(name, p, (0.045, 0.045, 0.045))

        return p.pose
    
    def _pickup(self, group, target, width):
        """
        Pick up a target using the planning group使用规划小组选择目标
        """

        # Obtain possible grasps from the grasp generator server:从掌握生成器服务器获取可能的抓握：
        grasps = self._generate_grasps(self._pose_coke_can, width)
        #print("--goal--start----")
        # Create and send Pickup goal:创建并发送zhua取目标：
        goal = self._create_pickup_goal(group, target, grasps)
        #print("--goal--end----")
        #print("---pick---up---1----")
        state = self._pickup_ac.send_goal_and_wait(goal)
        #print("-----------------state-------------------")
        #print(state)
        #print("---pick---up---2----")        
        
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Pick up goal failed!: %s' % self._pickup_ac.get_goal_status_text())
            return None

        result = self._pickup_ac.get_result()
        """
        print("-----test1--------")
        print(result)
        print("------test1--end------")
        """
        # Check for error:
        err = result.error_code.val
        print(err)
        if err != MoveItErrorCodes.SUCCESS:
            rospy.logwarn('Group %s cannot pick up target %s!: %s' % (group, target, str(moveit_error_dict[err])))

            return False
        print("pickup-----end-----")
        rospy.sleep(0.5)
        return True

    def _place(self, group, target, place):
        """
        Place a target using the planning group
        """

        # Obtain possible places:获取可能的位置：
        places = self._generate_places(place)

        # Create and send Place goal:创建并发送地方目标：
        goal = self._create_place_goal(group, target, places)

        state = self._place_ac.send_goal_and_wait(goal)
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Place goal failed!: %s' % self._place_ac.get_goal_status_text())
            return None

        result = self._place_ac.get_result()
        """
        print("-----test2--------")
        print(result)
        print("------test2--end------")
        """
        # Check for error:
        err = result.error_code.val
        if err != MoveItErrorCodes.SUCCESS:
            rospy.logwarn('Group %s cannot place target %s!: %s' % (group, target, str(moveit_error_dict[err])))

            return False

        return True

    def _publish_grasps(self, grasps):
        """
        Publish grasps as poses, using a PoseArray message使用PoseArray消息将抓取发布为姿势
        """
        print(self._grasps_pub.get_num_connections())
        if self._grasps_pub.get_num_connections() != 1:
            
            msg = PoseArray()
            msg.header.frame_id = self._robot.get_planning_frame()
            msg.header.stamp = rospy.Time.now()

            for grasp in grasps:
                p = grasp.grasp_pose.pose

                msg.poses.append(Pose(p.position, p.orientation))
                print(msg)
                rospy.loginfo('Publisher '+str(len(msg.poses))+' poses')
            
            self._grasps_pub.publish(msg)
            """
            print("11-111")
            rospy.loginfo('Publisher'+str(len(msg))+'poses')
            print("11-111")            
            rospy.sleep(2)
            """

    def _publish_places(self, places):
        """
        Publish places as poses, using a PoseArray message
        """

        if self._places_pub.get_num_connections() > 0:
            msg = PoseArray()
            msg.header.frame_id = self._robot.get_planning_frame()
            msg.header.stamp = rospy.Time.now()

            for place in places:
                msg.poses.append(place.place_pose.pose)

            self._places_pub.publish(msg)



    
