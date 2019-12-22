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


class add_box:
    def __init__(self):
        
        
        # Retrieve params:

        self._grasp_object_name = rospy.get_param('~grasp_object_name', 'Grasp_Object')
        self._grasp_object2_name = rospy.get_param('~grasp_object_name', 'Grasp_Object-2')

        self._grasp_object_width = rospy.get_param('~grasp_object_width', 0.01)

        self._arm_group     = rospy.get_param('~manipulator', 'manipulator')
        self._gripper_group = rospy.get_param('~gripper', 'gripper')

        self._approach_retreat_desired_dist = rospy.get_param('~approach_retreat_desired_dist', 0.2)
        self._approach_retreat_min_dist = rospy.get_param('~approach_retreat_min_dist', 0.1)

        # Create (debugging) publishers:创建（调试）发布者：
        self._grasps_pub = rospy.Publisher('grasps', PoseArray, queue_size=1, latch=True)
        self._places_pub = rospy.Publisher('places', PoseArray, queue_size=1, latch=True)

        # Create planning scene and robot commander:创建规划场景和机器人命令：
        self._scene = PlanningSceneInterface()#未知
        self._robot = RobotCommander()#未知

        rospy.sleep(0.5)

        # Clean the scene:清理现场：
        
        self._scene.remove_world_object(self._grasp_object_name)
        self._scene.remove_world_object(self._grasp_object2_name)
        


        # Add table and Coke can objects to the planning scene:将桌子和可乐罐对象添加到计划场景：

        self._pose_coke_can = self._add_grasp_block_(self._grasp_object_name)#增加障碍物块
        self._pose_coke_can = self._add_grasp_fanfkuai_(self._grasp_object2_name)#增加障碍物块

        rospy.sleep(0.5)

        
    def __del__(self):
        # Clean the scene:
        self._scene.remove_world_object(self._grasp_object_name)
        self._scene.remove_world_object(self._grasp_object2_name)
        
        

    
#----------------------------------1hang---------------------------------
    def _add_grasp_block_(self, name):
        """
        Create and add block to the scene创建场景并将其添加到场景
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
           
        p.pose.position.x = 0.6
        p.pose.position.y = -0.0
        p.pose.position.z = 0.75

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = Quaternion(*q)

        # Coke can size from ~/.gazebo/models/coke_can/meshes/coke_can.dae,
        # using the measure tape tool from meshlab.
        # The box is the bounding box of the coke cylinder.
        # The values are taken from the cylinder base diameter and height.
        self._scene.add_box(name, p, (0.045, 0.045, 0.045))
        
        return p.pose
        
    def _add_grasp_fanfkuai_(self, name):
        """
        Create and add block to the scene创建场景并将其添加到场景
        """
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
           
        p.pose.position.x = 0.6
        p.pose.position.y = 0.4
        p.pose.position.z = 0.75

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = Quaternion(*q)

        # Coke can size from ~/.gazebo/models/coke_can/meshes/coke_can.dae,
        # using the measure tape tool from meshlab.
        # The box is the bounding box of the coke cylinder.
        # The values are taken from the cylinder base diameter and height.
        self._scene.add_box(name, p, (0.045, 0.045, 0.045))
        
        return p.pose



def spawn_gazebo_model(model_path, model_name, model_pose, reference_frame="world"):
  """
  Spawn model in gazebo
  """
  model_xml = ''
  with open(model_path, "r") as model_file:
    model_xml = model_file.read().replace('\n', '')
  rospy.wait_for_service('/gazebo/spawn_urdf_model')
  try:
    spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    resp_urdf = spawn_urdf(model_name, model_xml, "/", model_pose, reference_frame)
  except rospy.ServiceException, e:
    rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_model(models):
  """
  Delete model in gazebo
  """
  try:
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    for a_model in models:
      resp_delete = delete_model(a_model)
  except rospy.ServiceException, e:
    rospy.loginfo("Delete Model service call failed: {0}".format(e))

if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place')

    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('ur5_single_arm_tufts')

    """
    rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/objects/table.urdf -urdf -x 0.85 -y 0.0 -z 0.73 -model my_object
    rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/objects/block.urdf -urdf -x 0.5 -y -0.0 -z 0.77 -model block
    """

    ban_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'ban.urdf'
    ban_name = 'ban'
    ban_pose = Pose(position=Point(x=0, y=0.6, z=0.7105+0.03))
    
    fangkuai_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'block.urdf'
    fangkuai_name = 'fangkuai'
    fangkuai_pose = Pose(position=Point(x=0.6, y=-0.0, z=0.7105+0.03))
    gold_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'gold_block.urdf'
    gold_block_name = 'gold_block'
    gold_block_pose = Pose(position=Point(x=0.6, y=0.4, z=0.7105+0.03))
    delete_gazebo_model([fangkuai_name,gold_block_name,ban_name])
    
    spawn_gazebo_model(fangkuai_path, fangkuai_name, fangkuai_pose)
    spawn_gazebo_model(gold_block_path, gold_block_name, gold_block_pose)
    spawn_gazebo_model(ban_path, ban_name, ban_pose)
    """
    #-----------------------------1hang---------------------------------------
    fangkuai_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'block.urdf'
    fangkuai_name = 'fangkuai'
    fangkuai_pose = Pose(position=Point(x=0.8, y=-0.4, z=0.74+0.03))
    
    
    gold_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'gold_block.urdf'
    gold_block_name = 'gold_block'
    gold_block_pose = Pose(position=Point(x=0.8, y=-0.2, z=0.74+0.03))
    
    hezi_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'123.urdf'
    hezi_name = 'hezi'
    hezi_pose = Pose(position=Point(x=0.8, y=0.0, z=0.74+0.03))
    
    white_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'white_block.urdf'
    white_block_name = 'white_block'
    white_block_pose = Pose(position=Point(x=0.8, y=0.2, z=0.74+0.03))
    
    green_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'green_block.urdf'
    green_block_name = 'green_block'
    green_block_pose = Pose(position=Point(x=0.8, y=0.4, z=0.74+0.03))
    
    #---------------------------------2hang------------------------------------------
    orange_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'orange_block.urdf'
    orange_block_name = 'orange_block'
    orange_block_pose = Pose(position=Point(x=0.6, y=-0.4, z=0.74+0.03))
    
    indigo_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'indigo_block.urdf'
    indigo_block_name = 'indigo_block'
    indigo_block_pose = Pose(position=Point(x=0.6, y=-0.2, z=0.74+0.03))
    
    com_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'com_block.urdf'
    com_block_name = 'com_block'
    com_block_pose = Pose(position=Point(x=0.6, y=0.0, z=0.74+0.03))
    
    wood_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'wood_block.urdf'
    wood_block_name = 'wood_block'
    wood_block_pose = Pose(position=Point(x=0.6, y=0.2, z=0.74+0.03))
    
    bricks_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'bricks_block.urdf'
    bricks_block_name = 'bricks_block'
    bricks_block_pose = Pose(position=Point(x=0.6, y=0.4, z=0.74+0.03))
    
    #----------------------------------------------3hang------------------------------------------
    
    road_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'road_block.urdf'
    road_block_name = 'road_block'
    road_block_pose = Pose(position=Point(x=0.4, y=-0.4, z=0.74+0.03))
    
    trunk_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'trunk_block.urdf'
    trunk_block_name = 'trunk_block'
    trunk_block_pose = Pose(position=Point(x=0.4, y=-0.2, z=0.74+0.03))
    
    grass_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'grass_block.urdf'
    grass_block_name = 'grass_block'
    grass_block_pose = Pose(position=Point(x=0.4, y=0.0, z=0.74+0.03))
    
    darkgrey_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'darkgrey_block.urdf'
    darkgrey_block_name = 'darkgrey_block'
    darkgrey_block_pose = Pose(position=Point(x=0.4, y=0.2, z=0.74+0.03))
    
    purple_block_path = pack_path+os.sep+'urdf'+os.sep+'objects'+os.sep+'purple_block.urdf'
    purple_block_name = 'purple_block'
    purple_block_pose = Pose(position=Point(x=0.4, y=0.4, z=0.74+0.03))
    
    
    
    
    
    
    #--------------------------------------------------------------------------------------------------
    delete_gazebo_model([fangkuai_name,gold_block_name,hezi_name,white_block_name,green_block_name,orange_block_name,indigo_block_name,com_block_name,wood_block_name,bricks_block_name,road_block_name,trunk_block_name,grass_block_name,darkgrey_block_name,purple_block_name,table_name])
    spawn_gazebo_model(table_path, table_name, table_pose)
    spawn_gazebo_model(fangkuai_path, fangkuai_name, fangkuai_pose)
    spawn_gazebo_model(hezi_path, hezi_name, hezi_pose)
    spawn_gazebo_model(gold_block_path, gold_block_name, gold_block_pose)
    spawn_gazebo_model(white_block_path, white_block_name, white_block_pose)
    spawn_gazebo_model(green_block_path, green_block_name, green_block_pose)
    spawn_gazebo_model(orange_block_path, orange_block_name, orange_block_pose)
    spawn_gazebo_model(indigo_block_path, indigo_block_name, indigo_block_pose)
    spawn_gazebo_model(com_block_path, com_block_name, com_block_pose)
    spawn_gazebo_model(wood_block_path, wood_block_name, wood_block_pose)
    spawn_gazebo_model(bricks_block_path, bricks_block_name, bricks_block_pose)
    #-------------------------------------------3-------------------------------------
    spawn_gazebo_model(road_block_path, road_block_name, road_block_pose)
    spawn_gazebo_model(trunk_block_path, trunk_block_name, trunk_block_pose)
    spawn_gazebo_model(grass_block_path, grass_block_name, grass_block_pose)
    spawn_gazebo_model(darkgrey_block_path, darkgrey_block_name, darkgrey_block_pose)
    spawn_gazebo_model(purple_block_path, purple_block_name, purple_block_pose)
    #delete_gazebo_model([table_name, block1_name,block2_name,block3_name])
    """
    
    
    
    #spawn_gazebo_model(block4_path, block4_name, block4_pose)
    cs=add_box()
    
    
    
