#! /usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import rospkg
from pick_and_place import pick_and_place

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
def main():
    p = Pick_Place()

    rospy.spin()
    rospy.sleep(1)



if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place')

    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('ur5_single_arm_tufts')

    """
    rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/objects/table.urdf -urdf -x 0.85 -y 0.0 -z 0.73 -model my_object
    rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/objects/block.urdf -urdf -x 0.5 -y -0.0 -z 0.77 -model block
    """

    
    main()

    roscpp_shutdown()
    rospy.sleep(1)
