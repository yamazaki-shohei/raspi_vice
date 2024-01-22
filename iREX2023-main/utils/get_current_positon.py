#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import copy
from math import pi
from moveit_commander import MoveGroupCommander

if __name__ == '__main__':
  rospy.init_node('message', anonymous=True)
  group = MoveGroupCommander('arm')
  exec_vel = 0.01
  try:
    wpose = group.get_current_pose().pose
    print(wpose)
    wjoint = group.get_current_joint_values()
    print(wjoint)
  except rospy.ROSInterruptException: pass

