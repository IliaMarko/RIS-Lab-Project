#!/usr/bin/env python
# Copyright (c) 2016-2019 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import print_function
import numpy
import rospy
import tf
import tf.transformations as trans
from os.path import isdir, join
from copy import deepcopy
import yaml
import tf2_ros
from geometry_msgs.msg import Wrench, WrenchStamped
from myrov_control.srv import *

        
def input_callback(msg):
     #print("Callback function call sucsses")
     rospy.wait_for_service('cmd_thruster')
     try:
         #print("I'm nsede service try")
         force = numpy.array((msg.force.x, msg.force.y, msg.force.z))
         torque = numpy.array((msg.torque.x, msg.torque.y, msg.torque.z))
         
         cmd_thruster = rospy.ServiceProxy('cmd_thruster', CmdThruster)
         resp1 = cmd_thruster(msg.force,msg.torque)
         #print("Service call sucsses")
         return True
         
     except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
def wernch_cmd():
     rospy.init_node('listener', anonymous=True) 
     print("Erench convrtion to thrust commonds node is starting")
     rospy.Subscriber("/thruster_manager/input", Wrench, input_callback)
     rospy.spin()
         
if __name__ == '__main__':
    try:
         wernch_cmd()
    except rospy.ROSInterruptException:
         pass
