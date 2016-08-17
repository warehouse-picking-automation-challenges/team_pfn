#!/usr/bin/env python

# Copyright 2016 Preferred Networks, Inc.
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

import sys
import rospy
import actionlib
import arm_control_wrapper
from geometry_msgs.msg import Twist, Vector3,Point
from item_locations import ItemLocations
import get_item
import numpy as np
from grab.vacuum import VacuumGrabbing
from grab.gripper import GripperGrabbing
from tote_stow_locator.random_location import RandomLocationStow
from shelf_stow_locator.always_a import AlwaysAStow
from shelf_stow_locator.random_location import RandomLocationStow as ShelfRandomLocationStow
from apc2016.srv import *
from apc2016.msg import *

rospy.wait_for_service('update_calibration')
s = rospy.ServiceProxy('update_calibration',CalibrationUpdate)
#response = s(upper_left=result.upper_left, upper_right=result.upper_right)
response = s(upper_left=Point(1274,55.5,0), upper_right=Point(1266,-821.,0))
