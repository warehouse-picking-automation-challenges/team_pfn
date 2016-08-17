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

import numpy
import rospy
from apc2016.srv import *


print 'init'
rospy.init_node("test_item_pose_est")

print 'wait srv'
rospy.wait_for_service('item_pose_est_adddata_right')
rospy.wait_for_service('item_pose_est_right')

print 'proxy'
sv_item_pose_est_adddata = rospy.ServiceProxy('item_pose_est_adddata_right', ItemPoseEstAddData)
item_pose_est_srv = rospy.ServiceProxy('item_pose_est_right', ItemPoseEst)

print 'execute'
#item = 'fitness_gear_3lb_dumbbell'
item = 'rolodex_jumbo_pencil_cup'

req = ItemPoseEstAddDataRequest()
req.item = item
req.x = 604
req.y = -646
req.z = 212
req.ax = 180
req.ay = -18
req.az = -180
req.is_first_data = True
            
sv_item_pose_est_adddata(req)

res = item_pose_est_srv(item)

print res
