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
from apc2016.msg import *
import actionlib


print 'init'
rospy.init_node("test_calib")


client = actionlib.SimpleActionClient('calibration', CalibrationAction)
print 'wait server'
client.wait_for_server()

g = CalibrationActionGoal()
client.send_goal(g)
print 'srv called'
client.wait_for_result(rospy.Duration.from_sec(300.0))

result = client.get_result()
print result
