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

## Node Name: pipe_control
## Action Name: pipe_switch
## Action Type: IntCode2Success
## input 0 or 1:
## 0: pipe 0 degree
## 1: pipe 90 degree
## Publish topic: "relay_command" to control relay
## provide action to control pipes

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
import actionlib
from apc2016.msg import IntCode2SuccessAction
from apc2016.msg import IntCode2SuccessResult
from apc2016.msg import IntCode2SuccessFeedback

class PipeAction(object):

    def __init__(self):
        rospy.loginfo('start!')
        self.pub = rospy.Publisher('relay_command',Int16,queue_size=10)
        self.acs = actionlib.SimpleActionServer('turn_pipe_air', IntCode2SuccessAction, execute_cb=self.callback, auto_start=False)
        self.acs.start()
    
    def callback(self, goal):
        rospy.loginfo('call back called')
        res = IntCode2SuccessResult(success = True)
        if goal.id == 0:
            rospy.loginfo("pipe 0 deg")
            self.pub.publish(40)
            self.acs.set_succeeded(res)
        elif goal.id == 1:
            rospy.loginfo("pipe 90 deg")
            self.pub.publish(41)
            self.acs.set_succeeded(res)
        else:
            rospy.loginfo('Invalid id')
            res = IntCode2SuccessResult(success = False)
            self.acs.set_succeeded(res)

if __name__ == "__main__":
    rospy.init_node('pipe_control')
    pipe = PipeAction()
    rospy.spin()

