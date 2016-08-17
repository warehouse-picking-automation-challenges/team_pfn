#! /usr/bin/env python

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

# from <http://wiki.ros.org/actionlib#Python_SimpleActionpipe_client>
# see also <http://docs.ros.org/api/actionlib/html/classactionlib_1_1simple__action__pipe_client_1_1SimpleActionpipe_client.html>

import sys
import time
import random
import math

import roslib
#roslib.load_manifest('my_pkg_name')
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from apc2016.msg import *

def print_status(fb):
    print "progress:\n ", "\n  ".join(str(fb).split("\n"))

if __name__ == '__main__':
    rospy.init_node('hoge_foobar')

    if False:
        pipe_client = actionlib.SimpleActionClient('turn_pipe', GenericAction)
        pipe_client.wait_for_server()

        hand_client = actionlib.SimpleActionClient('move_hand', GenericAction)
        hand_client.wait_for_server()

        pipe_goal = GenericGoal()
        pipe_goal.target = str(int(round(random.random())))  # 0 (down) or 1 (straight)
        pipe_client.send_goal(pipe_goal, feedback_cb=print_status)

        hand_goal = GenericGoal()
        hand_goal.target = "%.2f %.2f" % (random.random()*70, random.random()*180)
        hand_client.send_goal(hand_goal, feedback_cb=print_status)

        pipe_client.wait_for_result(rospy.Duration.from_sec(5.0))
        hand_client.wait_for_result(rospy.Duration.from_sec(5.0))

        pipe_res = pipe_client.get_result()
        hand_res = hand_client.get_result()

    if False:
        pipe_client = actionlib.SimpleActionClient('turn_pipe', GenericAction)
        pipe_client.wait_for_server()

        pipe_goal = GenericGoal()
        pipe_goal.target = sys.argv[1]
        pipe_client.send_goal(pipe_goal, feedback_cb=print_status)

        pipe_client.wait_for_result(rospy.Duration.from_sec(5.0))

        pipe_res = pipe_client.get_result()

    if True:
        pipe_client = actionlib.SimpleActionClient('/gripper_angle', GenericAction)
        pipe_client.wait_for_server()

        pipe_goal = GenericGoal()
        pipe_goal.target = sys.argv[1] # "240.01"
        pipe_client.send_goal(pipe_goal, feedback_cb=print_status)

        pipe_client.wait_for_result(rospy.Duration.from_sec(30.0))

        pipe_res = pipe_client.get_result()

        print "GRIP result:\n|", "\n| ".join(str(pipe_res).split("\n"))

    #print "HAND result:\n|", "\n| ".join(str(hand_res).split("\n"))
