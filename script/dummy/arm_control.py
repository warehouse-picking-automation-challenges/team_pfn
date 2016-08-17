#!/usr/bin/python

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
import actionlib
from geometry_msgs.msg import Twist, Vector3

from apc2016.msg import *


class DummyArmControl(object):
    def __init__(self):
        self.srv_lowlevel_left = \
            actionlib.SimpleActionServer('move_to_left',
                                         RobotArmMoveAction,
                                         execute_cb=self.cb_move_to_left,
                                         auto_start=False)
        self.srv_highlevel_left = \
            actionlib.SimpleActionServer('move_to_bin_left',
                                         BinToteMoveAction,
                                         execute_cb=self.cb_move_to_bin_left,
                                         auto_start=False)
        self.srv_lowlevel_right = \
            actionlib.SimpleActionServer('move_to_right',
                                         RobotArmMoveAction,
                                         execute_cb=self.cb_move_to_right,
                                         auto_start=False)
        self.srv_highlevel_right = \
            actionlib.SimpleActionServer('move_to_bin_right',
                                         BinToteMoveAction,
                                         execute_cb=self.cb_move_to_bin_right,
                                         auto_start=False)
        self.srv_lowlevel_left.start()
        self.srv_highlevel_left.start()
        self.srv_lowlevel_right.start()
        self.srv_highlevel_right.start()

    def cb_move_to_left(self, goal):
        print "moving away right arm, then moving left arm:"
        print goal.target_position
        result = RobotArmMoveResult(success=True,
                                    position=goal.target_position)
        self.srv_lowlevel_left.set_succeeded(result)

    def cb_move_to_bin_left(self, goal):
        if goal.position:
            pos = goal.position
        else:
            pos = "photo"
        print "looking up position for %s/%s" % (goal.bin, pos)
        pos = numpy.asarray([550, -146, 752, 181, 0, 180])
        p = Vector3(pos[0], pos[1], pos[2])
        r = Vector3(pos[3], pos[4], pos[5])
        print "moving away right arm, then moving left arm"
        result = BinToteMoveResult(success=True, position=Twist(p, r))
        self.srv_highlevel_left.set_succeeded(result)

    def cb_move_to_right(self, goal):
        print "moving away left arm, then moving right arm:"
        print goal.target_position
        result = RobotArmMoveResult(success=True,
                                    position=goal.target_position)
        self.srv_lowlevel_right.set_succeeded(result)

    def cb_move_to_bin_right(self, goal):
        if goal.position:
            pos = goal.position
        else:
            pos = "photo"
        print "looking up position for %s/%s" % (goal.bin, pos)
        pos = numpy.asarray([550, -146, 752, 184, 0, 180])
        p = Vector3(pos[0], pos[1], pos[2])
        r = Vector3(pos[3], pos[4], pos[5])
        print "moving away left arm, then moving right arm"
        result = BinToteMoveResult(success=True, position=Twist(p, r))
        self.srv_highlevel_right.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node("arm_control_dummy", anonymous=True)
    DummyArmControl()
    rospy.spin()
