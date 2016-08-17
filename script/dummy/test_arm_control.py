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
import unittest
import time

import rospy
import actionlib
from geometry_msgs.msg import Twist, Vector3

from apc2016.msg import *

from arm_control import DummyArmControl


_count = 0


class TestDummyArmControl(unittest.TestCase):
    def test_move_to_left(self):
        global _count
        _count += 1
        # build client
        client = actionlib.SimpleActionClient('move_to_left',
                                              RobotArmMoveAction)
        client.wait_for_server()
        # send command
        pos = numpy.asarray([550, -146, 752, 180, 0, 180])
        p = Vector3(pos[0], pos[1], pos[2])
        r = Vector3(pos[3], pos[4], pos[5])
        goal = RobotArmMoveGoal(target_position=Twist(p, r))
        client.send_goal(goal)
        # check result
        client.wait_for_result()
        res = client.get_result()
        self.assertTrue(res.success)
        self.assertEqual(res.position, Twist(p, r))

    def test_move_to_right(self):
        global _count
        _count += 1
        # build client
        client = actionlib.SimpleActionClient('move_to_right',
                                              RobotArmMoveAction)
        client.wait_for_server()
        # send command
        pos = numpy.asarray([550, -146, 752, 182, 0, 180])
        p = Vector3(pos[0], pos[1], pos[2])
        r = Vector3(pos[3], pos[4], pos[5])
        goal = RobotArmMoveGoal(target_position=Twist(p, r))
        client.send_goal(goal)
        # check result
        client.wait_for_result()
        res = client.get_result()
        self.assertTrue(res.success)
        self.assertEqual(res.position, Twist(p, r))

    def test_move_to_bin_left(self):
        global _count
        _count += 1
        # build client
        client = actionlib.SimpleActionClient('move_to_bin_left',
                                              BinToteMoveAction)
        client.wait_for_server()
        # send command
        goal = BinToteMoveGoal(bin="bin_A", position="left")
        client.send_goal(goal)
        # check result
        client.wait_for_result()
        res = client.get_result()
        self.assertTrue(res.success)
        self.assertTrue(res.position.angular.x == 181)

    def test_move_to_bin_right(self):
        global _count
        _count += 1
        # build client
        client = actionlib.SimpleActionClient('move_to_bin_right',
                                              BinToteMoveAction)
        client.wait_for_server()
        # send command
        goal = BinToteMoveGoal(bin="bin_B")
        client.send_goal(goal)
        # check result
        client.wait_for_result()
        res = client.get_result()
        self.assertTrue(res.success)
        self.assertTrue(res.position.angular.x == 184)

    def tearDown(self):
        # if we do not signal shutdown here,
        # unittest.main() will never return (???)
        if _count == 4:
            time.sleep(0.2)
            rospy.signal_shutdown("test is over")


if __name__ == '__main__':
    rospy.init_node("arm_control_test", anonymous=True)
    # launch the action server
    DummyArmControl()
    # run tests
    unittest.main()
