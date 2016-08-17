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

import unittest
import time

import rospy
import actionlib
from geometry_msgs.msg import Point

from apc2016.msg import *

from calibration import DummyCalibration


class TestDummyCalibration(unittest.TestCase):
    def test_calibration(self):
        # build client
        client = actionlib.SimpleActionClient('calibration',
                                              CalibrationAction)
        client.wait_for_server()
        # send command
        goal = CalibrationGoal()
        client.send_goal(goal)
        # check result
        client.wait_for_result()
        res = client.get_result()
        self.assertTrue(res.success)
        self.assertEqual(res.upper_left, Point(1250, 35, 1037))
        self.assertEqual(res.upper_right, Point(1250, -835, 1037))

    def tearDown(self):
        # if we do not signal shutdown here,
        # unittest.main() will never return (???)
        time.sleep(0.2)
        rospy.signal_shutdown("test is over")


if __name__ == '__main__':
    rospy.init_node("calibration_test", anonymous=True)
    # launch the action server
    DummyCalibration()
    # run tests
    unittest.main()
