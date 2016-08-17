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

import time

import rospy
import actionlib
from geometry_msgs.msg import Point

from apc2016.msg import *


class DummyCalibration(object):
    def __init__(self):
        self.calib_server = actionlib.SimpleActionServer(
            'calibration', CalibrationAction,
            execute_cb=self.calibrate,
            auto_start=False)
        self.calib_server.start()

    def calibrate(self, goal):
        print "hoge"
        rospy.loginfo("moving around and detecting shelf position")
        time.sleep(1)
        rospy.loginfo("still moving")
        time.sleep(1)
        result = CalibrationResult(success=True,
            upper_left=Point(1250, 35, 1037),
            upper_right=Point(1250, -835, 1037))
        self.calib_server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node("calibration", anonymous=True)
    DummyCalibration()
    rospy.spin()
