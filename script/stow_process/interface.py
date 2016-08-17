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

import time
import threading

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from apc2016.msg import CalibrationAction, CalibrationGoal
from apc2016.srv import Success, CalibrationUpdate

import util


(logdebug, loginfo, logwarn, logerr) = util.get_loggers("StowProcessStrategy")


class StowProcessStrategy(object):
    def __init__(self, pos_info):
        self.pos_info = pos_info
        self.start = time.time()
        cd = threading.Thread(target=self.countdown)
        cd.daemon = True
        cd.start()

    def countdown(self):
        while True:
            time.sleep(10)
            elapsed = util.get_elapsed_time(self.start)
            loginfo("elapsed time: %s" % elapsed)

    def calibrate(self):
        # forget the calibration data
        loginfo("wait for forget_calibration service")
        rospy.wait_for_service('forget_calibration')
        f = rospy.ServiceProxy('forget_calibration', Success)
        try:
            success = f()
        except Exception as e:
            logwarn("forget_calibration failed with an error: %s" % e)
            return False
        if not success:
            logwarn("forget_calibration did not succeed")
            return False
        loginfo("forget_calibration was successful")

        # calibrate the shelf
        loginfo("wait for calibration service")
        client = actionlib.SimpleActionClient('calibration',
                                              CalibrationAction)
        client.wait_for_server()
        loginfo("calibrating shelf position")
        client.send_goal(CalibrationGoal())
        client.wait_for_result(rospy.Duration.from_sec(120.0))
        state = client.get_state()
        if state != GoalStatus.SUCCEEDED:
            logwarn("calibration did not finish in time")
            return False
        result = client.get_result()
        if not result.success:
            logwarn("calibration did not succeed")
            return False
        ul, ur = result.upper_left, result.upper_right
        loginfo("calibration result: %s, %s" % (ul, ur))
        if ul.y == -1:
            return False

        # now update the calibration with the points we just obtained
        loginfo("wait for update_calibration service")
        rospy.wait_for_service('update_calibration')
        cal = rospy.ServiceProxy('update_calibration', CalibrationUpdate)
        try:
            loginfo("send calibration data")
            success = cal(ul, ur)
        except Exception as e:
            logwarn("update_calibration failed with an error: %s" % e)
            return False
        if not success:
            logwarn("update_calibration did not succeed")
            return False
        loginfo("update_calibration was successful")
        return True

    def execute(self):
        """Sort the items passed in as a parameter by some
        metric and move them into the shelf."""
        raise NotImplementedError("override this")
