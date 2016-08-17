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

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import socket, re, time, datetime, math
import actionlib
from apc2016.msg import RobotArmMoveGlobalAction, RobotArmMoveGlobalResult


class ArmControlLR:
    def __init__(self, where, port):
        self.where = where
        self.connected = False
        self.action_server = actionlib.SimpleActionServer('move_arm_' + where,
            RobotArmMoveGlobalAction, execute_cb=self.callback, auto_start=False)
        self.action_server.start()

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(("", port))
        self.s.listen(1)
        self.soc, self.addr = self.s.accept()
        rospy.loginfo("Connected by %s" % str(self.addr))
        #self.soc.settimeout(120.0)  # two minute timeout

        self.connected = True


    def callback(self, message):
        if self.connected == False:
            result = RobotArmMoveGlobalResult(success=False)
            self.action_server.set_aborted(result, "Not connected yet")
        else:
            x = message.target.linear.x
            y = message.target.linear.y
            z = message.target.linear.z
            w = message.target.angular.x
            p = message.target.angular.y
            r = message.target.angular.z
            kakujiku = message.target.kakujiku
            fut000 = message.target.fut000
            msg = "%02d%d %.3f %.3f %.3f %.3f %.3f %.3f" % \
                (fut000, kakujiku, x, y, z, w, p, r)
            self.soc.sendall(msg+"\n")

            try:
                data = self.soc.recv(1024)
                rospy.loginfo("robot response: %s" % data)
            except socket.timeout as e:
                rospy.logerr("exception while waiting for response: %s" % e)
                result = RobotArmMoveGlobalResult(success=False)
                self.action_server.set_aborted(result,
                                               "caught exception: %s" % e)
                return

            result = RobotArmMoveGlobalResult(success=True)
            self.action_server.set_succeeded(result)
