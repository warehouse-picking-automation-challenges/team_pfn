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
import time
from apc2016.srv import distance
from apc2016.srv import distanceResponse
from std_msgs.msg import Int16
from std_msgs.msg import Empty
class Dist_check:
    def __init__(self):
        self.pubr = rospy.Publisher('called_dist_r', Empty, queue_size = 10)
        self.publ = rospy.Publisher('called_dist_l', Empty, queue_size = 10)
        self.subr = rospy.Subscriber('distance_right', Int16, self.callbackr)
        self.subl = rospy.Subscriber('distance_left', Int16, self.callbackl)
        self.r_d = 0
        self.l_d = 0
        self.called_r = 0
        self.called_l = 0
        try:
            rospy.wait_for_service('d_r',5)
            rospy.loginfo('right distance sensor on')
            self.service = rospy.Service('distance_sensor_right', distance, self.disthandler_r)
        except:
            rospy.loginfo('no right distance sensor')
        try:
            rospy.wait_for_service('d_l',5)
            rospy.loginfo('left distance sensor on')
            self.service = rospy.Service('distance_sensor_left', distance, self.disthandler_l)
        except:
            rospy.loginfo('no left distance sensor')
    def callbackr(self, message):
        self.r_d = message.data
        self.called_r = 1
    def callbackl(self, message):
        self.l_d = message.data
        self.called_l = 1

    def disthandler_r(self, req):
        self.pubr.publish()
        count = 0
        while(self.called_r == 0 and count < 10):
            time.sleep(0.1)
            count = count + 1
        if count == 10:
            d = -1
        else:
            d = self.r_d
        self.called_r = 0
        return distanceResponse(dist = d)

    def disthandler_l(self, req):
        self.publ.publish()
        count = 0
        while(self.called_l == 0 and count < 10):
            time.sleep(0.1)
            count = count + 1
        if count == 10:
            d = -1
        else:
            d = self.l_d
        self.called_l = 0
        return distanceResponse(dist = d)


if __name__ == '__main__':
    rospy.init_node('dist_check')
    Dist_check()
    rospy.spin()

