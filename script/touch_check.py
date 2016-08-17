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

## Node Name: touch_check
## Service Name: touch_check
## Service Type: Success (input Empty output success(bool))
## Subscribe topic: "touch" touch sensor data from Arduino
## provide service to return wether bellows touches something or not

import rospy
import time
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from apc2016.srv import Success
from apc2016.srv import SuccessResponse


class Touch_check:

    def __init__(self):
        self.sub = rospy.Subscriber('contact', Bool, self.callback)
        self.pub = rospy.Publisher('call_touch', Empty, queue_size = 10)
        self.service = rospy.Service('touch_check', Success, self.touch_handler)
        self.touch = False
        self.called = 0

    def callback(self, message):
        self.touch = message.data
        self.called = 1

    def touch_handler(self,req):
        self.pub.publish()
        count = 0
        while(self.called == 0 and count < 10):
            time.sleep(0.1)
            count = count + 1
        return SuccessResponse(success=self.touch)

if __name__=="__main__":
    rospy.init_node('touch_check')
    touch_check = Touch_check()
    rospy.spin()
