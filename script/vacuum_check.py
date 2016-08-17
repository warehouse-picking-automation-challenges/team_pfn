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

## Node Name: vacuum_check
## Service Name: vacuum_check
## Service Type: ItemName2Success (input id(string) output success(bool))
## Subscribe topic: "pressure" pressure data from the sensor (Arduino) 
## provide service to return wether vaccum is success or fail

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool
from apc2016.srv import ItemName2Success
from apc2016.srv import ItemName2SuccessResponse


class Vac_check:

    def __init__(self):
        self.sub = rospy.Subscriber('pressure', Float32, self.callback)
        self.service = rospy.Service('vacuum_check',ItemName2Success, self.pressure_handler)
        #self.service_high = rospy.Service('vacuum_check_high',ItemName2Success, self.pressure_handler_high)
        self.pressure = 0
        self.basepressure = 1000
        self.flg = False
        self.normal_threshold = 805
        self.special_threshold = 0

    def callback(self, message):
        self.pressure = message.data
        if self.flg == False and message.data > 980:
            self.basepressure = message.data
            self.normal_threshold = self.basepressure - 195 #200 datta -> 180 ni
            self.special_threshold = self.basepressure - 185 #170 datta
            self.flg = True
            rospy.loginfo('set base pressure %s', self.pressure);

    def pressure_handler(self,req): 
        is_success = False
        rospy.loginfo("vacuum check req.id - %s" % req.id)
        # case if the target is scissors or toothbrush
        if req.id in ["fiskars_scissors_red","oral_b_toothbrush_red","oral_b_toothbrush_green", "kyjen_squeakin_eggs_plush_puppies"]:
        #if req.id in ["fiskars_scissors_red","oral_b_toothbrush_red","oral_b_toothbrush_green"]:
            rospy.loginfo("very small object")
            #is_success = True
            if self.pressure > 700 and self.pressure < self.special_threshold:
                is_success = True

        #otherwise
        else:
            if self.pressure > 700 and self.pressure < self.normal_threshold:
                is_success = True
        f = open("log_pressure.txt", "a")
        rospy.loginfo("item: %s pressure %s is_success %s"%(req.id, self.pressure, is_success))
        f.write("item: %s pressure %s is_success %s \n"%(req.id, self.pressure, is_success))
        f.close()
        return ItemName2SuccessResponse(success=is_success)


if __name__=="__main__":
    rospy.init_node('vacuum_check')
    vac_check = Vac_check()
    rospy.spin()
