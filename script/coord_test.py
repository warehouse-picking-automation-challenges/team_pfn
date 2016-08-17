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
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3, Point
from apc2016.srv import *

class coordinate_transform_test:
    def __init__(self):
        rospy.wait_for_service('forget_calibration2')
        rospy.wait_for_service('update_calibration2')
        rospy.wait_for_service('bin2global2')
        rospy.wait_for_service('global2bin2')
        rospy.loginfo("ready")
    def call_service(self):
        print 'call forget_calibration'
        service = rospy.ServiceProxy('forget_calibration2', Success)
        response = service()
        print response
    def call_service2(self,x1=1250,x2=1250):
        print 'call update calibration'
        service = rospy.ServiceProxy('update_calibration2', CalibrationUpdate)
        response = service(upper_left = Point(x1,52,1057),upper_right=Point(x2,-843,1057))
    def call_service3(self):
        print 'call bin2global'
        service = rospy.ServiceProxy('bin2global2',CoordinateTransform)
        response = service(bin='bin_A',point = Twist(Vector3(-600,0,0),Vector3(89.9,-89.9,89.9)))
        print response
    def call_service4(self):
        print 'call global2bin'
        service = rospy.ServiceProxy('global2bin2',CoordinateTransform)
        response = service(bin='bin_G',point = Twist(Vector3(660,-145,62),Vector3(89.9,-89.9,89.9)))
        print response
    def call_service5(self):
        print 'call adjustglobal'
        service = rospy.ServiceProxy('adjustglobal2', CoordinateTransform)
        response = service(bin='',point=Twist(Vector3(755,-102,770),Vector3(180,-20,179.9)))
        print response.point.linear.x, response.point.linear.y

if __name__ == "__main__":
    rospy.loginfo("start transform test")
    s = coordinate_transform_test()
    #s.call_service()
    s.call_service2(1290,1320)
    #s.call_service3()
    #s.call_service4()
    s.call_service3()
    #s.call_service2(1220,1250)
    #s.call_service5()
    #s.call_service2(1280,1250)
    #s.call_service5()
    #s.call_service2(1250,1220)
    #s.call_service5()
    #s.call_service2(1250,1280)
    #s.call_service5()
