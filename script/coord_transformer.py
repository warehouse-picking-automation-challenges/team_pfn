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

# node name 'coord_transformer'
# provide 4 services 'forget_calibration', 'update_calibration', 'global2bin' and 'bin2global'
import numpy as np
import math
import conv_bin_coord
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3, Point
from apc2016.srv import *
from apc2016.msg import FanucTwist
from util import camera_to_global, camera_to_global_right, camera_to_global_right_all

class coordinate_transform:
    def __init__(self):
        self.is_calibrated = False
        self.point_1 = np.asarray([1250,40,1057])
        self.point_2 = np.asarray([1250,-830,1057])
        self.service_f = rospy.Service('forget_calibration', Success, self.forget_calibration)
        self.service_u = rospy.Service('update_calibration', CalibrationUpdate, self.update_calibration)
        self.service_c = rospy.Service('check_is_calibrated', CalibData, self.check_is_calibration)
        self.service_b = rospy.Service('bin2global', CoordinateTransform, self.bin2global)
        self.service_g = rospy.Service('global2bin', CoordinateTransform, self.global2bin)
        self.service_a = rospy.Service('adjustglobal', CoordinateTransform, self.adjustglobal)
        self.service_c2g = rospy.Service('camera2global', CameraCoordinateTransform, self.camera2global)
        self.service_c2g = rospy.Service('camera2global_right', CameraCoordinateTransform, self.camera2global_right)
        self.service_c2ga = rospy.Service('camera2global_right_all', CameraCoordinateTransformAll, self.camera2global_right_all)
        self.c = conv_bin_coord.conv_bin_coord()
    def forget_calibration(self, req):
        self.is_calibrated = False
        return SuccessResponse(success=True)

    def check_is_calibration(self, req):
        if self.is_calibrated == True:
            p_1 = Point(self.point_1[0],self.point_1[1],self.point_1[2])
            p_2 = Point(self.point_2[0],self.point_2[1],self.point_2[2])
        else:
            p_1 = Point(0,0,0)
            p_2 = Point(0,0,0)
        return CalibDataResponse(success=self.is_calibrated, upper_left = p_1, upper_right = p_2)

    def update_calibration(self, req):
        self.point_1[0] = req.upper_left.x
        self.point_1[1] = req.upper_left.y
        self.point_1[2] = req.upper_left.z
        self.point_2[0] = req.upper_right.x
        self.point_2[1] = req.upper_right.y
        self.point_2[2] = req.upper_right.z
        self.is_calibrated = True
        return CalibrationUpdateResponse(success=True)

    def bin2global(self, req):
        if req.bin not in ['bin_' + j for j in 'ABCDEFGHIJKL']:
            rospy.loginfo('Invalid bin name')
            return CoordinateTransformResponse(point = req.point, is_calibrated = self.is_calibrated, success=False)
    
        if self.is_calibrated == False:
            return CoordinateTransformResponse(point = req.point, is_calibrated = self.is_calibrated, success=False)

        point = self.c.conv_from_bin_coord_to_global_coord(self.twist2array(req.point), req.bin, self.point_1, self.point_2)
        return CoordinateTransformResponse(point = self.array2twist(point), is_calibrated = self.is_calibrated, success=True)

    def global2bin(self, req):
        if req.bin not in ['bin_' + j for j in 'ABCDEFGHIJKL']:
            rospy.loginfo('Invalid bin name')
            return CoordinateTransformResponse(point = req.point, is_calibrated = self.is_calibrated, success=False)
    
        point = self.c.conv_to_bin_coord_from_global_coord(self.twist2array(req.point), req.bin, self.point_1, self.point_2)
        return CoordinateTransformResponse(point = self.array2twist(point), is_calibrated = self.is_calibrated, success=True)
        
    def camera2global(self, req):
        ret = np.zeros(6).astype('f')
        ret[:3] = camera_to_global(self.twist2array(req.point)[:3], self.twist2array(req.xyzwpr))
        return CameraCoordinateTransformResponse(point = self.array2twist(ret))
        
        
    def camera2global_right(self, req):
        ret = np.zeros(6).astype('f')
        ret[:3] = camera_to_global_right(self.twist2array(req.point)[:3], self.twist2array(req.xyzwpr))
        return CameraCoordinateTransformResponse(point = self.array2twist(ret))
        
    def camera2global_right_all(self, req):
        n = len(req.x)
        rospy.loginfo("camera2global_right_all called, len = %d"%n)
        points = np.zeros((3, n)).astype("f")
        points[0,:] = req.x
        points[1,:] = req.y
        points[2,:] = req.z
        ret = camera_to_global_right(points, self.twist2array(req.xyzwpr))
        rospy.loginfo("camera2global_right_all finished")
        return CameraCoordinateTransformAllResponse(x = ret[0,:].tolist(), y = ret[1,:].tolist(), z = ret[2,:].tolist())
        
    
    def adjustglobal(self,req):
        point = self.c.adjustglobal(self.twist2array(req.point), self.point_1, self.point_2)
        return CoordinateTransformResponse(point = self.array2twist(point), is_calibrated = self.is_calibrated, success=True)

    def twist2array(self,t):
        return np.asarray( [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z])

    def array2twist(self,t):
        return Twist(Vector3(t[0],t[1],t[2]), Vector3(t[3],t[4],t[5]))
if __name__ == "__main__":
    rospy.init_node('coord_transformer')
    coordinate_transform()
    rospy.spin()
