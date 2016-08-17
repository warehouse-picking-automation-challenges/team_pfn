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
import interface
import numpy as np
import rospy
import actionlib
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Vector3
from actionlib_msgs.msg import GoalStatus
from apc2016.msg import *
from apc2016.srv import *
import os, sys
from cv_bridge import CvBridge
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')
import arm_control_wrapper
from arm_control import LeftRight_arm

#from util import camera_to_global, item_string_to_id

import chainer
from chainer import cuda
from chainer import serializers
from chainer import Variable
import chainer.functions as F
import chainer.links as L



import util


def l2a(p,a):
    return np.asarray(p + a)
    
def l2a_(p):
    return np.asarray(p)
    
class gripper_sensing:
    def __init__(self):
        
        rospy.loginfo( 'wait for item_pose_est_right')
        rospy.wait_for_service('item_pose_est_right')
        self.sv_item_pose_est = rospy.ServiceProxy('item_pose_est_right', ItemPoseEst)
        
        rospy.loginfo( 'wait bin2global and global2bin')
        rospy.wait_for_service('bin2global')
        rospy.wait_for_service('global2bin')
        self.sv_bin2global = rospy.ServiceProxy('bin2global',CoordinateTransform)
        self.sv_global2bin = rospy.ServiceProxy('global2bin',CoordinateTransform)
        
        rospy.loginfo( 'wait item_pose_est_adddata_right')
        rospy.wait_for_service('item_pose_est_adddata_right')
        self.sv_item_pose_est_adddata = rospy.ServiceProxy('item_pose_est_adddata_right', ItemPoseEstAddData)
        
        rospy.loginfo('gripper -- ok!')
        
    def move_for_scan(self, pos, angle):
        arm_control_wrapper.move_right_arm_local(self.target_bin, l2a(pos, angle), 'nut000', 'chokusen', 'normal')
        
        self.prev_grip_armpose = l2a(pos, angle)
        
    def move_for_grip(self, pos, angle):
        arm_control_wrapper.move_right_arm_local(self.target_bin, l2a(pos, angle), 'fut000', 'kakujiku', 'normal')
        
        self.prev_grip_armpose = l2a(pos, angle)
        
        
    def move_for_grip_tote(self, p):
        arm_control_wrapper.move_right_arm_global( p, 'fut000', 'kakujiku', 'normal')
        
        self.prev_grip_armpose = l2a(pos, angle)
        
          
    def sensing_normal(self, bin, item):

        #scan poses
        pf = np.asarray([-410,-100,315,-180,-24,-180])
        pl = np.asarray([-410,30,315,-180,-24,-200])
        pr = np.asarray([-410,-260,315,-180,-24,200])
        pb = np.asarray([-520,-100,115,-180,0,180])
        
        for i, p in enumerate([pf, pl, pr, pb]):
            rospy.loginfo('gripper -- scan pos')
            
            res = arm_control_wrapper.move_right_arm_local(bin, p, 'nut000', 'kakujiku')
            rospy.loginfo(res)
            #raw_input("stop")
            
            rospy.loginfo('gripper -- convert arm pose to global')
            ct = CoordinateTransformRequest()
            ct.bin = bin
            ct.point = util.array2twist(p)
            res = self.sv_bin2global(ct)
            pp = util.twist2array(res.point)
            rospy.loginfo(pp)
            
            rospy.loginfo('gripper -- add pcl data')
            req = ItemPoseEstAddDataRequest()
            req.item = item
            req.x = pp[0] 
            req.y = pp[1]
            req.z = pp[2]
            req.ax = pp[3]
            req.ay = pp[4]
            req.az = pp[5]
            req.is_first_data = True if i == 0 else False
            self.sv_item_pose_est_adddata(req)
            #raw_input("stop")
        
        
        rospy.loginfo('gripper -- call item_pose_est')
        res = self.sv_item_pose_est(item)
        rospy.loginfo(res)
        #raw_input("stop")
        if res.success == False:
            return False, None

        if res.no_item == True:
            return False, None

        rospy.loginfo('gripper -- convert item coordinates global to bin')
        ct = CoordinateTransformRequest()
        ct.bin = bin
        ct.point = util.array2twist([res.x, res.y, res.z, res.ax, res.ay, res.az])
        res_ = self.sv_global2bin(ct)
        itmps_bin = util.twist2array(res_.point)
        
        itmps_bin[3] = res.ax
        itmps_bin[4] = res.ay
        itmps_bin[5] = res.az
        
        rospy.loginfo('gripper -- item in bin %f %f %f %f %f %f' % (itmps_bin[0], itmps_bin[1], itmps_bin[2], itmps_bin[3], itmps_bin[4], itmps_bin[5]))
        
        return True, itmps_bin
        
        
    def sensing_normal2(self, bin, item):

        #scan poses
        
        if bin == 'bin_B' or bin == 'bin_E' or bin == 'bin_H' or bin == 'bin_K':
            cy = -150
        else:
            cy = -125
        
        
        #pf = np.asarray([-430, cy, 304,-180,-25,-180])
        pf = np.asarray([-450, cy, 304,-180,-25,-180])    
        pp = np.asarray([-530, cy, 150,-180,-25,-180])
      
        
        if bin == 'bin_A' or bin == 'bin_B' or bin == 'bin_C':
            res = arm_control_wrapper.move_right_arm_local(bin, pp, 'nut000', 'kakujiku')

        for i, p in enumerate([pf]):
            rospy.loginfo('gripper -- scan pos')
            
            res = arm_control_wrapper.move_right_arm_local(bin, p, 'nut000', 'kakujiku')
            rospy.loginfo(res)
            #raw_input("stop")
            
            time.sleep(1)
            
            rospy.loginfo('gripper -- convert arm pose to global')
            ct = CoordinateTransformRequest()
            ct.bin = bin
            ct.point = util.array2twist(p)
            res = self.sv_bin2global(ct)
            pp = util.twist2array(res.point)
            rospy.loginfo(pp)
            
            rospy.loginfo('gripper -- add pcl data')
            req = ItemPoseEstAddDataRequest()
            req.item = item
            req.x = pp[0] 
            req.y = pp[1]
            req.z = pp[2]
            req.ax = pp[3]
            req.ay = pp[4]
            req.az = pp[5]
            req.is_first_data = True if i == 0 else False
            self.sv_item_pose_est_adddata(req)
            #raw_input("stop")
        
        
        rospy.loginfo('gripper -- call item_pose_est')
        res = self.sv_item_pose_est(item)
        rospy.loginfo(res)
        #raw_input("stop")
        if res.success == False:
            return False, None

        if res.no_item == True:
            return False, None

        rospy.loginfo('gripper -- convert item coordinates global to bin')
        ct = CoordinateTransformRequest()
        ct.bin = bin
        ct.point = util.array2twist([res.x, res.y, res.z, res.ax, res.ay, res.az])
        res_ = self.sv_global2bin(ct)
        itmps_bin = util.twist2array(res_.point)
        
        itmps_bin[3] = res.ax
        itmps_bin[4] = res.ay
        itmps_bin[5] = res.az
        
        rospy.loginfo('gripper -- item in bin %f %f %f %f %f %f' % (itmps_bin[0], itmps_bin[1], itmps_bin[2], itmps_bin[3], itmps_bin[4], itmps_bin[5]))
        
        return True, itmps_bin
        
        
    def sensing_norma_for_pencilcup(self, bin, item):

        #scan poses
        
        if bin == 'bin_B' or bin == 'bin_E' or bin == 'bin_H' or bin == 'bin_K':
            cy = -150
        else:
            cy = -125
        
        
        pf = np.asarray([-430, cy, 304,-180,-25,-180])  
        pp = np.asarray([-530, cy, 150,-180,-25,-180])
      
        
        if bin == 'bin_A' or bin == 'bin_B' or bin == 'bin_C':
            res = arm_control_wrapper.move_right_arm_local(bin, pp, 'nut000', 'kakujiku')

        for i, p in enumerate([pf]):
            rospy.loginfo('gripper -- scan pos')
            
            res = arm_control_wrapper.move_right_arm_local(bin, p, 'nut000', 'kakujiku')
            rospy.loginfo(res)
            #raw_input("stop")
            
            time.sleep(1)
            
            rospy.loginfo('gripper -- convert arm pose to global')
            ct = CoordinateTransformRequest()
            ct.bin = bin
            ct.point = util.array2twist(p)
            res = self.sv_bin2global(ct)
            pp = util.twist2array(res.point)
            rospy.loginfo(pp)
            
            rospy.loginfo('gripper -- add pcl data')
            req = ItemPoseEstAddDataRequest()
            req.item = item
            req.x = pp[0] 
            req.y = pp[1]
            req.z = pp[2]
            req.ax = pp[3]
            req.ay = pp[4]
            req.az = pp[5]
            req.is_first_data = True if i == 0 else False
            self.sv_item_pose_est_adddata(req)
            #raw_input("stop")
        
        
        rospy.loginfo('gripper -- call item_pose_est')
        res = self.sv_item_pose_est(item)
        rospy.loginfo(res)
        #raw_input("stop")
        if res.success == False:
            return False, None

        if res.no_item == True:
            return False, None

        itmps_bin = [res.x, res.y, res.z, res.ax, res.ay, res.az]

        rospy.loginfo('gripper -- pencil sensing in bin %f %f %f %f %f %f' % (itmps_bin[0], itmps_bin[1], itmps_bin[2], itmps_bin[3], itmps_bin[4], itmps_bin[5]))
        
        return True, itmps_bin
        
        
        
    def sensing_tote(self, item):
        
        #bef moving
        rospy.loginfo('gripper -- pre moving')       
        arm_control_wrapper.move_right_arm_global( l2a_([550,-900,100,-180, -0,-180]), 'nut000', 'chokusen', 'normal')
        #arm_control_wrapper.move_right_arm_global( l2a_([550,-900,100, -90, 90, -90]), 'fut000', 'kakujiku', 'normal')
        #arm_control_wrapper.move_right_arm_global( l2a_([550,-900,100,90,0,90]), 'fut000', 'kakujiku', 'fast')
        arm_control_wrapper.move_right_arm_global( l2a_([550,-900,100,90,-90,90]), 'fut000', 'kakujiku', 'normal')

            
            
        prg = np.asarray([550,-550,-220,90,-90,90])
        plg = np.asarray([550,-250,-220,90,-90,90])
        
        for i, p in enumerate([prg, plg]):
            rospy.loginfo('gripper -- scan pos')            
            arm_control_wrapper.move_right_arm_global( l2a_(p), 'fut000', 'chokusen', 'normal')
            
            rospy.loginfo('gripper -- add pcl data')
            req = ItemPoseEstAddDataRequest()
            req.item = item
            req.x = p[0] 
            req.y = p[1]
            req.z = p[2]
            req.ax = p[3]
            req.ay = p[4]
            req.az = p[5]
            req.is_first_data = True if i == 0 else False
            self.sv_item_pose_est_adddata(req)
            #raw_input("stop") 
        
        
        
        rospy.loginfo('gripper -- call item_pose_est')
        res = self.sv_item_pose_est(item)
        rospy.loginfo(res)
        #raw_input("stop")

            
        rospy.loginfo('gripper -- %f %f %f %f %f %f' % (res.x, res.y, res.z, res.ax, res.ay, res.az))
        
        #post moving
        rospy.loginfo('gripper -- post moving')        
        arm_control_wrapper.move_right_arm_global( l2a_([550,-900,100,90,-90,90]), 'fut000', 'chokusen', 'normal')
        #arm_control_wrapper.move_right_arm_global( l2a_([550,-900,100,90,0,90]), 'fut000', 'kakujiku', 'normal')
        #arm_control_wrapper.move_right_arm_global( l2a_([550,-900,100, -90, 90, -90]), 'fut000', 'kakujiku', 'normal')
        arm_control_wrapper.move_right_arm_global( l2a_([550,-900,100,-180, -0,-180]), 'nut000', 'kakujiku', 'normal')


        if res.success == False:
            return False, None

        if res.no_item == True:
            return False, None
            
        return True, [res.x, res.y, res.z, res.ax, res.ay, res.az]

        
        
