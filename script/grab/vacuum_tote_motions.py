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

import arm_control_wrapper
import time
import numpy as np
import rospy

import util
from util import twist2array, array2twist, force_retry_items
from apc2016.msg import *
from apc2016.srv import *
from arm_control import LeftRight_arm

class motion_base(object):
    def __init__(self):
        
        self.t_bin_srv = rospy.ServiceProxy('global2bin',CoordinateTransform)
        self.f_bin_srv = rospy.ServiceProxy('bin2global',CoordinateTransform)


class pick_tote_from_above(motion_base):
    def __init__(self, bin, item, pos):
        
        super(pick_tote_from_above, self).__init__()
        self.bin = bin
        self.item = item
        
        
        target_pos = np.asarray([0,0,0,0,0,0]).astype('f')
        target_pos[:3] += pos
        target_pos_global = target_pos.copy()
        target_pos_global[3:] = np.asarray([-180, 0, 0]).astype('f')
        target_pos_global[:3] += np.asarray([0,0,650+10]).astype('f')
        
        target_pos_global[2] = max(-120, target_pos_global[2])
        if item=="dasani_water_bottle":
            target_pos_global[2] += 50
        
        self.target_pos              = target_pos_global + np.asarray([0,0,0,0,0,0])
        self.pre_target_pos          = target_pos_global + np.asarray([0,0,400,0,0,0])
        self.approach_target_pos     = target_pos_global + np.asarray([0,0,100,0,0,0])
        self.approach_target_pos_ret = target_pos_global + np.asarray([0,0,100,0,0,0])
        self.pre_target_pos_ret      = target_pos_global + np.asarray([0,0,400,0,0,0])
        
    def check_safety(self):
        ret = True
        #ret &= LeftRight_arm.inside_bin_check(self.approach_target_pos[:3].tolist(), self.bin)
        ret &= LeftRight_arm.inside_tote_check((self.target_pos[:]+np.asarray([0,0,-10,0,0,0])).tolist())
        
        if self.item=="kleenex_paper_towels":
            if self.target_pos[2] > 50:
                return False
        if self.item=="dasani_water_bottle":
            if self.target_pos[2] > -20:
                return False
                
        return ret
        
    def move(self, grabber):
        grabber.pipe_0_deg()
        print 'pre_target_pos'
        arm_control_wrapper.move_left_arm_global(self.pre_target_pos, keitai='nut000', kakujiku='chokusen', speed="fast")
        print 'approach_target_pos'
        arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen', speed="fast")
        grabber.vacuum_on()
        print 'target_pos'
        arm_control_wrapper.move_left_arm_global(self.target_pos, keitai='nut000', kakujiku='chokusen')
        time.sleep(1.0)
        # konoyouna shori wo ireru?
        #if grabber.vac_high(self.item).success:
        #    grabber.vacuum_off()
        #    time.sleep(0.5)
        #    grabber.vacuum_on()
        print 'approach_target_pos_ret'
        arm_control_wrapper.move_left_arm_global(self.approach_target_pos_ret, keitai='nut000', kakujiku='chokusen')
        
        force_retry = True if self.item in force_retry_items else False
        time.sleep(0.5)
        if self.item=="scotch_duct_tape" and not grabber.vac_ok(self.item).success:
            print 'high pressure -> retry # duct tape specific'
            grabber.pipe_90_deg()
            arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([-30,0,-10,0,0,0]), keitai='nut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
        elif self.item!="scotch_duct_tape":
            if (force_retry or not grabber.vac_ok(self.item).success) and self.target_pos[0]>50:
                grabber.vacuum_on()
                print 'high pressure -> retry #1'
                arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([0,0,-10,0,0,0]), keitai='nut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
            if (force_retry or not grabber.vac_ok(self.item).success):
                grabber.vacuum_on()
                print 'high pressure -> retry #2'
                arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([-20,0,0,0,0,0]), keitai='nut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
            if (force_retry or not grabber.vac_ok(self.item).success):
                print 'high pressure -> retry #3'
                arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([20,0,0,0,0,0]), keitai='nut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
                print 'high pressure -> retry #4'
                arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([0,-20,0,0,0,0]), keitai='nut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
            if (force_retry or not grabber.vac_ok(self.item).success):
                print 'high pressure -> retry #5'
                arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([0,20,0,0,0,0]), keitai='nut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
        
        if not grabber.vac_ok(self.item).success:
            rospy.loginfo("pick tried but seemes not success (vac not ok)")
            arm_control_wrapper.move_left_arm_to_bin('tote','', keitai='nut000', kakujiku='chokusen')
            grabber.vacuum_off()
            return False
           
        print 'pre_target_pos_ret'
        arm_control_wrapper.move_left_arm_global(self.pre_target_pos_ret, keitai='nut000', kakujiku='chokusen')
        
        time.sleep(0.5)
        if not grabber.vac_ok(self.item).success:
            rospy.loginfo("pick tried but seemes not success (vac not ok)")
            arm_control_wrapper.move_left_arm_to_bin('tote','', keitai='nut000', kakujiku='chokusen')
            grabber.vacuum_off()
            return False
        #arm_control_wrapper.move_left_arm_to_bin('tote','', keitai='nut000', kakujiku='chokusen')
        #arm_control_wrapper.move_left_arm_to_bin(self.bin,'pull', keitai='fut000', kakujiku='chokusen')  # TODO kore ha global de ugokashi tain dakedo
        return True


class pick_tote_from_side(motion_base):
    def __init__(self, bin, item, pos, approach_dir):
        
        super(pick_tote_from_side, self).__init__()
        self.bin = bin
        self.item = item
        
        if approach_dir=='xm':
            self.dx = -20
            self.dx2 = -50
            self.dy = 0
            self.dy2 = 0
            self.r = -180
        
        if approach_dir=='xp':
            self.dx = 20
            self.dx2 = 50
            self.dy = 0
            self.dy2 = 0
            self.r = 0
        
        if approach_dir=='ym':
            self.dy = -20
            self.dy2 = -50
            self.dx = 0
            self.dx2 = 0
            self.r = -90
        
        if approach_dir=='yp':
            self.dy = 20
            self.dy2 = 50
            self.dx = 0
            self.dx2 = 0
            self.r = 90
        
        
        target_pos = np.asarray([0,0,0,0,0,0]).astype('f')
        target_pos[:3] += pos
        target_pos_global = target_pos.copy()
        target_pos_global[3:] = np.asarray([180,0,self.r]).astype('f')
        target_pos_global[:3] += np.asarray([self.dx,self.dy,650-30]).astype('f')
        self.target_pos              = target_pos_global + np.asarray([0,0,0,0,0,0])
        self.pre_target_pos          = target_pos_global + np.asarray([self.dx2,self.dy2,400,0,0,0])
        self.approach_target_pos0     = target_pos_global + np.asarray([self.dx2,self.dy2,60,0,0,0])
        self.approach_target_pos     = target_pos_global + np.asarray([self.dx2,self.dy2,30,0,0,0])
        self.approach_target_pos_ret = target_pos_global + np.asarray([self.dx2,self.dy2,30,0,0,0])
        self.pre_target_pos_ret      = target_pos_global + np.asarray([self.dx2,self.dy2,400,0,0,0])
        
    def check_safety(self):
        ret = True
        #ret &= LeftRight_arm.inside_bin_check(self.approach_target_pos[:3].tolist(), self.bin)
        ret &= LeftRight_arm.inside_tote_check(self.approach_target_pos[:].tolist())
        #ret &= LeftRight_arm.inside_tote_check(self.target_pos[:].tolist())
        if self.item=="dasani_water_bottle":
            return False
        return ret
        
    def move(self, grabber):
        grabber.pipe_0_deg()
        print 'pre_target_pos'
        arm_control_wrapper.move_left_arm_global(self.pre_target_pos, keitai='nut000', kakujiku='kakujiku', speed="fast")
        print 'approach_target_pos'
        grabber.vacuum_on()
        arm_control_wrapper.move_left_arm_global(self.approach_target_pos0, keitai='nut000', kakujiku='chokusen', speed="fast")
        
        time.sleep(0.5)
        if grabber.vac_ok("").success:
            rospy.loginfo("pick tried but seemes not success (butsukatta?)")
            grabber.vacuum_off()
            arm_control_wrapper.move_left_arm_to_bin('tote','', keitai='nut000', kakujiku='chokusen')
            return False
            
        arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen', speed="fast")
        
        time.sleep(0.5)
        if grabber.vac_ok("").success:
            rospy.loginfo("pick tried but seemes not success (butsukatta?)")
            grabber.vacuum_off()
            arm_control_wrapper.move_left_arm_to_bin('tote','', keitai='nut000', kakujiku='chokusen')
            return False
            
        grabber.pipe_90_deg()
        print 'target_pos'
        arm_control_wrapper.move_left_arm_global(self.target_pos, keitai='nut000', kakujiku='chokusen')
        time.sleep(1.0)
        print 'approach_target_pos_ret'
        arm_control_wrapper.move_left_arm_global(self.approach_target_pos_ret, keitai='nut000', kakujiku='chokusen')
        
        force_retry = True if self.item in force_retry_items else False
        time.sleep(0.5)
        if (force_retry or not grabber.vac_ok(self.item).success) and self.target_pos[0]>50:
            grabber.vacuum_on()
            print 'high pressure -> retry #1'
            arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([0,0,-10,0,0,0]), keitai='nut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
        if (force_retry or not grabber.vac_ok(self.item).success):
            print 'high pressure -> retry #2'
            arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([-20,0,0,0,0,0]), keitai='nut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
        if (force_retry or not grabber.vac_ok(self.item).success):
            print 'high pressure -> retry #3'
            arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([20,0,0,0,0,0]), keitai='nut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
            print 'high pressure -> retry #4'
            arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([0,-20,0,0,0,0]), keitai='nut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
        if (force_retry or not grabber.vac_ok(self.item).success):
            print 'high pressure -> retry #5'
            arm_control_wrapper.move_left_arm_global(self.target_pos + np.asarray([0,20,0,0,0,0]), keitai='nut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_global(self.approach_target_pos, keitai='nut000', kakujiku='chokusen')
        
        if not grabber.vac_ok(self.item).success:
            rospy.loginfo("pick tried but seemes not success (vac not ok)")
            grabber.vacuum_off()
            arm_control_wrapper.move_left_arm_to_bin('tote','', keitai='nut000', kakujiku='chokusen')
            return False
          
        print 'pre_target_pos_ret'
        grabber.pipe_0_deg_slow()
        arm_control_wrapper.move_left_arm_global(self.pre_target_pos_ret, keitai='nut000', kakujiku='chokusen')
        #arm_control_wrapper.move_left_arm_to_bin('tote','', keitai='nut000', kakujiku='chokusen')
        #arm_control_wrapper.move_left_arm_to_bin(self.bin,'pull', keitai='fut000', kakujiku='chokusen')  # TODO kore ha global de ugokashi tain dakedo
        #grabber.pipe_90_deg_very_slow()
        
        time.sleep(0.5)
        if not grabber.vac_ok(self.item).success:
            grabber.vacuum_off()
            rospy.loginfo("pick tried but seemes not success (vac not ok)")
            arm_control_wrapper.move_left_arm_to_bin('tote','', keitai='nut000', kakujiku='chokusen')
            return False
        return True

