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

class pick_shelf_from_above(motion_base):
    def __init__(self, bin, item, pos):
        
        super(pick_shelf_from_above, self).__init__()
        self.bin = bin
        self.item = item
        
        
        target_pos = np.asarray([0,0,0,0,0,0]).astype('f')
        target_pos[:3] += pos
        target_pos_bin = twist2array(self.t_bin_srv(bin = self.bin, point=array2twist(target_pos)).point)
        target_pos_bin[3:] = np.asarray([90,-90,90]).astype('f')
        
        
        if self.item=="kleenex_paper_towels":
            if target_pos_bin[0] < 200:
                target_pos_bin[0] = 200
                
        target_pos_bin[:3] += np.asarray([-650,0, 20]).astype('f')
        self.target_pos              = target_pos_bin + np.asarray([0,0,0,0,0,0])
        self.pre_target_pos          = target_pos_bin + np.asarray([-200,0,100,0,0,0])
        self.approach_target_pos     = target_pos_bin + np.asarray([0,0,100,0,0,0])
        self.approach_target_pos_ret = target_pos_bin + np.asarray([0,0,150,0,0,0])
        self.pre_target_pos_ret      = target_pos_bin + np.asarray([-200,0,150,0,0,0])
        
        # hikidashi ha absolute coordinate
        self.pre_target_pos[0] = -100-650
        self.pre_target_pos_ret[0] = -100-650
        
        # hashira wo yokeru
        if bin in ['bin_A', 'bin_D', 'bin_G', 'bin_J']:
            if target_pos_bin[1]>-50:
                self.pre_target_pos[1] -= 30
                self.approach_target_pos[1] -= 30
                self.approach_target_pos_ret[1] -= 30
                self.pre_target_pos_ret[1] -= 30
        elif bin in ['bin_C', 'bin_F', 'bin_I', 'bin_L']:
            if target_pos_bin[1]<-200:
                self.pre_target_pos[1] += 30
                self.approach_target_pos[1] += 30
                self.approach_target_pos_ret[1] += 30
                self.pre_target_pos_ret[1] += 30
            
        # zlim based on item_size
        item_min_height = util.item_string_to_3side(item)[2]*10
        self.target_pos[2] = max(item_min_height+15, self.target_pos[2])
        
        if bin in ['bin_A', 'bin_B', 'bin_C']:
            zlim = 190
        elif bin in ['bin_D', 'bin_E', 'bin_F']:
            zlim = 160
        elif bin in ['bin_G','bin_H','bin_I']:
            zlim = 160
        elif bin in ['bin_J','bin_K','bin_L']:
            zlim = 190
        self.pre_target_pos[2] = zlim
        self.approach_target_pos[2] = zlim
        self.approach_target_pos_ret[2] = zlim
        self.pre_target_pos_ret[2] = zlim
        
    def check_safety(self):
        ret = True
        ret &= LeftRight_arm.inside_bin_check((self.target_pos[:3]+np.asarray([0,0,20])).tolist(), self.bin)
        return ret
        
    def move(self, grabber):
        grabber.pipe_90_deg()
        print 'pre_target_pos'
        arm_control_wrapper.move_left_arm_local(self.bin, self.pre_target_pos, keitai='fut000', kakujiku='chokusen')
        grabber.vacuum_on()
        print 'approach_target_pos'
        arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
        grabber.vacuum_on()
        print 'target_pos'
        arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos, keitai='fut000', kakujiku='chokusen')
        time.sleep(0.5)
        
        
        force_retry = True if self.item in force_retry_items else False
        # retry 
        if (not grabber.vac_ok(self.item).success) and self.target_pos[2]>40:
            grabber.vacuum_on()
            print 'high pressure -> retry #1'
            grabber.pipe_45_deg()
            arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([20,0,-20,0,0,0]), keitai='fut000', kakujiku='chokusen')
        if (not grabber.vac_ok(self.item).success) and self.target_pos[0]<450:
            print 'high pressure -> retry #2'
            grabber.pipe_90_deg()
            arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([20,0,0,90,3,-90]), keitai='fut000', kakujiku='chokusen')
        if (not grabber.vac_ok(self.item).success):
            print 'high pressure -> retry #3'
            grabber.pipe_90_deg()
            arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([-20,0,0,90,3,-90]), keitai='fut000', kakujiku='chokusen')
        if self.item in ['dasani_water_bottle', 'command_hooks', 'platinum_pets_dog_bowl']:  # transparent items.  (they are sometimes depth deeper)
            if not grabber.vac_ok(self.item).success:
                print 'high pressure -> retry #4'
                grabber.pipe_90_deg()
                arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([-70,0,0,0,0,0]), keitai='fut000', kakujiku='chokusen')
        else:
            if (not grabber.vac_ok(self.item).success) and self.target_pos[0]<400:
                print 'high pressure -> retry #4'
                grabber.pipe_90_deg()
                arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([50,0,0,90,3,-90]), keitai='fut000', kakujiku='chokusen')
        #if not grabber.vac_ok(self.item).success:
        #    print 'high pressure -> retry #5'
        #    grabber.pipe_90_deg()
        #    arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
        #    arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([-50,0,0,0,0,0]), keitai='fut000', kakujiku='chokusen')
        
        
        print 'approach_target_pos_ret'
        arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos_ret, keitai='fut000', kakujiku='chokusen')
        
        print 'pre_target_pos_ret'
        arm_control_wrapper.move_left_arm_local(self.bin, self.pre_target_pos_ret, keitai='fut000', kakujiku='chokusen')
        
        time.sleep(0.5)
        if not grabber.vac_ok(self.item).success:
            rospy.loginfo("pick tried but seemes not success (vac not ok)")
            arm_control_wrapper.move_left_arm_local(self.bin, self.pre_target_pos_ret, keitai='fut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_to_bin(self.bin,'pull', keitai='fut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_to_bin(self.bin,'pre', keitai='nut000', kakujiku='kakujiku')
            if force_retry:
                arm_control_wrapper.move_left_arm_to_bin('bin_K','pre', keitai='nut000', kakujiku='chokusen', speed="fast")
                grabber.vacuum_off()
                arm_control_wrapper.move_left_arm_to_bin('bin_H','pre', keitai='nut000', kakujiku='chokusen', speed="fast")
            else:
                grabber.vacuum_off()
            return False
            
        arm_control_wrapper.move_left_arm_to_bin(self.bin,'pull', keitai='fut000', kakujiku='chokusen')  # TODO kore ha global de ugokashi tain dakedo
        
        return True

class pick_shelf_from_front(motion_base):
    def __init__(self, bin, item, pos):
        
        super(pick_shelf_from_front, self).__init__()
        self.bin = bin
        self.item = item
        
        
        target_pos = np.asarray([0,0,0,0,0,0]).astype('f')
        target_pos[:3] += pos
        target_pos_bin = twist2array(self.t_bin_srv(bin = self.bin, point=array2twist(target_pos)).point)
        target_pos_bin[3:] = np.asarray([90,-90,90]).astype('f')
        target_pos_bin[:3] += np.asarray([-650+10,0, 0]).astype('f')
        self.target_pos              = target_pos_bin + np.asarray([0,0,0,0,0,0])
        self.pre_target_pos          = target_pos_bin + np.asarray([-200,0,80,0,0,0])
        self.approach_target_pos     = target_pos_bin + np.asarray([-100,0,0,0,0,0])
        self.approach_target_pos_ret = target_pos_bin + np.asarray([0,0,80,0,0,0])
        self.pre_target_pos_ret      = target_pos_bin + np.asarray([-200,0,80,0,0,0])
        
        
        # hikidashi ha absolute coordinate
        self.pre_target_pos[0] = -100-650
        self.pre_target_pos_ret[0] = -100-650
        
        # hashira wo yokeru
        if bin in ['bin_A', 'bin_D', 'bin_G', 'bin_J']:
            if target_pos_bin[1]>-130:
                self.pre_target_pos[1] -= 30
                self.approach_target_pos[1] -= 30
                self.approach_target_pos_ret[1] -= 30
                self.pre_target_pos_ret[1] -= 30
        elif bin in ['bin_C', 'bin_F', 'bin_I', 'bin_L']:
            if target_pos_bin[1]<-120:
                self.pre_target_pos[1] += 30
                self.approach_target_pos[1] += 30
                self.approach_target_pos_ret[1] += 30
                self.pre_target_pos_ret[1] += 30
            
        # xlim based on item_size
        item_min_height = util.item_string_to_3side(item)[2]*10
        self.target_pos[0] = min(450-item_min_height-12, self.target_pos[0])
                
        if bin in ['bin_A', 'bin_B', 'bin_C']:
            zlim = 180
        elif bin in ['bin_D', 'bin_E', 'bin_F']:
            zlim = 130
        elif bin in ['bin_G','bin_H','bin_I']:
            zlim = 130
        elif bin in ['bin_J','bin_K','bin_L']:
            zlim = 180
            
        self.pre_target_pos[2] = min(zlim, self.pre_target_pos[2])
        self.approach_target_pos_ret[2] = min(zlim, self.approach_target_pos_ret[2])
        self.pre_target_pos_ret[2] = min(zlim, self.pre_target_pos_ret[2])
        
        if self.target_pos[2] < 18:
            self.target_pos[2] = 18
            self.approach_target_pos[2] = 18
        
        
        
    def check_safety(self):
        ret = True
        #ret &= LeftRight_arm.inside_bin_check(self.approach_target_pos[:3].tolist(), self.bin)
        ret &= LeftRight_arm.inside_bin_check((self.target_pos[:3]+np.asarray([0,0,40])).tolist(), self.bin)
        if self.item=="kleenex_paper_towels":
            return False
        return ret
        
    def move(self, grabber):
        grabber.pipe_0_deg()
        print 'pre_target_pos'
        arm_control_wrapper.move_left_arm_local(self.bin, self.pre_target_pos, keitai='fut000', kakujiku='chokusen')
        grabber.vacuum_on()
        print 'approach_target_pos'
        arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
        grabber.vacuum_on()
        print 'target_pos'
        arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos, keitai='fut000', kakujiku='chokusen')
        time.sleep(0.5)
        
        force_retry = True if self.item in force_retry_items else False
        # retry 
        if (not grabber.vac_ok(self.item).success) and self.target_pos[2]>10:
            grabber.vacuum_on()
            print 'high pressure -> retry #1'
            grabber.pipe_45_deg()
            arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([20,0,-18,0,0,0]), keitai='fut000', kakujiku='chokusen')
        if (not grabber.vac_ok(self.item).success) and self.target_pos[0]<430:
            print 'high pressure -> retry #2'
            grabber.pipe_0_deg()
            arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([30,0,0,0,0,0]), keitai='fut000', kakujiku='chokusen')
            
        print 'approach_target_pos_ret'
        arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos_ret, keitai='fut000', kakujiku='chokusen')
        
            
        print 'pre_target_pos_ret'
        arm_control_wrapper.move_left_arm_local(self.bin, self.pre_target_pos_ret, keitai='fut000', kakujiku='chokusen')
        
        time.sleep(0.5)
        if not grabber.vac_ok(self.item).success:
            rospy.loginfo("pick tried but seemes not success (vac not ok)")
            arm_control_wrapper.move_left_arm_local(self.bin, self.pre_target_pos_ret, keitai='fut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_to_bin(self.bin,'pull', keitai='fut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_to_bin(self.bin,'pre', keitai='nut000', kakujiku='kakujiku')
            if force_retry:
                arm_control_wrapper.move_left_arm_to_bin('bin_K','pre', keitai='nut000', kakujiku='chokusen', speed="fast")
                grabber.vacuum_off()
                arm_control_wrapper.move_left_arm_to_bin('bin_H','pre', keitai='nut000', kakujiku='chokusen', speed="fast")
            else:
                grabber.vacuum_off()
            return False
            
        arm_control_wrapper.move_left_arm_to_bin(self.bin,'pull', keitai='fut000', kakujiku='chokusen')  # TODO kore ha global de ugokashi tain dakedo
        
        return True



class pick_shelf_from_side(motion_base):
    def __init__(self, bin, item, pos, lr):
        
        super(pick_shelf_from_side, self).__init__()
        self.bin = bin
        self.item = item
        self.lr = lr
        
        target_pos = np.asarray([0,0,0,0,0,0]).astype('f')
        target_pos[:3] += pos
        target_pos_bin = twist2array(self.t_bin_srv(bin = self.bin, point=array2twist(target_pos)).point)
        if lr=='left':
            target_pos_bin[3:] = np.asarray([90,0,90]).astype('f')
            if bin in ['bin_J']:
                target_pos_bin[3:] = np.asarray([90,-20,90]).astype('f')
            target_pos_bin[:3] += np.asarray([-650,40, 0]).astype('f')
            self.target_pos              = target_pos_bin + np.asarray([0,0,0,0,0,0])
            self.pre_target_pos          = target_pos_bin + np.asarray([-200,50,50,0,0,0])
            self.approach_target_pos     = target_pos_bin + np.asarray([   0,50,50,0,0,0])
            self.approach_target_pos_ret = target_pos_bin + np.asarray([   0,50,50,0,0,0])
            self.pre_target_pos_ret      = target_pos_bin + np.asarray([-200,50,50,0,0,0])
        elif lr=='right':
            target_pos_bin[3:] = np.asarray([90,-180,90]).astype('f')
            target_pos_bin[:3] += np.asarray([-650,-40, 0]).astype('f')
            self.target_pos              = target_pos_bin + np.asarray([0,0,0,0,0,0])
            self.pre_target_pos          = target_pos_bin + np.asarray([-200,-50,50,0,0,0])
            self.approach_target_pos     = target_pos_bin + np.asarray([   0,-50,50,0,0,0])
            self.approach_target_pos_ret = target_pos_bin + np.asarray([   0,-50,50,0,0,0])
            self.pre_target_pos_ret      = target_pos_bin + np.asarray([-200,-50,50,0,0,0])
        
        # hikidashi ha absolute coordinate
        self.pre_target_pos[0] = -100-650
        self.pre_target_pos_ret[0] = -100-650
        
        # hashira wo yokeru
        if bin in ['bin_A', 'bin_D', 'bin_G', 'bin_J']:
            if target_pos_bin[1]>-130:
                self.approach_target_pos[1] -= 20
                self.approach_target_pos_ret[1] -= 20
                self.pre_target_pos_ret[1] -= 20
        elif bin in ['bin_C', 'bin_F', 'bin_I', 'bin_L']:
            if target_pos_bin[1]<-120:
                self.approach_target_pos[1] += 20
                self.approach_target_pos_ret[1] += 20
                self.pre_target_pos_ret[1] += 20
        
    def check_safety(self):
        ret = True
        #ret &= LeftRight_arm.inside_bin_check(self.approach_target_pos[:3].tolist(), self.bin)
        offset = np.asarray([0,0,0]).astype("f")
        if self.bin in ['A', 'D', 'G', 'J'] and self.target_pos[:2]>-200:
            offset[1] = -20
        if self.bin in ['C', 'F', 'I', 'L'] and self.target_pos[:2]<-100:
            offset[1] = 20
        ret &= LeftRight_arm.inside_bin_check((self.approach_target_pos_ret[:3]+offset).tolist(), self.bin)
        ret &= LeftRight_arm.inside_bin_check((self.target_pos[:3]+offset).tolist(), self.bin)
        if self.item in ["kleenex_paper_towels", "dasani_water_bottle"]:
            return False
        return ret
        
    def move(self, grabber):
        grabber.pipe_90_deg()
        print 'pre_target_pos'
        arm_control_wrapper.move_left_arm_local(self.bin, self.pre_target_pos, keitai='fut000', kakujiku='chokusen')
        grabber.vacuum_on()
        print 'approach_target_pos'
        arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
        grabber.vacuum_on()
        print 'target_pos'
        arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos, keitai='fut000', kakujiku='chokusen')
        time.sleep(0.5)
        
        print 'approach_target_pos_ret'
        arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos_ret, keitai='fut000', kakujiku='chokusen')
        
        force_retry = True if self.item in force_retry_items else False
        time.sleep(0.5)
        if (not grabber.vac_ok(self.item).success):
            grabber.vacuum_on()
            print 'high pressure -> retry #1'
            if self.lr=='left':
                arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([0,-10,0,0,0,0]), keitai='fut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
            else:
                arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([0,10,0,0,0,0]), keitai='fut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
        if (not grabber.vac_ok(self.item).success):
            print 'high pressure -> retry #2'
            if self.lr=='left':
                arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([20,-10,0,0,0,0]), keitai='fut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
            else:
                arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([20,10,0,0,0,0]), keitai='fut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
        if (not grabber.vac_ok(self.item).success):
            print 'high pressure -> retry #3'
            if self.lr=='left':
                arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([-20,-10,0,0,0,0]), keitai='fut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
            else:
                arm_control_wrapper.move_left_arm_local(self.bin, self.target_pos + np.asarray([-20,10,0,0,0,0]), keitai='fut000', kakujiku='chokusen')
                arm_control_wrapper.move_left_arm_local(self.bin, self.approach_target_pos, keitai='fut000', kakujiku='chokusen')
               
          
        print 'pre_target_pos_ret'
        arm_control_wrapper.move_left_arm_local(self.bin, self.pre_target_pos_ret, keitai='fut000', kakujiku='chokusen')
        
        
        time.sleep(0.5) 
        if not grabber.vac_ok(self.item).success:
            rospy.loginfo("pick tried but seemes not success (vac not ok)")
            arm_control_wrapper.move_left_arm_local(self.bin, self.pre_target_pos_ret, keitai='fut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_to_bin(self.bin,'pull', keitai='fut000', kakujiku='chokusen')
            arm_control_wrapper.move_left_arm_to_bin(self.bin,'pre', keitai='nut000', kakujiku='kakujiku')
            if force_retry:
                arm_control_wrapper.move_left_arm_to_bin('bin_K','pre', keitai='nut000', kakujiku='chokusen', speed="fast")
                grabber.vacuum_off()
                arm_control_wrapper.move_left_arm_to_bin('bin_H','pre', keitai='nut000', kakujiku='chokusen', speed="fast")
            else:
                grabber.vacuum_off()
            return False
        
        arm_control_wrapper.move_left_arm_to_bin(self.bin,'pull', keitai='fut000', kakujiku='chokusen')  # TODO kore ha global de ugokashi tain dakedo
        
        return True

