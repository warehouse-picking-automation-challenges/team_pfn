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
import threading
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
from snapshot import Snapshot

import util
from util import camera_to_global, item_string_to_id, camera_to_global_all, item_id_to_string
from grab.vacuum_imgproc import VacImgProc, gaussian_interpolation, get_normal_image, decompose_into_regions
import grab.vacuum_strategies
import grab.vacuum_tote_strategies

import chainer
from chainer import cuda
from chainer import serializers
from chainer import Variable
import chainer.functions as F
import chainer.links as L
import cv2

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
    
class VacuumGrabbing(interface.GrabStrategy):
    """VacuumGrabbing uses the left hand and the attached pipe
    to grab an item via suction."""
    
    def __init__(self, vision_only=False):
        # vacuum action & service
        self.vision_only = vision_only
        if not vision_only:
            self.vac_client = actionlib.SimpleActionClient('vacuum_switch', VacuumAction)
            self.vac_client.wait_for_server()
            rospy.wait_for_service('vacuum_check')
            #rospy.wait_for_service('vacuum_check_high')
            self.vac_ok = rospy.ServiceProxy('vacuum_check', ItemName2Success)
            #self.vac_high = rospy.ServiceProxy('vacuum_check_high', ItemName2Success)
            rospy.wait_for_service('touch_check')
            self.touch_ok = rospy.ServiceProxy('touch_check', Success)
            rospy.loginfo("vac action ok")
            # pipe action
            self.pipe_client = actionlib.SimpleActionClient('turn_pipe', GenericAction)
            self.pipe_client.wait_for_server()
            rospy.loginfo("pipe action ok")
        self.segm_service_left = rospy.ServiceProxy('img_segm_left',
                                           Segmentation)
        self.vacimgproc = VacImgProc()
        self.current_candidates = None
        self.bridge = CvBridge()
        
        # for visualization
        self.str_map = np.zeros((240,320)).astype('i')
        self.reject_map = np.zeros((240,320)).astype('i')
        self.rgb_map = None
        self.depth_map = None
        self.score_map = None
        self.toru_map = None
        self.selected_candidate = None
        self.strategies = None
        self.strategy_cache = {}
        self.final_mode = {}
        for b in ["bin_A", "bin_B", "bin_C", "bin_D", "bin_E", "bin_F", "bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L"]:
            self.strategy_cache[b] = []
            self.final_mode[b] = False
        
        self.blacklist = []
        
        self.log_cnt = 0
        log_files = os.listdir('log_imgs')
        for f in log_files:
            if 'jpg' in f:
                self.log_cnt = max(self.log_cnt, int(f[4:-4])+1)
    
    def pipe_0_deg(self):
        if not self.vision_only:
            pipe_goal = GenericGoal()
            pipe_goal.target = "1"
            self.pipe_client.send_goal(pipe_goal)
            self.pipe_client.wait_for_result()
            result = self.pipe_client.get_result()
            return result

    def pipe_0_deg_slow(self, wait=False):
        if not self.vision_only:
            pipe_goal = GenericGoal()
            pipe_goal.target = "4"
            self.pipe_client.send_goal(pipe_goal)
            if wait:
                self.pipe_client.wait_for_result()
                result = self.pipe_client.get_result()
                return result
            else:
                return None

    def pipe_0_deg_very_slow(self, wait=False):
        if not self.vision_only:
            pipe_goal = GenericGoal()
            pipe_goal.target = "6"
            self.pipe_client.send_goal(pipe_goal)
            if wait:
                self.pipe_client.wait_for_result()
                result = self.pipe_client.get_result()
                return result
            else:
                return None

    def pipe_45_deg(self):
        if not self.vision_only:
            pipe_goal = GenericGoal()
            pipe_goal.target = "2"
            self.pipe_client.send_goal(pipe_goal)
            self.pipe_client.wait_for_result()
            result = self.pipe_client.get_result()
            return result

    def pipe_90_deg(self):
        if not self.vision_only:
            pipe_goal = GenericGoal()
            pipe_goal.target = "0"
            self.pipe_client.send_goal(pipe_goal)
            self.pipe_client.wait_for_result()
            result = self.pipe_client.get_result()
            return result

    def pipe_90_deg_slow(self, wait=False):
        if not self.vision_only:
            pipe_goal = GenericGoal()
            pipe_goal.target = "3"
            self.pipe_client.send_goal(pipe_goal)
            if wait:
                self.pipe_client.wait_for_result()
                result = self.pipe_client.get_result()
                return result
            else:
                return None

    def pipe_90_deg_very_slow(self, wait=False):
        if not self.vision_only:
            pipe_goal = GenericGoal()
            pipe_goal.target = "5"
            self.pipe_client.send_goal(pipe_goal)
            if wait:
                self.pipe_client.wait_for_result()
                result = self.pipe_client.get_result()
                return result
            else:
                return None

    # to grab the object
    def vacuum_on(self):
        if not self.vision_only:
            vac_goal = VacuumGoal()
            vac_goal.status = "vacuum"
            self.vac_client.send_goal(vac_goal)
            self.vac_client.wait_for_result(rospy.Duration.from_sec(10.0))
            result = self.vac_client.get_result()
            return result

    # to put the object
    def vacuum_off(self):
        if not self.vision_only:
            vac_goal = VacuumGoal()
            vac_goal.status = "off"
            self.vac_client.send_goal(vac_goal)
            self.vac_client.wait_for_result(rospy.Duration.from_sec(10.0))
            result = self.vac_client.get_result()
            return result

    # normal condition
    def vacuum_normal(self):
        if not self.vision_only:
            vac_goal = VacuumGoal()
            vac_goal.status = "normal"
            self.vac_client.send_goal(vac_goal)
            self.vac_client.wait_for_result(rospy.Duration.from_sec(10.0))
            result = self.vac_client.get_result()
            return result

    def grab_from_tote(self, item):
        # TODO: implement this
        print "trying to grab item \"%s\" from tote" % \
            item
            
        # move pre->front->item_front->item_adj
        arm_control_wrapper.move_left_arm_to_bin('tote','', keitai='nut000', kakujiku='kakujiku', speed="fast")  # TODO 'tote,'' de yoi?
        #arm_control_wrapper.move_left_arm_to_bin(bin,'front', keitai='fut000', kakujiku='kakujiku')
        
        # pick!
        strategy = self.selected_strategy
        self.vacuum_on()
        success = strategy['motion'].move(self)  # kokode ugoki masu
        
        # back to pre
        #arm_control_wrapper.move_left_arm_to_bin(bin,'pre')
        if success:
            arm_control_wrapper.move_left_arm_to_bin('bin_G','rpre', keitai='nut000', kakujiku='chokusen', speed="fast")
            self.pipe_90_deg_slow()
            arm_control_wrapper.move_left_arm_to_bin('bin_G','pull', keitai='fut000', kakujiku='kakujiku')
            #arm_control_wrapper.move_left_arm_to_bin('tote','photo', keitai='fut000', kakujiku='kakujiku')
        else:
            arm_control_wrapper.move_left_arm_to_bin('tote','photo', keitai='fut000', kakujiku='kakujiku', speed="fast")
            #arm_control_wrapper.move_left_arm_to_bin('tote','photo', keitai='fut000', kakujiku='kakujiku', speed="fast")
        
        #self.output_log()
        self.strategies = None
        self.selected_strategy = None
                    
        self.selected_mode = ""
        self.selected_strategy = None
        self.selected_item = ""
        return success

    def stow_in_tote(self, tote_location, item=""):
        tote_location[2] += 0   # tote top 
        print "go to tote"
        # move to tote_location
        if item in "kleenex_paper_towels":
            arm_control_wrapper.move_left_arm_to_bin('bin_H','pull', keitai='fut000', kakujiku='chokusen', speed="fast")
        else:
            arm_control_wrapper.move_left_arm_to_bin('bin_H','pull', keitai='fut000', kakujiku='chokusen', speed="fast")
        
        if not self.vac_ok(item).success:
            rospy.loginfo("stow tried but seemes not success (vac not ok)")
            self.vacuum_off()
            arm_control_wrapper.move_left_arm_to_bin('bin_H','pre', keitai='nut000', kakujiku='kakujiku', speed="fast")
            self.vacuum_normal()
            self.pipe_0_deg()
            return False
        
        self.pipe_0_deg_very_slow()
        if item in "kleenex_paper_towels":
            arm_control_wrapper.move_left_arm_to_bin('bin_H','pre', keitai='nut000', kakujiku='kakujiku')
        else:
            arm_control_wrapper.move_left_arm_to_bin('bin_H','pre', keitai='nut000', kakujiku='kakujiku')
        result = arm_control_wrapper.move_left_arm_global(tote_location, keitai='nut000', kakujiku='chokusen', speed="fast")
        print "vacuum off"
        # drop the object
        self.vacuum_off()
        #time.sleep(3)
        #self.vacuum_normal()
        if result == False:
            return False
        if item in util.picopico:
            self.pipe_90_deg()
            self.pipe_0_deg()
            self.pipe_90_deg()
            self.pipe_0_deg()
            self.pipe_90_deg()
        self.pipe_0_deg()
        arm_control_wrapper.move_left_arm_to_bin('bin_H','pre', keitai='nut000', kakujiku='chokusen', speed="fast")
        return True
        
        
    def stow_in_shelf(self, bin, pos, item=""):
        print "trying to stow item to  \"%s\"" % \
            (bin)
        
        
        if not self.vac_ok(self.selected_item).success:
            rospy.loginfo("stow tried but seemes not success (vac not ok)")
            self.vacuum_off()
            arm_control_wrapper.move_left_arm_to_bin(bin,'pull', keitai='fut000', kakujiku='chokusen', speed="fast")
            #arm_control_wrapper.move_left_arm_to_bin(bin,'pre', keitai='nut000', kakujiku='kakujiku', speed="fast")
            self.vacuum_normal()
            self.pipe_0_deg()
            return False
            
        arm_control_wrapper.move_left_arm_to_bin(bin,'pull', keitai='fut000', kakujiku='chokusen', speed="fast")
          
        if not self.vac_ok(item).success:
            rospy.loginfo("stow tried but seemes not success (vac not ok)")
            self.vacuum_off()
            arm_control_wrapper.move_left_arm_to_bin(bin,'pull', keitai='fut000', kakujiku='chokusen', speed="fast")
            #arm_control_wrapper.move_left_arm_to_bin(bin,'pre', keitai='nut000', kakujiku='kakujiku', speed="fast")
            self.vacuum_normal()
            self.pipe_0_deg()
            return False
            
            
        self.pipe_90_deg_slow()
        
        # stow!
        
        if bin in ['bin_A', 'bin_B', 'bin_C']:
            zlim = 200
        elif bin in ['bin_D', 'bin_E', 'bin_F']:
            zlim = 160
        elif bin in ['bin_G','bin_H','bin_I']:
            zlim = 160
        elif bin in ['bin_J','bin_K','bin_L']:
            zlim = 200
        #arm_control_wrapper.move_left_arm_local(bin, np.asarray([-150-600,-120,zlim,90,-90,90]).astype('f'), keitai='fut000', kakujiku='chokusen')
        arm_control_wrapper.move_left_arm_local(bin, np.asarray([200-600,-120,zlim,90,-90,90]).astype('f'), keitai='fut000', kakujiku='chokusen')
        
      
        
        self.vacuum_off()
        #time.sleep(3)
        #self.vacuum_normal()
        
        if item in util.picopico:
            self.pipe_0_deg()
            self.pipe_90_deg()
            self.pipe_0_deg()
            self.pipe_90_deg()
        self.pipe_0_deg()
        
        # back to pre
        arm_control_wrapper.move_left_arm_to_bin(bin,'pull', keitai='fut000', kakujiku='chokusen', speed="fast")
        #arm_control_wrapper.move_left_arm_to_bin(bin,'pre', keitai='nut000', kakujiku='kakujiku', speed="fast")
        
        return True
        
    # self.strategies
    #   move
    def grab_from_shelf(self, item, bin):
        item_id = item_string_to_id(item)
        print "trying to grab item \"%s\" (%d) from \"%s\"" % \
            (item, item_id, bin)
        
        # move pre->front->item_front->item_adj
        arm_control_wrapper.move_left_arm_to_bin(bin,'pre', keitai='nut000', kakujiku='chokusen', speed="fast")
        arm_control_wrapper.move_left_arm_to_bin(bin,'front', keitai='fut000', kakujiku='kakujiku', speed="fast")
        
        # pick!
        strategy = self.selected_strategy
        success = strategy['motion'].move(self)  # kokode ugoki masu
        self.blacklist.append(strategy)
        
        # back to pre
        #arm_control_wrapper.move_left_arm_to_bin(bin,'pre')
        
        #self.output_log()
        #self.strategies = None
        self.strategy_cache[bin] = []
        self.selected_strategy = None
                    
        self.selected_mode = ""
        self.selected_strategy = None
        self.selected_item = ""
        return success
        
    def get_strategy_bin(self, bin, item, region, xyz, normal, normal_rs, score_map, toru, photo_dir):
        nuno = [ 
            "womens_knit_gloves",
            "cherokee_easy_tee_shirt",
            "cloud_b_plush_bear",
            "cool_shot_glue_sticks",
            
            "fiskars_scissors_red",
            "oral_b_toothbrush_green",
            "oral_b_toothbrush_red"
        ]
        if( item in nuno ):
            strategies = grab.vacuum_strategies.nuno_strategy(bin, item, region, xyz, normal, normal_rs, score_map, toru, photo_dir)
        else:
            strategies = grab.vacuum_strategies.basic_strategy(bin, item, region, xyz, normal, normal_rs, score_map, toru, photo_dir)
        return strategies
        
    def get_strategy_tote(self, bin, item, region, xyz, normal, score_map, toru, photo_dir):
        
        # TODO different strategy for different situation
        strategies = grab.vacuum_tote_strategies.basic_strategy_tote(bin, item, region, xyz, normal, score_map, toru, photo_dir)
        return strategies
        for i,s in enumerate(strategies):
            grab.vacuum_strategies.logging_strategy(f, s, 'log_imgs/%s_%d.png'%(self.log_cnt, i))
        
    
    # from raw images to strategies
    def pick_imgproc_bin(self, bin, resp, photo_pos, items_in_bin, photo_dir):
    
        cv_image = self.bridge.imgmsg_to_cv2(resp.realsense_rgb_img, "bgr8")
        cam_rgb = cv2.resize(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB), (320,240))
        
        fx8_x = (np.asarray(self.bridge.imgmsg_to_cv2(resp.fx8_x_img, "mono16")).astype('f')-30000).reshape((480,640))
        fx8_y = (np.asarray(self.bridge.imgmsg_to_cv2(resp.fx8_y_img, "mono16")).astype('f')-30000).reshape((480,640))
        fx8_z = (np.asarray(self.bridge.imgmsg_to_cv2(resp.fx8_z_img, "mono16")).astype('f')-30000).reshape((480,640))
        fx8_invalid = ((fx8_x==0).astype('i') * (fx8_y==0).astype('i') * (fx8_z==0).astype('i'))
        fx8_shift = np.zeros((3,480,640)).astype('f')
        fx8_shift[0,:480,:] = fx8_x[:,:640]
        fx8_shift[1,:480,:] = fx8_y[:,:640]
        fx8_shift[2,:480,:] = fx8_z[:,:640]
        fx8_invalid_shift = np.zeros((1,480,640)).astype('f')
        #print fx8_shift.shape, fx8_invalid_shift.shape
        fx8_invalid_shift[0,:480,:] = fx8_invalid[:,:640]
        #print fx8_x.shape, fx8_invalid.shape
        xyz_global = ((-1000000*fx8_invalid_shift) + camera_to_global_all(fx8_shift, photo_pos)).astype('f')  # (3, 480, 640) -> (3, 480, 640)
        
        realsense_x = (np.asarray(self.bridge.imgmsg_to_cv2(resp.realsense_x_img, "mono16")).astype('f')-30000).reshape((480,640)) * 1.4
        realsense_y = (np.asarray(self.bridge.imgmsg_to_cv2(resp.realsense_y_img, "mono16")).astype('f')-30000).reshape((480,640)) * 1.4
        realsense_z = (np.asarray(self.bridge.imgmsg_to_cv2(resp.realsense_z_img, "mono16")).astype('f')-30000).reshape((480,640)) * 1.4
        realsense_invalid = ((realsense_x==0).astype('i') * (realsense_y==0).astype('i') * (realsense_z==0).astype('i'))
        #print realsense_x.shape, realsense_invalid.shape
        xyz_global_rs = ((-1000000*realsense_invalid) + camera_to_global_all(np.asarray([realsense_x,realsense_y,realsense_z]), photo_pos)).astype('f')[:,::2,::2]  # (3, 480, 640) -> (3, 240640)
        
        print xyz_global.shape
        
        # probs
        score_map = np.asarray(resp.score_map).reshape((1,40,240,320))
        for i in range(1,40):
            if not item_id_to_string(i) in items_in_bin:
                score_map[0,i,:,:] -= 20.0
        # toruichi
        toru = np.asarray(resp.toru_map).reshape((1,1,240,320))
        
        
        # bin with small number of items hosei #1
        if len(items_in_bin)<=2:
            score_map[0,0,:,:] -= 0.2
            toru += 0.2
        # bin with small number of items hosei #2
        elif len(items_in_bin)<=3:
            score_map[0,0,:,:] -= 0.1
            toru += 0.1
        
        
        # depth
        # TODO better downscale
        
        t_start = time.time()
        xyz_global_interpolated = gaussian_interpolation(xyz_global)[:,::2,::2]  # (3, 480, 640) -> (3, 240, 320)
        self.xyz = xyz_global_interpolated
        # normal
        
        t_start1 = time.time()
        normal_map = get_normal_image(xyz_global_interpolated)
        
        t_start2 = time.time()
        normal_map_rs = get_normal_image(xyz_global_rs)
        elapsed_time = time.time() - t_start2
        
        print ("(-_-) elapsed_time: {0} {1} {2} [sec]".format(t_start1-t_start, t_start2-t_start1, elapsed_time))
        
        # regions
        toru_with_segm = score_map.copy()
        toru_with_segm[0,0,:,:] += ( - toru[0,0,:,:])  # this means low toru score is treated as background
        t_start = time.time()
        self.regions_for_items = decompose_into_regions(np.argmax(toru_with_segm, axis=1)[0,:])
        elapsed_time = time.time() - t_start
        print ("(^_^) elapsed_time: {0} [sec]".format(elapsed_time))
        
        for key,region in self.regions_for_items.items():
            n_regions = region['n_regions']
            mx = 10000
            for i in range(n_regions):
                cx = int(region['centroid'][i][0])
                cy = int(region['centroid'][i][1])
                self.item_coords[region["item_id"]].append(xyz_global_interpolated[:,cy,cx])
                
        # get strategies for each region
        strategies = []
        for key,val in self.regions_for_items.items():
            strategies += self.get_strategy_bin(bin, item_id_to_string(val['item_id']), val, xyz_global_interpolated, normal_map, normal_map_rs, score_map, toru, photo_dir)
        
        # TODO better sort
        # sort by x-axis
        def strategy_cmp(x, y):
            if not x['is_ok']:
                return 1
            if not y['is_ok']:
                return -1
            return 1 if x['toru'] < y['toru'] else -1
            #return 1 if x['pos'][0] > y['pos'][0] else -1
        strategies.sort(cmp=strategy_cmp)
        
        xs = []
        ys = []
        xs_rejected = []
        ys_rejected = []
        rs_rejected = []
        for s in strategies:
            if s['is_ok']:
                xs.append(s['px'])
                ys.append(s['py'])
            else:
                xs_rejected.append(s['px'])
                ys_rejected.append(s['py'])
                if s['rejected_reason'] == 'invalid direction':
                    rs_rejected.append(0)
                elif s['rejected_reason'] == 'normal instability':
                    rs_rejected.append(1)
                elif s['rejected_reason'] == 'collide with shelf':
                    rs_rejected.append(2)
        xs_rejected = np.asarray(xs_rejected)
        ys_rejected = np.asarray(ys_rejected)
        rs_rejected = np.asarray(rs_rejected)
        
        # TODO logging
        f = open("./log_txt/log_items_%d.txt"%self.log_cnt, "w")
        for i in items_in_bin:
            f.write(str(i)+"\n")
        f.close()
        self.output_log(cam_rgb, score_map, toru, xyz_global_interpolated, xyz_global_rs, normal_map, normal_map_rs, xs, ys, xs_rejected, ys_rejected, rs_rejected)
        f = open("./log_txt/log_candidates_%d.txt"%self.log_cnt,"w")
        #f.write('\n\n\n')
        for i,s in enumerate(strategies):
            if i<10:
                grab.vacuum_strategies.logging_strategy(f, s, 'log_imgs/%s_%d.png'%(self.log_cnt, i), out_img=False)
            else:
                grab.vacuum_strategies.logging_strategy(f, s, 'log_imgs/%s_%d.png'%(self.log_cnt, i), out_img=False)
        f.close()
        #self.log_cnt += 1
        #f.write(str(
        
        return strategies
        
    
    # from raw images to strategies
    def pick_imgproc_tote(self, bin, resp, photo_pos, items_in_bin, photo_dir):
    
        cv_image = self.bridge.imgmsg_to_cv2(resp.realsense_rgb_img, "bgr8")
        cam_rgb = cv2.resize(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB), (320,240))
        
        fx8_x = (np.asarray(self.bridge.imgmsg_to_cv2(resp.fx8_x_img, "mono16")).astype('f')-30000).reshape((480,640))
        fx8_y = (np.asarray(self.bridge.imgmsg_to_cv2(resp.fx8_y_img, "mono16")).astype('f')-30000).reshape((480,640))
        fx8_z = (np.asarray(self.bridge.imgmsg_to_cv2(resp.fx8_z_img, "mono16")).astype('f')-30000).reshape((480,640))
        fx8_invalid = ((fx8_x==0).astype('i') * (fx8_y==0).astype('i') * (fx8_z==0).astype('i'))
        print fx8_x.shape, fx8_invalid.shape
        xyz_global = ((-1000000*fx8_invalid) + camera_to_global_all(np.asarray([fx8_x,fx8_y,fx8_z]), photo_pos)).astype('f')  # (3, 480, 640) -> (3, 480, 640)
        
        print xyz_global.shape
        
        
        realsense_x = (np.asarray(self.bridge.imgmsg_to_cv2(resp.realsense_x_img, "mono16")).astype('f')-30000).reshape((480,640)) * 1.4
        realsense_y = (np.asarray(self.bridge.imgmsg_to_cv2(resp.realsense_y_img, "mono16")).astype('f')-30000).reshape((480,640)) * 1.4
        realsense_z = (np.asarray(self.bridge.imgmsg_to_cv2(resp.realsense_z_img, "mono16")).astype('f')-30000).reshape((480,640)) * 1.4
        realsense_invalid = ((realsense_x==0).astype('i') * (realsense_y==0).astype('i') * (realsense_z==0).astype('i'))
        #print realsense_x.shape, realsense_invalid.shape
        xyz_global_rs = ((-1000000*realsense_invalid) + camera_to_global_all(np.asarray([realsense_x,realsense_y,realsense_z]), photo_pos)).astype('f')[:,::2,::2]  # (3, 480, 640) -> (3, 240640)
        
        
        # probs
        score_map = np.asarray(resp.score_map).reshape((1,40,240,320))
        for i in range(1,40):
            if not item_id_to_string(i) in items_in_bin:
                score_map[0,i,:,:] -= 20.0
        # toruichi
        toru = np.asarray(resp.toru_map).reshape((1,1,240,320))
        
        # bin with small number of items hosei
        if len(items_in_bin)<=2:
            score_map[0,0,:,:] -= 0.2
            toru += 0.2
        # duct tape penalty
        score_map[0,33,:,:] -= 0.2
        
        
        # depth
        # TODO better downscale
        xyz_global_interpolated = gaussian_interpolation(xyz_global)[:,::2,::2]  # (3, 480, 640) -> (3, 240, 320)
        # normal
        normal_map = get_normal_image(xyz_global_interpolated)
        normal_map_rs = get_normal_image(xyz_global_rs)
        
        # regions
        toru_with_segm = score_map.copy()
        toru_with_segm[0,0,:,:] += ( - toru[0,0,:,:])  # this means low toru score is treated as background
        self.regions_for_items = decompose_into_regions(np.argmax(toru_with_segm, axis=1)[0,:])
        
        
        for key,region in self.regions_for_items.items():
            n_regions = region['n_regions']
            mx = 10000
            for i in range(n_regions):
                cx = int(region['centroid'][i][0])
                cy = int(region['centroid'][i][1])
                self.item_coords[region["item_id"]].append(xyz_global_interpolated[:,cy,cx])
                
        # get strategies for each region
        strategies = []
        for key,val in self.regions_for_items.items():
            strategies += self.get_strategy_tote(bin, item_id_to_string(val['item_id']), val, xyz_global_interpolated, normal_map, score_map, toru, photo_dir)
        
        # TODO better sort
        # sort by x-axis
        def strategy_cmp(x, y):
            if not x['is_ok']:
                return 1
            if not y['is_ok']:
                return -1
            return 1 if x['toru'] < y['toru'] else -1
            #return 1 if x['pos'][0] > y['pos'][0] else -1
        strategies.sort(cmp=strategy_cmp)
        
        
        xs = []
        ys = []
        xs_rejected = []
        ys_rejected = []
        rs_rejected = []
        for s in strategies:
            if s['is_ok']:
                xs.append(s['px'])
                ys.append(s['py'])
            else:
                xs_rejected.append(s['px'])
                ys_rejected.append(s['py'])
                if s['rejected_reason'] == 'invalid direction':
                    rs_rejected.append(0)
                elif s['rejected_reason'] == 'normal instability':
                    rs_rejected.append(1)
                elif s['rejected_reason'] == 'collide with shelf':
                    rs_rejected.append(2)
                else:
                    rs_rejected.append(3)
        xs_rejected = np.asarray(xs_rejected)
        ys_rejected = np.asarray(ys_rejected)
        rs_rejected = np.asarray(rs_rejected)
        
        # TODO logging
        self.output_log(cam_rgb, score_map, toru, xyz_global_interpolated, xyz_global_rs, normal_map, normal_map_rs, xs, ys, xs_rejected, ys_rejected, rs_rejected)
        f = open("./log_tote.txt","a")
        f.write('\n\n\n')
        for i,s in enumerate(strategies):
            grab.vacuum_tote_strategies.logging_strategy(f, s, 'log_imgs/%s_%d.png'%(self.log_cnt, i))
            if i>25:
                break
        f.close()
        #self.log_cnt += 1
        #f.write(str(
        
        return strategies
        
        
        
    # calc candidates
    def can_grab_item(self, item, bin, items_in_bin):
        self.vacuum_normal()
        if bin=='tote':
            return self.can_grab_item_tote(item, bin, items_in_bin)
        else:
            return self.can_grab_item_bin(item, bin, items_in_bin)
        
    def can_grab_item_bin(self, item, bin, items_in_bin):
        item_id = item_string_to_id(item)
        
        # move to pre for avoiding over-rotation(J6)
        #rospy.loginfo('move to pre for avoiding over-rotation(J6)')
        #######################arm_control_wrapper.move_left_arm_to_bin(bin,'pre')
        
        
        for b in ["bin_A", "bin_B", "bin_C", "bin_D", "bin_E", "bin_F", "bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L"]:
            print b, len(self.strategy_cache[b])
        
        self.strategies = self.strategy_cache[bin]
        if len(self.strategies)==0:        # TODO reuse previous result (caution: this is dangerous)
            self.strategies = []
            self.selected_mode = ""
            self.selected_strategy = None
            self.selected_item = item
            self.prev_bin = bin
            
            # move to photo
            rospy.loginfo('move to photo')
            
            arm_control_wrapper.move_left_arm_to_bin(bin,'pre', keitai='nut000', kakujiku='kakujiku')
            
            # take photo from front
            (_,_,_,self.photo_pos) = arm_control_wrapper.move_left_arm_to_bin(bin,'photo', keitai='nut000', kakujiku='chokusen', speed="fast")
            time.sleep(2.0) # wait for camera delay
            # take snapshot and imgprocs
            rospy.loginfo('take snapshot and imgprocs')
            resp = self.segm_service_left(['a'])
            
            self.item_coords = [[] for i in range(40)]
            
            def th_imgproc(self, resp, bin, items_in_bin, photo_dir):
                print 'imgproc start'
                Snapshot.save_snapshot(resp, str(self.log_cnt), 'log_imgs_raw')
                self.strategies += self.pick_imgproc_bin(bin, resp, self.photo_pos, items_in_bin, photo_dir=photo_dir)
                print 'imgproc end'
                
            def th_movearm(self, bin, photo_dir):
                print 'movearm start'
                (_,_,_,self.next_photo_pos) = arm_control_wrapper.move_left_arm_to_bin(bin, photo_dir, keitai='nut000', kakujiku='chokusen', speed="fast")
                time.sleep(2.0) # wait for camera delay
                print 'movearm end'
                
            
            th1 = threading.Thread(target=th_imgproc, name="th1", args=(self, resp, bin, items_in_bin, 'center'))
            th2 = threading.Thread(target=th_movearm, name="th2", args=(self, bin, 'photo_l'))
            th1.start()
            th2.start()
            th1.join()
            th2.join()
            self.photo_pos = self.next_photo_pos
            self.log_cnt += 1
            
            
            # take snapshot and imgprocs
            resp = self.segm_service_left(['a'])
            
            th1 = threading.Thread(target=th_imgproc, name="th1", args=(self, resp, bin, items_in_bin, 'left'))
            th2 = threading.Thread(target=th_movearm, name="th2", args=(self, bin, 'photo_r'))
            th1.start()
            th2.start()
            th1.join()
            th2.join()
            self.photo_pos = self.next_photo_pos
            self.log_cnt += 1
            
            resp = self.segm_service_left(['a'])
            
            th1 = threading.Thread(target=th_imgproc, name="th1", args=(self, resp, bin, items_in_bin, 'right'))
            th2 = threading.Thread(target=th_movearm, name="th2", args=(self, bin, 'pre'))
            th1.start()
            th2.start()
            th1.join()
            th2.join()
            self.log_cnt += 1
            
            
        item_ranks = self.sort_item_priority_bin()
        #elif self.selected_mode=="bin2bin":
        #    if self.selected_item==item:
        #        return True
        #    else:
        #        return False
        
        # TODO better sort
        mxs = None
        mxt = -100
        
        
        
        # check blacklist
        
        rospy.loginfo("%d vac_grab strategy found"%len(self.strategies))
        
        if len(self.strategies)>0:
            for s in self.strategies:
                if s['is_ok'] and s['item']==item:
                    is_white = True
                    for bs in self.blacklist:
                        d = np.sum((s["pos"] - bs["pos"])**2)
                        if d<20**2 and s["motion_name"] == bs["motion_name"]:
                            print "rejected by blacklist: ", str(s["pos"]), s["motion_name"]
                            is_white=False
                            
                    if is_white and mxt < s['toru']:
                        mxs = s
                        mxt = s['toru']
        self.strategy_cache[bin] = []
        self.strategy_cache[bin] += self.strategies
                    
                    
        if self.final_mode[bin]:
            if ((mxs is None) or (mxt-item_ranks.tolist().index(item_string_to_id(s["item"]))*0.05<0.2)):
                return False
            rospy.loginfo("!!!!! %s is final mode"%bin)
        elif (mxs is None) or (mxt-item_ranks.tolist().index(item_string_to_id(s["item"]))*0.075<0.4):
            rospy.loginfo("no vac_grab strategy found...")
            """
            mxs = None
            mxt = -10000
        
            if len(self.strategies)>0:
                for s in self.strategies:
                    if s['is_ok']:
                        score = s['toru']
                        score -= s['pos'][0]/200.0   # items with small x get high score
                        if mxt < score:
                            mxs = s
                            mxt = score
            
            self.selected_mode = "bin2bin"
            self.selected_strategy = mxs
            if mxs is not None:
                self.selected_item = mxs['item']
            """
            return False
                    
                    
        self.selected_mode = "bin2tote"
        self.selected_strategy = mxs
        self.selected_item = item
        return True
        
    def can_grab_item_tote(self, item, bin, items_in_bin):
        item_id = item_string_to_id(item)
        
        # move to pre for avoiding over-rotation(J6)
        #rospy.loginfo('move to pre for avoiding over-rotation(J6)')
        #######################arm_control_wrapper.move_left_arm_to_bin(bin,'pre')
        
        
        if self.strategies is None or self.prev_bin != bin:        # TODO reuse previous result (caution: this is dangerous)
            self.strategies = []
            self.selected_mode = ""
            self.selected_strategy = None
            self.selected_item = item
            self.prev_bin = bin
            
            # move to photo
            rospy.loginfo('move to photo')
            
            (_,_,_,self.photo_pos) = arm_control_wrapper.move_left_arm_to_bin('tote','photo', keitai='nut000', kakujiku='chokusen', speed="fast")  # TODO chokusen de yoi?
            time.sleep(2.0) # wait for camera delay
            # take snapshot and imgprocs
            rospy.loginfo('take snapshot and imgprocs')
            resp = self.segm_service_left(['a'])
            
            self.item_coords = [[] for i in range(40)]
            
            def th_imgproc(self, resp, bin, items_in_bin, photo_dir):
                print 'imgproc start'
                Snapshot.save_snapshot(resp, str(self.log_cnt), 'log_imgs_raw')
                self.strategies += self.pick_imgproc_tote(bin, resp, self.photo_pos, items_in_bin, photo_dir=photo_dir)
                print 'imgproc end'
                
            def th_movearm(self, bin, photo_dir):
                print 'movearm start'
                (_,_,_,self.next_photo_pos) = arm_control_wrapper.move_left_arm_to_bin(bin, photo_dir, keitai='nut000', kakujiku='chokusen', speed="fast")
                time.sleep(2.0) # wait for camera delay
                print 'movearm end'
                
            
            th1 = threading.Thread(target=th_imgproc, name="th1", args=(self, resp, 'tote', items_in_bin, 'center'))
            th2 = threading.Thread(target=th_movearm, name="th2", args=(self, 'tote', 'photo_xm'))
            th1.start()
            th2.start()
            th1.join()
            th2.join()
            self.photo_pos = self.next_photo_pos
            self.log_cnt += 1
                
            resp = self.segm_service_left(['a'])
            
            th1 = threading.Thread(target=th_imgproc, name="th1", args=(self, resp, 'tote', items_in_bin, 'xm'))
            th2 = threading.Thread(target=th_movearm, name="th2", args=(self, 'tote', 'photo_xp'))
            th1.start()
            th2.start()
            th1.join()
            th2.join()
            self.photo_pos = self.next_photo_pos
            self.log_cnt += 1
                
            resp = self.segm_service_left(['a'])
            
            th1 = threading.Thread(target=th_imgproc, name="th1", args=(self, resp, 'tote', items_in_bin, 'xp'))
            th2 = threading.Thread(target=th_movearm, name="th2", args=(self, 'tote', 'photo_ym'))
            th1.start()
            th2.start()
            th1.join()
            th2.join()
            self.photo_pos = self.next_photo_pos
            self.log_cnt += 1
                
            resp = self.segm_service_left(['a'])
            
            th1 = threading.Thread(target=th_imgproc, name="th1", args=(self, resp, 'tote', items_in_bin, 'ym'))
            th2 = threading.Thread(target=th_movearm, name="th2", args=(self, 'tote', 'photo_yp'))
            th1.start()
            th2.start()
            th1.join()
            th2.join()
            self.photo_pos = self.next_photo_pos
            self.log_cnt += 1
                
            resp = self.segm_service_left(['a'])
            
            th1 = threading.Thread(target=th_imgproc, name="th1", args=(self, resp, 'tote', items_in_bin, 'yp'))
            th2 = threading.Thread(target=th_movearm, name="th2", args=(self, 'tote', 'photo'))
            th1.start()
            th2.start()
            th1.join()
            th2.join()
            self.log_cnt += 1
            
            
        
        mxs = None
        mxt = -100
        
        rospy.loginfo("%d vac_grab strategy found"%len(self.strategies))
        
        if len(self.strategies)>0:
            for s in self.strategies:
                if s['is_ok'] and s['item']==item:
                    if mxt < s['toru']:
                        mxs = s
                        mxt = s['toru']
                    
        if mxs is None:
            rospy.loginfo("no vac_grab strategy found...")
            return False
                    
        self.selected_mode = "tote2bin"
        self.selected_strategy = mxs
        self.selected_item = item
        return True
        
    def sort_item_priority_bin(self):
        # weight for each item
        # x-zahyou
        # toru score
        item_xs = [10000 for i in range(40)]
        for i in range(40):
            if len(self.item_coords[i])==0:
                continue
            item_xs[i] = np.mean(np.asarray(self.item_coords[i])[:,0])
        item_ranks = np.argsort(np.asarray(item_xs))
        for i,ir in enumerate(item_ranks):
            if item_xs[ir]<10000:
                print "sort #%d: %s"%(i, item_id_to_string(ir))
        #item_ranks.remove(0)
        return item_ranks
        
    def sort_item_priority_tote(self):
        # weight for each item
        # z-zahyou
        # toru score
        item_xs = [-10000 for i in range(40)]
        for i in range(40):
            if len(self.item_coords[i])==0:
                continue
            item_xs[i] = np.mean(np.asarray(self.item_coords[i])[:,2])
        item_ranks = np.argsort(-np.asarray(item_xs))
        for i,ir in enumerate(item_ranks):
            if item_xs[ir]>-10000:
                print "sort #%d: %s"%(i, item_id_to_string(ir))
        #item_ranks.remove(0)
        return item_ranks
            
    def output_log(self, rgb, score, toru, xyz, xyzrs, normal, normal_rs, xs, ys, xs_rejected, ys_rejected, rs_rejected):
        #plt.rcParams['figure.figsize'] = (20.0,20.0)
        plt.figure(figsize=(20.0,20.0))
        plt.clf()
        
        plt.subplot(4,3,1)
        plt.imshow(np.asarray(rgb)[:,:,:])
        plt.plot(xs, ys, marker='x', markersize=10, color='k', markeredgewidth=2, lw=0)
        plt.plot(xs_rejected[rs_rejected==0], ys_rejected[rs_rejected==0], marker='x', markersize=5, color='r', markeredgewidth=1, lw=0)
        plt.plot(xs_rejected[rs_rejected==1], ys_rejected[rs_rejected==1], marker='x', markersize=5, color='g', markeredgewidth=1, lw=0)
        plt.plot(xs_rejected[rs_rejected==2], ys_rejected[rs_rejected==2], marker='x', markersize=5, color='b', markeredgewidth=1, lw=0)
        #pylab.colorbar()
        plt.xlim(0,320)
        plt.ylim(240,0)
        plt.subplot(4,3,2)
        #pylab.clf()
        plt.imshow(np.asarray(rgb), alpha=0.2, interpolation='none')
        plt.imshow(toru[0,0,:,:], vmin=0, vmax=1, cmap=plt.cm.copper, alpha=0.7, interpolation='none')
        #plt.plot(xs, ys, marker='x', markersize=10, color='r', markeredgewidth=2, lw=0)
        #plt.colorbar()
        plt.xlim(0,320)
        plt.ylim(240,0)
        plt.axis('off')
        plt.subplot(4,3,3)
        #pylab.clf()
        plt.imshow((np.argmax(score, axis=1)[0,:]), cmap=plt.cm.Paired, vmin=0, vmax=40, interpolation='none')
        #plt.plot(xs, ys, marker='x', markersize=10, color='r', markeredgewidth=2, lw=0)
        plt.colorbar()
        plt.xlim(0,320)
        plt.ylim(240,0)
        plt.axis('off')
        
        plt.subplot(4,3,4)
        plt.imshow(xyz[0,:,:], cmap=plt.cm.Paired, vmin=500, vmax=2000, interpolation='none')
        plt.plot(xs, ys, marker='x', markersize=10, color='r', markeredgewidth=1, lw=0)
        plt.colorbar()
        plt.xlim(0,320)
        plt.ylim(240,0)
        plt.axis('off')
        
        plt.subplot(4,3,5)
        plt.imshow(xyz[1,:,:], cmap=plt.cm.Paired, vmin=-1000, vmax=500, interpolation='none')
        plt.plot(xs, ys, marker='x', markersize=10, color='r', markeredgewidth=1, lw=0)
        plt.colorbar()
        plt.xlim(0,320)
        plt.ylim(240,0)
        plt.axis('off')
        
        plt.subplot(4,3,6)
        plt.imshow(xyz[2,:,:], cmap=plt.cm.Paired, vmin=-1000, vmax=1000, interpolation='none')
        plt.plot(xs, ys, marker='x', markersize=10, color='r', markeredgewidth=1, lw=0)
        plt.colorbar()
        plt.axis('off')
        
        plt.subplot(4,3,7)
        plt.imshow(xyzrs[0,:,:], cmap=plt.cm.Paired, vmin=500, vmax=2000, interpolation='none')
        plt.plot(xs, ys, marker='x', markersize=10, color='r', markeredgewidth=1, lw=0)
        plt.colorbar()
        plt.xlim(0,320)
        plt.ylim(240,0)
        plt.axis('off')
        
        plt.subplot(4,3,8)
        plt.imshow(xyzrs[1,:,:], cmap=plt.cm.Paired, vmin=-1000, vmax=500, interpolation='none')
        plt.plot(xs, ys, marker='x', markersize=10, color='r', markeredgewidth=1, lw=0)
        plt.colorbar()
        plt.xlim(0,320)
        plt.ylim(240,0)
        plt.axis('off')
        
        plt.subplot(4,3,9)
        plt.imshow(xyzrs[2,:,:], cmap=plt.cm.Paired, vmin=-1000, vmax=1000, interpolation='none')
        plt.plot(xs, ys, marker='x', markersize=10, color='r', markeredgewidth=1, lw=0)
        plt.colorbar()
        plt.axis('off')
        
        plt.subplot(4,3,10)
        #plt.imshow(np.asarray(rgb), alpha=0.2, interpolation='none')
        plt.imshow(normal.transpose(1,2,0)*0.5+0.5, interpolation='none')
        plt.plot(xs, ys, marker='x', markersize=10, color='r', markeredgewidth=1, lw=0)
        plt.axis('off')
        plt.xlim(0,320)
        plt.ylim(240,0)
        
        plt.subplot(4,3,11)
        #plt.imshow(np.asarray(rgb), alpha=0.2, interpolation='none')
        plt.imshow(normal_rs.transpose(1,2,0)*0.5+0.5, interpolation='none')
        plt.plot(xs, ys, marker='x', markersize=10, color='r', markeredgewidth=1, lw=0)
        plt.axis('off')
        plt.xlim(0,320)
        plt.ylim(240,0)
        
        plt.savefig('log_imgs/log_%d.jpg'%self.log_cnt)
        
        
        
        dpi = 80
        figsize = 320/dpi, 240/dpi
        fig = plt.figure(figsize=figsize)
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        
        fig.clf()
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        ax.imshow(normal.transpose(1,2,0)*0.5+0.5, interpolation='none')
        #ax.axis('off')
        #plt.subplots_adjust(left=0,bottom=0,right=1,top=1)
        ax.set(xlim=[0,320],ylim=[240,0], aspect=1)
        fig.savefig('log_imgs/normal_fx8_%d.png'%(self.log_cnt), dpi=dpi, transparent=True)
        
        
        fig.clf()
        ax = fig.add_axes([0,0,1,1])
        ax.axis('off')
        ax.imshow(normal_rs.transpose(1,2,0)*0.5+0.5, interpolation='none')
        #ax.axis('off')
        #plt.subplots_adjust(left=0,bottom=0,right=1,top=1)
        ax.set(xlim=[0,320],ylim=[240,0], aspect=1)
        fig.savefig('log_imgs/normal_realsense_%d.png'%(self.log_cnt), dpi=dpi, transparent=True)
                
if __name__ == '__main__':
    rospy.init_node('vacuum_grabbing')
    VacuumGrabbing()
