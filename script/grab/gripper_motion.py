#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
import math
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

from util import camera_to_global, item_string_to_id

import chainer
from chainer import cuda
from chainer import serializers
from chainer import Variable
import chainer.functions as F
import chainer.links as L


from gripper_sensing import l2a, l2a_


from util import camera_to_global, item_string_to_id


def print_status(fb):
    print "progress:\n ", "\n  ".join(str(fb).split("\n"))
    

    
class hand_params:
    def __init__(self):
        self.arm_L_x = 630
        
        self.grip_offset_x = 30
        self.grip_center_diff_y = 45#70
        
        
    

class gripper_motion:
    def __init__(self, hand_params):
        
        rospy.loginfo( 'wait bin2global and global2bin')
        rospy.wait_for_service('bin2global')
        rospy.wait_for_service('global2bin')
        self.sv_bin2global = rospy.ServiceProxy('bin2global',CoordinateTransform)
        self.sv_global2bin = rospy.ServiceProxy('global2bin',CoordinateTransform)
        
        rospy.loginfo( 'wait gripper_angle')
        self.gpcl = actionlib.SimpleActionClient('/gripper_angle', GenericAction)
        self.gpcl.wait_for_server()
        
        
        self.hand_params = hand_params
        
        self.target_bin = ''
        
        self.prev_grip_type = -1
        self.prev_grip_armpose = None
        
        
        rospy.loginfo('gripper -- ok!')
        
        
    def move_grip(self, vstr):
        rospy.loginfo('gripper -- open/close grip ' + vstr)
        
        glgoal = GenericGoal()
        glgoal.target = vstr
        self.gpcl.send_goal(glgoal, feedback_cb=print_status)
        self.gpcl.wait_for_result(rospy.Duration.from_sec(300.0))
        gres = self.gpcl.get_result()  
        
        rospy.loginfo('gripper -- open/close grip finish ' + vstr)
        
        
        
    def set_bin_for_move(self, bin):
        self.target_bin = bin
        
    def move_for_scan(self, pos, angle):
        arm_control_wrapper.move_right_arm_local(self.target_bin, l2a(pos, angle), 'nut000', 'chokusen', 'normal')
        
        self.prev_grip_armpose = l2a(pos, angle)
        
    def move_for_grip(self, pos, angle):
        arm_control_wrapper.move_right_arm_local(self.target_bin, l2a(pos, angle), 'fut000', 'kakujiku', 'normal')
        
        self.prev_grip_armpose = l2a(pos, angle)
        
        
    def move_for_grip_tote(self, pos, angle):
        arm_control_wrapper.move_right_arm_global( l2a(pos, angle), 'nut000', 'kakujiku', 'normal')
        
        self.prev_grip_armpose = l2a(pos, angle)
        
        
    def grip_dumbbell_horizontal(self, bin, item, item_pose, rl):  #rl : 'right'= plate is right side
        
        
        rospy.loginfo('gripper -- grip_dumbbell_horizontal')
        
        
        self.set_bin_for_move(bin)
        
        #raw_input("stop")
        
        
        angle_start = [180,0,180]
        
        angle_r = [98.5,0,90]     #plate right
        angle_lo = [-92,0,-90]   #plate left     + open 2000
        angle_lc = [-97,0,-90]   #plate left     + close 0
        angle_d = [90,-90,90]    #plate down
        
        if rl == 'right':
            angle_hori = angle_r
        else:
            angle_hori = angle_lo
            
            
        #rist pos
        
        horoffset = 0
        if abs(item_pose[3]) > 0.35 and abs(item_pose[4]) > 0.35:
            horoffset = 5 if item_pose[4] > 0 else -5
        
        rx = item_pose[0] - self.hand_params.arm_L_x + self.hand_params.grip_offset_x + 30+40
        ry = item_pose[1] + self.hand_params.grip_center_diff_y if rl == 'right' else -self.hand_params.grip_center_diff_y + horoffset
        rz = item_pose[2]
        
        
        #motion params
        upz = 110
        dwnangleoffz = 75 + 10 + 5
        

        
        if bin == 'bin_J' or bin == 'bin_K' or bin == 'bin_L':
            h_up_b = 250
        else:
            h_up_b = 102

        
        h_up = 102        
                    
            
        h_down = 102
        
        tsoffx = 95    #sliding pos
        
        
        pos_start = [-700, ry, h_up_b]
        pos_grip_up = [rx - tsoffx, ry, h_up]
        pos_grip_down = [rx - tsoffx, ry, h_down]
        pos_grip_inside = [rx, ry, h_down-30]
        pos_grip_inside_up = [rx, ry, h_up+50]
        pos_start_up = [-700, ry, h_up+50]
        
        
        # start
        rospy.loginfo('gripper -- start grip horizontal')
        
        
        arm_control_wrapper.move_right_arm_local(bin, l2a_(pos_start+angle_start), 'nut000', 'kakujiku', 'normal')

        arm_control_wrapper.move_right_arm_local(bin, l2a_(pos_start+angle_hori), 'fut000', 'kakujiku', 'normal')
        
        self.move_grip('2000')
        
        arm_control_wrapper.move_right_arm_local(bin, l2a_(pos_grip_up+angle_hori), 'fut000', 'chokusen', 'fast')
                
        arm_control_wrapper.move_right_arm_local(bin, l2a_(pos_grip_down+angle_hori), 'fut000', 'chokusen', 'normal')
        
        arm_control_wrapper.move_right_arm_local(bin, l2a_(pos_grip_inside+angle_hori), 'fut000', 'chokusen', 'normal')
        
        self.move_grip('0')
        rospy.loginfo('gripper hori1')
        arm_control_wrapper.move_right_arm_local(bin, l2a_(pos_grip_inside_up+angle_hori), 'fut000', 'chokusen', 'normal')
        
        
        corner_shift_y = 0
        if bin == 'bin_A' or bin == 'bin_D' or bin == 'bin_G' or bin == 'bin_J':
            if item_pose[1] > -130:
                corner_shift_y = -35
            
        if bin == 'bin_C' or bin == 'bin_F' or bin == 'bin_I' or bin == 'bin_L':
            if item_pose[1] < -160:
                corner_shift_y = 35
        
        rospy.loginfo('gripper hori2')
        sp = pos_grip_inside_up
        sp[1] = sp[1] + corner_shift_y
        arm_control_wrapper.move_right_arm_local(bin, l2a_(sp+angle_hori), 'fut000', 'chokusen', 'normal')
        
        rospy.loginfo('gripper hori3') 
        sp = pos_start_up
        sp[1] = sp[1] + corner_shift_y
        arm_control_wrapper.move_right_arm_local(bin, l2a_(sp+angle_hori), 'fut000', 'chokusen', 'normal')
        
               

        self.prev_grip_type = 1
        
        rospy.loginfo('gripper -- finish grip horizontal')
        
        
        
    def grip_dumbbell_vertical_stand(self, bin, item, item_pose):
        
        
        rospy.loginfo('gripper -- grip_dumbbell_vertical_stand')
        
        
        self.set_bin_for_move(bin)
        
        #raw_input("stop")
        
            
        open_offset_y = 25
            
        #rist pos
        rx = item_pose[0] - self.hand_params.arm_L_x + self.hand_params.grip_offset_x +30
        ry = item_pose[1] + 20
        rz = 50 #item_pose[2] + 50  #kimeuchi
               
        
        tsoffx = 100    #sliding pos
        inoffx = 50 #tsukkomi
        
        
        # start
        rospy.loginfo('gripper -- start grip vertical stand')
        
        
        
        arm_control_wrapper.move_right_arm_local(bin,  l2a_([-700,ry,rz,-90,-90,-90]), 'fut000', 'kakujiku', 'normal')
        
        self.move_grip('2000')
                
        arm_control_wrapper.move_right_arm_local(bin, l2a_([rx+inoffx,ry,rz,-90,-90,-90]), 'fut000', 'chokusen', 'normal')
       
        self.move_grip('0')       
          
        arm_control_wrapper.move_right_arm_local(bin, l2a_([-700,ry,rz,-90,-90,-90]), 'fut000', 'chokusen', 'normal')
        
        
        self.prev_grip_type = 2
        
        rospy.loginfo('gripper -- finish grip vertical stand')
        
        
        
    def grip_dumbbell_vertical(self, bin, item, item_pose):
        
        
        rospy.loginfo('gripper -- grip_dumbbell_vertical')
        
        self.set_bin_for_move(bin)
        
        #raw_input("stop")
        
        rx = item_pose[0] - 500
        ry = item_pose[1] + 20
        rz = item_pose[2] + 40
        
        
        arm_control_wrapper.move_right_arm_local(bin, l2a_([-700, ry+20, 190] + [180, -79,0]), 'fut000', 'kakujiku', 'normal')

             
        if item_pose[0] > 100:  #150made hikidasu
            #hikidasi
            
            self.move_grip('3000')
            
            arm_control_wrapper.move_right_arm_local(bin, l2a_([rx, ry+20, 185] + [180, -79,0]), 'fut000', 'chokusen', 'normal')
            arm_control_wrapper.move_right_arm_local(bin, l2a_([rx, ry+20, 157] + [180, -79,0]), 'fut000', 'chokusen', 'normal')
            
            self.move_grip('1000')
            
            arm_control_wrapper.move_right_arm_local(bin, l2a_([-420, ry+20, 185] + [180, -79,0]), 'fut000', 'chokusen', 'normal')
            
            self.move_grip('3000')
            
            
            
            arm_control_wrapper.move_right_arm_local(bin, l2a_([-700, ry+20, 185] + [180, -79,0]), 'fut000', 'chokusen', 'normal') #ichido dasu
            
            
            
            ix = 100
            
        else:
            ix = item_pose[0]
            
        
        self.move_grip('2000')
        
        hx = 410
        hz = 430
        hax=180
        hay=-50
        haz=0
        
        poffx=100
        poffz = 80
        arm_control_wrapper.move_right_arm_local(bin, l2a_([ix-hx -poffx, ry, hz+poffz] + [hax, hay, haz]), 'fut000', 'chokusen', 'normal')
        
        arm_control_wrapper.move_right_arm_local(bin, l2a_([ix-hx, ry,  hz+poffz] + [hax, hay, haz]), 'fut000', 'chokusen', 'normal')
        
        arm_control_wrapper.move_right_arm_local(bin, l2a_([ix-hx, ry,  hz] + [hax, hay, haz]), 'fut000', 'chokusen', 'normal')
        
        self.move_grip('0')
        
        arm_control_wrapper.move_right_arm_local(bin, l2a_([ix-hx, ry, hz] + [hax, hay, haz]), 'fut000', 'chokusen', 'normal')
        
        arm_control_wrapper.move_right_arm_local(bin, l2a_([ix-hx, ry, hz+poffz] + [hax, hay, haz]), 'fut000', 'chokusen', 'normal')
        
        arm_control_wrapper.move_right_arm_local(bin, l2a_([ix-hx -poffx, ry, hz+poffz] + [hax, hay, haz]), 'fut000', 'chokusen', 'normal')
    
   
        arm_control_wrapper.move_right_arm_local(bin, l2a_([-700, ry, hz+poffz] + [hax, hay, haz]), 'fut000', 'chokusen', 'normal')
        
        
        self.prev_grip_type = 2
        
        rospy.loginfo('gripper -- finish girp')
        
        
    def grip_pencilcup_w_someitems(self,bin,item,res):

        rospy.loginfo('gripper -- grip_pencilcup_w_someitems')
        
        print 'res'
        print res
        
        rz = 90
        
        xpre = -700
        xinside = -350
        #xinside = -425
        
        
        cbin = False
        if bin == 'bin_B' or bin == 'bin_E' or bin == 'bin_H' or bin == 'bin_K':
            cbin = True            
            
        if abs(res[1]) < 30:   #pixel
            if cbin == True: #c bin
                ypos = -85 #left
            else:
                ypos = -60  #right
        else:
            if cbin == True: #c bin
                if res[1] > 0:
                    ypos = -23 #left
                else:
                    ypos = -155  #right
                    
            else:   #r or l bin
                if res[1] > 0:
                    ypos = -23 #left
                else:
                    ypos = -105  #right
            
        print 'ypos'
        print ypos
        
        
        if bin == 'bin_J' or bin == 'bin_K' or bin == 'bin_L':
            h_up = 250
        else:
            h_up = 0
            
        arm_control_wrapper.move_right_arm_local(bin,  l2a_([xpre, ypos,rz+h_up,-90,-90,-90]), 'fut000', 'kakujiku', 'normal')
        arm_control_wrapper.move_right_arm_local(bin,  l2a_([xpre,ypos,rz,-90,-90,-90]), 'fut000', 'chokusen', 'normal') 
          
        self.move_grip('5000')
                
        arm_control_wrapper.move_right_arm_local(bin, l2a_([xinside,ypos,rz,-90,-90,-90]), 'fut000', 'chokusen', 'normal')
       
        self.move_grip('3500')       
          
        zup = 60
        arm_control_wrapper.move_right_arm_local(bin, l2a_([xinside,ypos,rz+zup,-90,-90,-90]), 'fut000', 'chokusen', 'normal')
        arm_control_wrapper.move_right_arm_local(bin, l2a_([xpre,ypos,rz+zup,-90,-90,-90]), 'fut000', 'chokusen', 'normal')
        
        
        self.prev_grip_type = 10
        
        rospy.loginfo('gripper -- finish grip_pencilcup_w_someitems') 
        
        
        
    
    
    
        
    def grip_pencilcup_one(self,bin,item,res):

        rospy.loginfo('gripper -- grip_pencilcup_one')
        
        print 'res'
        print res
        
        rz = 90
        
        xpre = -700
        xinside = -250
        
        
        cbin = False
        if bin == 'bin_B' or bin == 'bin_E' or bin == 'bin_H' or bin == 'bin_K':
            cbin = True            
            
        if abs(res[1]) < 30:   #pixel
            if cbin == True: #c bin
                ypos = -85 #left
            else:
                ypos = -60  #right
        else:
            if cbin == True: #c bin
                if res[1] > 0:
                    ypos = -23 #left
                else:
                    ypos = -155  #right
                    
            else:   #r or l bin
                if res[1] > 0:
                    ypos = -23 #left
                else:
                    ypos = -105  #right
            
        print 'ypos'
        print ypos
        
        
        if bin == 'bin_J' or bin == 'bin_K' or bin == 'bin_L':
            h_up = 250
        else:
            h_up = 0
            
            
        arm_control_wrapper.move_right_arm_local(bin,  l2a_([xpre, ypos,rz+h_up,-90,-90,-90]), 'fut000', 'kakujiku', 'normal')
        arm_control_wrapper.move_right_arm_local(bin,  l2a_([xpre,ypos,rz,-90,-90,-90]), 'fut000', 'chokusen', 'normal') 
               
        self.move_grip('5000')
                
        arm_control_wrapper.move_right_arm_local(bin, l2a_([xinside,ypos,rz,-90,-90,-90]), 'fut000', 'chokusen', 'normal')
       
        self.move_grip('3500')       
        rospy.loginfo('gripper cup1')   
        zup = 60
        arm_control_wrapper.move_right_arm_local(bin, l2a_([xinside,ypos,rz+zup,-90,-90,-90]), 'fut000', 'chokusen', 'normal')
        
        rospy.loginfo('gripper cup2') 
        arm_control_wrapper.move_right_arm_local(bin, l2a_([xpre,ypos,rz+zup,-90,-90,-90]), 'fut000', 'chokusen', 'normal')
        
        
        self.prev_grip_type = 10
        
        rospy.loginfo('gripper -- finish grip_pencilcup_one') 
        
        
        


    def stow_in_tote(self, pos):


        print 'prev grip type : %d' % self.prev_grip_type
        print self.prev_grip_armpose

        #if self.prev_grip_type == 1:    #horizontal

        #change angle

        ap = [500, -220, -110]
        agl = [-180,0,90]
        rospy.loginfo('gripper tote0')
        arm_control_wrapper.move_right_arm_global( l2a(ap, agl), 'nut000', 'kakujiku', 'normal')


        pos = [550, -220, -980]    #stow pos
        
        bp = [0,0,0]
        bp[0] = pos[0]
        bp[1] = pos[1] + self.hand_params.grip_center_diff_y
        bp[2] = pos[2] + self.hand_params.arm_L_x

        rospy.loginfo('gripper tote1')
        arm_control_wrapper.move_right_arm_global( l2a(bp, agl), 'nut000', 'kakujiku', 'normal')


        if self.prev_grip_type < 10:
            self.move_grip('2000')

        else:
            #for cup
            self.move_grip('5000')
            time.sleep(1.0)
            self.move_grip('3500')
            
        
        rospy.loginfo('gripper tote2')
        
        hup = 300
        cp = bp
        cp[2] = cp[2] + hup
        arm_control_wrapper.move_right_arm_global( l2a(cp, agl), 'nut000', 'kakujiku', 'normal')
    
        
        
    def grip_from_tote(self, itempose):  #global

        rospy.loginfo('gripper -- grip from tote')
            
        x = itempose[0]
        y = itempose[1]
        z = itempose[2]
        
        ax = itempose[3]    #seibun
        ay = itempose[4]
        az = itempose[5]
        
        
        if abs(ax - ay) < 0.2:
            #stand
            ax = 0
        else:
            az_ = math.atan2(ay,ax)
            
            
         
        tote_z = -1160
        z1 = -1160 + self.hand_params.arm_L_x + 300
        z2 = -1160 + self.hand_params.arm_L_x + 0
        
        arm_control_wrapper.move_right_arm_global( l2a_([x,y,z1,-180, -0,az]), 'nut000', 'kakujiku', 'normal') #-180,0, then az
        
        self.move_grip('2000')
        
        arm_control_wrapper.move_right_arm_global( l2a_([x,y,z2,-180, -0,az]), 'nut000', 'chokusen', 'normal') #-180,0, then az
        
        self.move_grip('0')
    
        arm_control_wrapper.move_right_arm_global( l2a_([x,y,z1,-180, -0,az]), 'nut000', 'chokusen', 'normal') #-180,0, then az
        
        arm_control_wrapper.move_right_arm_global( l2a_([550,y,z1,-180, -0,az]), 'nut000', 'chokusen', 'normal')
        
        arm_control_wrapper.move_right_arm_global( l2a_([550,y,z1,90,0,90]), 'fut000', 'kakujiku', 'normal')
        
        
        
        
        
    def stow_in_shelf(self, target_bin, target_position):

        if self.prev_grip_type == 10:
            self.stow_in_shelf_cup(target_bin, target_position)
            return

    
        rospy.loginfo('gripper -- stow shelf')
        p1 = np.asarray([-700,-120,270,112,0,90])
        p2 = np.asarray([-400,-120,270,112,0,90])
        p3 = np.asarray([-400,-120,270,112,0,90])
        
        
        arm_control_wrapper.move_right_arm_local(target_bin, p1, 'fut000', 'chokusen', 'normal')
        arm_control_wrapper.move_right_arm_local(target_bin, p2, 'fut000', 'chokusen', 'normal')
        arm_control_wrapper.move_right_arm_local(target_bin, p3, 'fut000', 'chokusen', 'normal')
        

        self.move_grip('2000')
        
        arm_control_wrapper.move_right_arm_local(target_bin, p2, 'fut000', 'chokusen', 'normal')

        arm_control_wrapper.move_right_arm_local(target_bin, p1, 'fut000', 'chokusen', 'normal')
    
        self.move_grip('0')
        
        
    def stow_in_shelf_cup(self, target_bin, target_position):
    
        rospy.loginfo('gripper -- stow shelf')
        p1 = np.asarray([-700,-120,150,-90,-90,-90])
        p2 = np.asarray([-400,-120,150,-90,-90,-90])
        p3 = np.asarray([-400,-120,150,-90,-90,-90])
        
        
        arm_control_wrapper.move_right_arm_local(target_bin, p1, 'fut000', 'chokusen', 'normal')
        arm_control_wrapper.move_right_arm_local(target_bin, p2, 'fut000', 'chokusen', 'normal')
        arm_control_wrapper.move_right_arm_local(target_bin, p3, 'fut000', 'chokusen', 'normal')
        

        #for cup
        self.move_grip('5000')

        
        arm_control_wrapper.move_right_arm_local(target_bin, p2, 'fut000', 'chokusen', 'normal')

        arm_control_wrapper.move_right_arm_local(target_bin, p1, 'fut000', 'chokusen', 'normal')
    
        self.move_grip('2000')
