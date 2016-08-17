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

import numpy as np

import rospy
from std_msgs.msg import String


from std_msgs.msg import Bool
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Vector3, Point
from std_srvs.srv import Empty


import time
import random
import math
import roslib
import actionlib
from actionlib_msgs.msg import GoalStatus
from apc2016.msg import *
from apc2016.srv import *

import util


class LeftRight_arm:
    def __init__(self, lr, w0=480, h0=360, with_vac=True):	#lr=('left' or 'right')
        rospy.loginfo("initialising %s arm" % lr)
        self.pub = rospy.Publisher( lr+'_state', String , queue_size = 10, latch=True)        
        msg = String()
        #msg.data  = "workspace"
        #self.pub.publish(msg)
        # calc_x: 1235, -207, 559
        # tool_x: 790, -231, 515
        #self.hosei = np.asarray([790-1235+30,(-231)-(-207),515-559+0,0,0,0])
        
        # calc_x: 494, -181, 913
        # tool_x: 666, -29, 415
        self.hosei = np.asarray([666-494,(-29)-(-181),415-913+0,0,0,0])
        
        self.lr = lr
        lr_offset = 0
        lr_roll = 0
        lr_yaw = 0
        if lr == 'right':
            lr_offset = 0
#            lr_roll = -180
#            lr_yaw = -360
        self.preset_position = {}
        dy = [0, -280, -560]
        # dz = [0, -250, -480, -730]
        dz = [0, -265, -494, -723]
        # 72
        
        if lr == 'left':
            self.preset_position["bin_A_photo"] = np.asarray([775+0-50,lr_offset-102-20,690,180,-20,179.9])#-170])
        else:
            self.preset_position["bin_A_photo"] = np.asarray([775+50,lr_offset-102-20,720,180,-20,0])#-170])
        self.preset_position["bin_B_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[1], dz[0], 0, 0, 0])
        self.preset_position["bin_C_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[2], dz[0], 0, 0, 0])
        self.preset_position["bin_D_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[0], dz[1], 0, 0, 0])
        self.preset_position["bin_E_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[1], dz[1], 0, 0, 0])
        self.preset_position["bin_F_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[2], dz[1], 0, 0, 0])
        self.preset_position["bin_G_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[0], dz[2], 0, 0, 0])
        self.preset_position["bin_H_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[1], dz[2], 0, 0, 0])
        self.preset_position["bin_I_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[2], dz[2], 0, 0, 0])
        self.preset_position["bin_J_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[0], dz[3], 0, 0, 0])
        self.preset_position["bin_K_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[1], dz[3], 0, 0, 0])
        self.preset_position["bin_L_photo"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[2], dz[3], 0, 0, 0])
        
        if lr == 'left': 
            self.preset_position["bin_A_photo_r"] = np.asarray([810-50,lr_offset-356,690,180,-20,-160])
        else:
            self.preset_position["bin_A_photo_r"] = np.asarray([810,lr_offset-356,720,180,-20,20])
        self.preset_position["bin_B_photo_r"] = self.preset_position["bin_A_photo_r"] + np.asarray([0, dy[1], dz[0], 0, 0, 0])
        self.preset_position["bin_C_photo_r"] = self.preset_position["bin_A_photo"] + np.asarray([0, dy[2], dz[0], 0, 0, -350])
        self.preset_position["bin_D_photo_r"] = self.preset_position["bin_A_photo_r"] + np.asarray([0, dy[0], dz[1], 0, 0, 0])
        self.preset_position["bin_E_photo_r"] = self.preset_position["bin_A_photo_r"] + np.asarray([0, dy[1], dz[1], 0, 0, 0])
        self.preset_position["bin_F_photo_r"] = self.preset_position["bin_A_photo_r"] + np.asarray([0, dy[2], dz[1], 0, 0, 0])
        self.preset_position["bin_G_photo_r"] = self.preset_position["bin_A_photo_r"] + np.asarray([0, dy[0], dz[2], 0, 0, 0])
        self.preset_position["bin_H_photo_r"] = self.preset_position["bin_A_photo_r"] + np.asarray([0, dy[1], dz[2], 0, 0, 0])
        self.preset_position["bin_I_photo_r"] = self.preset_position["bin_A_photo_r"] + np.asarray([0, dy[2], dz[2], 0, 0, 0])
        self.preset_position["bin_J_photo_r"] = self.preset_position["bin_A_photo_r"] + np.asarray([0, dy[0], dz[3], 0, 0, 0])
        self.preset_position["bin_K_photo_r"] = self.preset_position["bin_A_photo_r"] + np.asarray([0, dy[1], dz[3], 0, 0, 0])
        self.preset_position["bin_L_photo_r"] = self.preset_position["bin_A_photo_r"] + np.asarray([0, dy[2], dz[3], 0, 0, 0])
        
        if lr == 'left':
            self.preset_position["bin_A_photo_l"] = np.asarray([810-50,lr_offset+114,690,180,-20,160])
        else:
            self.preset_position["bin_A_photo_l"] = np.asarray([810,lr_offset+154,720,180,-20,-20])
        self.preset_position["bin_B_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[1], dz[0], 0, 0, 0])
        self.preset_position["bin_C_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[2], dz[0], 0, 0, 0])
        self.preset_position["bin_D_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[0], dz[1], 0, 0, 0])
        self.preset_position["bin_E_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[1], dz[1], 0, 0, 0])
        self.preset_position["bin_F_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[2], dz[1], 0, 0, 0])
        self.preset_position["bin_G_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[0], dz[2], 0, 0, 0])
        self.preset_position["bin_H_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[1], dz[2], 0, 0, 0])
        self.preset_position["bin_I_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[2], dz[2], 0, 0, 0])
        self.preset_position["bin_J_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[0], dz[3], 0, 0, 0])
        self.preset_position["bin_K_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[1], dz[3], 0, 0, 0])
        self.preset_position["bin_L_photo_l"] = self.preset_position["bin_A_photo_l"] + np.asarray([0, dy[2], dz[3], 0, 0, 0])
        
        self.preset_position["bin_A_pre"] = np.asarray([550,lr_offset-102,713,180+lr_yaw,-0.1,179.9+lr_roll])
        self.preset_position["bin_B_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[1], dz[0], 0, 0, 0])
        self.preset_position["bin_C_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[2], dz[0], 0, 0, 0])
        self.preset_position["bin_D_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[0], dz[1], 0, 0, 0])
        self.preset_position["bin_E_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[1], dz[1], 0, 0, 0])
        self.preset_position["bin_F_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[2], dz[1], 0, 0, 0])
        self.preset_position["bin_G_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[0], dz[2], 0, 0, 0])
        self.preset_position["bin_H_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[1], dz[2], 0, 0, 0])
        self.preset_position["bin_I_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[2], dz[2], 0, 0, 0])
        self.preset_position["bin_J_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[0], dz[3], 0, 0, 0])
        self.preset_position["bin_K_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[1], dz[3], 0, 0, 0])
        self.preset_position["bin_L_pre"] = self.preset_position["bin_A_pre"] + np.asarray([0, dy[2], dz[3], 0, 0, 0])
        
        self.preset_position["bin_A_rpre"] = np.asarray([550,lr_offset-102,713, -180, 0, 0])
        self.preset_position["bin_B_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[1], dz[0], 0, 0, 0])
        self.preset_position["bin_C_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[2], dz[0], 0, 0, 0])
        self.preset_position["bin_D_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[0], dz[1], 0, 0, 0])
        self.preset_position["bin_E_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[1], dz[1], 0, 0, 0])
        self.preset_position["bin_F_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[2], dz[1], 0, 0, 0])
        self.preset_position["bin_G_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[0], dz[2], 0, 0, 0])
        self.preset_position["bin_H_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[1], dz[2], 0, 0, 0])
        self.preset_position["bin_I_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[2], dz[2], 0, 0, 0])
        self.preset_position["bin_J_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[0], dz[3], 0, 0, 0])
        self.preset_position["bin_K_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[1], dz[3], 0, 0, 0])
        self.preset_position["bin_L_rpre"] = self.preset_position["bin_A_rpre"] + np.asarray([0, dy[2], dz[3], 0, 0, 0])
        
        self.preset_position["bin_A_front"] = np.asarray([432,lr_offset-87,483,90,-90,90])
        self.preset_position["bin_B_front"] = self.preset_position["bin_A_front"] + np.asarray([0, dy[1], dz[0], 0, 0, 0])
        self.preset_position["bin_C_front"] = self.preset_position["bin_A_front"] + np.asarray([0, dy[2], dz[0], 0, 0, 0])
        self.preset_position["bin_D_front"] = self.preset_position["bin_A_front"] + np.asarray([0, dy[0], dz[1], 0, 0, 0])
        self.preset_position["bin_E_front"] = self.preset_position["bin_A_front"] + np.asarray([0, dy[1], dz[1], 0, 0, 0])
        self.preset_position["bin_F_front"] = self.preset_position["bin_A_front"] + np.asarray([0, dy[2], dz[1], 0, 0, 0])
        self.preset_position["bin_G_front"] = self.preset_position["bin_A_front"] + np.asarray([50, dy[0], dz[2], 0, 0, 0])
        self.preset_position["bin_H_front"] = self.preset_position["bin_A_front"] + np.asarray([0, dy[1], dz[2], 0, 0, 0])
        self.preset_position["bin_I_front"] = self.preset_position["bin_A_front"] + np.asarray([0, dy[2], dz[2], 0, 0, 0])
        self.preset_position["bin_J_front"] = self.preset_position["bin_A_front"] + np.asarray([50, dy[0], dz[3], 0, 0, 0])
        self.preset_position["bin_K_front"] = self.preset_position["bin_A_front"] + np.asarray([0, dy[1], dz[3], 0, 0, 0])
        self.preset_position["bin_L_front"] = self.preset_position["bin_A_front"] + np.asarray([0, dy[2], dz[3], 0, 0, 0])
        
        self.preset_position["bin_A_pull"] = np.asarray([432,lr_offset-87,560-40,90,-90,90])  #TODO takasugi??
        self.preset_position["bin_B_pull"] = self.preset_position["bin_A_pull"] + np.asarray([0, dy[1], dz[0], 0, 0, 0])
        self.preset_position["bin_C_pull"] = self.preset_position["bin_A_pull"] + np.asarray([0, dy[2], dz[0], 0, 0, 0])
        self.preset_position["bin_D_pull"] = self.preset_position["bin_A_pull"] + np.asarray([0, dy[0], dz[1], 0, 0, 0])
        self.preset_position["bin_E_pull"] = self.preset_position["bin_A_pull"] + np.asarray([0, dy[1], dz[1], 0, 0, 0])
        self.preset_position["bin_F_pull"] = self.preset_position["bin_A_pull"] + np.asarray([0, dy[2], dz[1], 0, 0, 0])
        self.preset_position["bin_G_pull"] = self.preset_position["bin_A_pull"] + np.asarray([50, dy[0], dz[2], 0, 0, 0])
        self.preset_position["bin_H_pull"] = self.preset_position["bin_A_pull"] + np.asarray([0, dy[1], dz[2], 0, 0, 0])
        self.preset_position["bin_I_pull"] = self.preset_position["bin_A_pull"] + np.asarray([0, dy[2], dz[2], 0, 0, 0])
        self.preset_position["bin_J_pull"] = self.preset_position["bin_A_pull"] + np.asarray([50, dy[0], dz[3], 0, 0, 0])
        self.preset_position["bin_K_pull"] = self.preset_position["bin_A_pull"] + np.asarray([0, dy[1], dz[3], 0, 0, 0])
        self.preset_position["bin_L_pull"] = self.preset_position["bin_A_pull"] + np.asarray([0, dy[2], dz[3], 0, 0, 0])
        
        
        self.preset_position["origin"] = np.asarray([550,lr_offset-146,752,180,0,180])
        
        
        self.preset_position["tote"] = np.asarray([800, 620, 260, -180, 0, 0])
        self.preset_position["tote_photo"] = np.asarray([800, 620, 260-30, 0, -90, -180])
        self.preset_position["tote_photo_xm"] = np.asarray([466, 620, 255-30, 0, -70, 180])
        self.preset_position["tote_photo_xp"] = np.asarray([1094, 620, 255-30, 180, -70, 0])
        self.preset_position["tote_photo_ym"] = np.asarray([800, 290, 255-30, -90, -70, -90])
        self.preset_position["tote_photo_yp"] = np.asarray([800, 980, 255-30, 90, -70, 90])
        
        #TODO 
        #self.preset_position["escape_right"] = np.asarray([305.607,-1276,463,-179.9,0.1,0.1])
        self.preset_position["escape_right"] = np.asarray([258,-1337,458,-179.9,0.1,114])
        self.preset_position["escape_left"] = np.asarray([599,1164,3,-177,-2,-173])
        rospy.loginfo("action client start")
        print 'move_arm_' + lr
        self.action_client = actionlib.SimpleActionClient('move_arm_' + lr, RobotArmMoveGlobalAction)
        self.action_client.wait_for_server()
        rospy.loginfo("%s arm action ok" % lr)
        
        
    # avoid collision between hand and shelf
    # inside_bin_check([1,2,3],'bin_C')
    @staticmethod
    def inside_bin_check(target, bin):
        #print 'inside bin check, target = ', target, 'bin = ', bin
        bin_name = bin[-1]
        #print 'target = ',target
        xlim = (0-1000, 445-600) # (650, 1045)
        
        if bin_name in ['A', 'D', 'G', 'J']:
            ylim = (-230, -15)
        elif bin_name in ['B', 'E', 'H', 'K']:
            ylim = (-285, -15)
        elif bin_name in ['C', 'F', 'I', 'L']:
            ylim = (-235, -15)
        
        if bin_name in ['A', 'B', 'C']:
            zlim = (20,210)
        elif bin_name in ['D', 'E', 'F']:
            zlim = (20,170)
        elif bin_name in ['G','H','I']:
            zlim = (20, 170)
        elif bin_name in ['J','K','L']:
            zlim = (20,210)
        reject = False
        #print target[:]
        if target[0] < xlim[0]:
            print "rejected by x< limit"
            reject = True
        if target[0] > xlim[1]:
            print "rejected by x> limit"
            reject = True
        
        if target[1] < ylim[0]: #-570:
            print "rejected by y< limit"
            reject = True
        if target[1] > ylim[1]: #-350:
            print "rejected by y> limit"
            reject = True
        if target[2] < zlim[0]:
            print "rejected by z< limit"
            reject = True
        if target[2] > zlim[1]:
            print "rejected by z> limit"
            reject = True
        
        return not reject
        
    # avoid collision between hand and tote
    # inside_bin_check([1,2,3])
    @staticmethod
    def inside_tote_check(target):
        xlim = (650, 930) 
        ylim = (390, 870) 
        zlim = (-130, 200) 
        reject = False
        
        if target[0] < xlim[0]:
            print "rejected by x< limit"
            reject = True
        if target[0] > xlim[1]:
            print "rejected by x> limit"
            reject = True
        
        if target[1] < ylim[0]:
            print "rejected by y< limit"
            reject = True
        if target[1] > ylim[1]:
            print "rejected by y> limit"
            reject = True
        if target[2] < zlim[0]:
            print "rejected by z< limit"
            reject = True
        if target[2] > zlim[1]:
            print "rejected by z> limit"
            reject = True
        
        return not reject
        
    # avoid collision between hand and shelf
    # inside_bin_check([1,2,3],'bin_C')
    @staticmethod
    def inside_bin_check_global(target, bin):
        bin_name = bin[-1]
        #print 'target = ',target
        xlim = (600, 1045) # (650, 1045)
        
        if bin_name in ['A', 'D', 'G', 'J']:
            ylim = (-195, 7)
        elif bin_name in ['B', 'E', 'H', 'K']:
            ylim = (-495, -249)
        elif bin_name in ['C', 'F', 'I', 'L']:
            ylim = (-745, -554)
        
        if bin_name in ['A', 'B', 'C']:
            zlim = (392,571)
        elif bin_name in ['D', 'E', 'F']:
            zlim = (162,309)
        elif bin_name in ['G','H','I']:
            zlim = (-68, 82)
        elif bin_name in ['J','K','L']:
            zlim = (-331,-145)
        reject = False
        #print target[:]
        if target[0] < xlim[0]:
            print "rejected by x< limit"
            reject = True
        if target[0] > xlim[1]:
            print "rejected by x> limit"
            reject = True
        
        if target[1] < ylim[0]: #-570:
            #print "rejected by y< limit"
            reject = True
        if target[1] > ylim[1]: #-350:
            #print "rejected by y> limit"
            reject = True
        if target[2] < zlim[0]:
            #print "rejected by z< limit"
            reject = True
        if target[2] > zlim[1]:
            #print "rejected by z> limit"
            reject = True
        
        return not reject

    # pos: numpy array (shape (6)) xyzwpr
    def send_msg_and_wait(self, pos, fut000 = 1, kakujiku = 1):
        print 'send message to %s arm: %s' % (self.lr, pos)
        print 'fut000=', fut000, 'kakujiku = ',kakujiku
        goal = RobotArmMoveGlobalGoal(target=FanucTwist(Vector3(pos[0],pos[1],pos[2]), Vector3(pos[3],pos[4],pos[5]), fut000, kakujiku))
        self.action_client.send_goal(goal)
        done_before_timeout = self.action_client.wait_for_result(rospy.Duration.from_sec(1400.0))

        # error check
        if not done_before_timeout:
            rospy.logerr("robot arm move did not finish in time")
            raise util.RobotTimeoutException(self.lr, goal)

        state = self.action_client.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr("robot move action did not finish correctly")
            raise util.RobotActionFailure(self.lr, goal, state)

        result = self.action_client.get_result()
        if not result.success:
            rospy.logerr("robot arm did not move correctly")
            raise util.RobotMoveFailure(self.lr, goal)

        result = self.action_client.get_result()
        status = self.action_client.get_state()
        rospy.loginfo("result %s status %s"%(result, status))
        self.current_position = pos
            
    # usage
    # move_position('bin_A', 'photo')
    def move_position(self, bin_name, fut000 = 1, kakujiku = 1, pos=''):
        rospy.loginfo("move %s %s fut = %s kakujiku = %s"%(bin_name, pos, fut000, kakujiku))
        if bin_name=='origin':
            self.send_msg_and_wait(self.preset_position['origin'], fut000, kakujiku)
        elif bin_name=='tote':
            self.send_msg_and_wait(self.preset_position['tote'], fut000, kakujiku)
        elif 'tote' in bin_name:
            self.send_msg_and_wait(self.preset_position[bin_name], fut000, kakujiku)
        else:
            x = self.preset_position['%s_%s'%(bin_name, pos)]
            self.send_msg_and_wait(x, fut000, kakujiku)
    #TODO
    # usage
    # go_to_safe_area()
    def go_to_safe_area(self):
        rospy.loginfo("move %s arm to safe area"%(self.lr))
        if self.lr=='right':
            self.send_msg_and_wait(self.preset_position['bin_F_pre'],12,0)
            self.send_msg_and_wait(self.preset_position['escape_right'],12,0)
        else:
            self.send_msg_and_wait(self.preset_position['bin_D_pre'],12,0)
            self.send_msg_and_wait(self.preset_position['escape_left'],12,0)
        msg = String()
        msg.data = "safe"
        self.pub.publish(msg)

    def return_from_safe_area(self):
        rospy.loginfo("move %s arm to work space"%(self.lr))
        if self.lr=='right':
            self.send_msg_and_wait(self.preset_position['bin_F_pre'],12,1)
        else:
            self.send_msg_and_wait(self.preset_position['bin_D_pre'],12,1)
        msg = String()
        msg.data = "workspace"
        self.pub.publish(msg)


class ArmControl(object):
    def __init__(self):
        self.left = LeftRight_arm('left')
        self.right = LeftRight_arm('right')

        self.srv_lowlevel_left = \
            actionlib.SimpleActionServer('move_to_left',
                                         RobotArmMoveAction,
                                         execute_cb=self.cb_move_to_left,
                                         auto_start=False)
        self.srv_lowlevel_left_g = \
            actionlib.SimpleActionServer('move_to_left_global',
                                         RobotArmMoveGlobalAction,
                                         execute_cb=self.cb_move_to_left_global,
                                         auto_start=False)
        self.srv_highlevel_left = \
            actionlib.SimpleActionServer('move_to_bin_left',
                                         BinToteMoveAction,
                                         execute_cb=self.cb_move_to_bin_left,
                                         auto_start=False)
        self.srv_highlevel_left_g = \
            actionlib.SimpleActionServer('move_to_bin_left_global',
                                         BinToteMoveAction,
                                         execute_cb=self.cb_move_to_bin_left_global,
                                         auto_start=False)
        
        #self.right = LeftRight_arm('right')
        self.srv_lowlevel_right = \
            actionlib.SimpleActionServer('move_to_right',
                                         RobotArmMoveAction,
                                         execute_cb=self.cb_move_to_right,
                                         auto_start=False)
        self.srv_lowlevel_right_g = \
            actionlib.SimpleActionServer('move_to_right_global',
                                         RobotArmMoveGlobalAction,
                                         execute_cb=self.cb_move_to_right_global,
                                         auto_start=False)
        self.srv_highlevel_right = \
            actionlib.SimpleActionServer('move_to_bin_right',
                                         BinToteMoveAction,
                                         execute_cb=self.cb_move_to_bin_right,
                                         auto_start=False)
        self.srv_highlevel_right_g = \
            actionlib.SimpleActionServer('move_to_bin_right_global',
                                         BinToteMoveAction,
                                         execute_cb=self.cb_move_to_bin_right_global,
                                         auto_start=False)
        
        self.srv_lowlevel_left.start()
        self.srv_lowlevel_left_g.start()
        self.srv_highlevel_left.start()
        self.srv_highlevel_left_g.start()
        self.srv_lowlevel_right.start()
        self.srv_lowlevel_right_g.start()
        self.srv_highlevel_right.start()
        self.srv_highlevel_right_g.start()
        rospy.loginfo("initialize node")
        rospy.wait_for_service('global2bin')
        rospy.wait_for_service('bin2global')
        rospy.wait_for_service('adjustglobal')
        rospy.wait_for_service('check_is_calibrated')
        rospy.loginfo('raeday to use coord transform service')
        self.global2bin = rospy.ServiceProxy('global2bin',CoordinateTransform)
        self.bin2global = rospy.ServiceProxy('bin2global',CoordinateTransform)
        self.adjustglobal = rospy.ServiceProxy('adjustglobal', CoordinateTransform)
        self.checkcalib = rospy.ServiceProxy('check_is_calibrated', CalibData)
        self.r_state = ""
        self.l_state = ""
        self.sub_l = rospy.Subscriber('left_state', String  ,self.cb_left_movement)
        self.sub_r = rospy.Subscriber('right_state', String  ,self.cb_right_movement)
        #self.l_state = 'safe'
        #self.r_state = 'safe'

    def twist2str(self, t):
        try:
            return "lin:(%s, %s, %s)/ang:(%s, %s, %s)/fut000:%s/k:%s" % (t.linear.x,
                                                                         t.linear.y,
                                                                         t.linear.z,
                                                                         t.angular.x,
                                                                         t.angular.y,
                                                                         t.angular.z,
                                                                         t.fut000,
                                                                         t.kakujiku)
        except:
            return "lin:(%s, %s, %s)/ang:(%s, %s, %s)" % (t.linear.x,
                                                          t.linear.y,
                                                          t.linear.z,
                                                          t.angular.x,
                                                          t.angular.y,
                                                          t.angular.z)

    def cb_left_movement(self, message):
        rospy.loginfo("SET LEFT STATE TO " + message.data)
        self.l_state = message.data

    def cb_right_movement(self, message):
        rospy.loginfo("SET RIGHT STATE TO " + message.data)
        self.r_state = message.data

    def left_preparation(self):
        if self.r_state == 'workspace':
            rospy.loginfo('move right arm')
            self.right.go_to_safe_area()
        if self.l_state == 'safe':
            rospy.loginfo('move left arm')
            self.left.return_from_safe_area()

    def right_preparation(self):
        #return
        if self.l_state == 'workspace':
            self.left.go_to_safe_area()
        if self.r_state == 'safe':
            self.right.return_from_safe_area()

    #RobotArmMove bin, target(twist) -> success
    def cb_move_to_left(self, goal):
        print "move_to_left:", self.twist2str(goal.target)
        try:
            self.left_preparation()
        except Exception as e:
            errmsg = "exception during left_preparation: %s" % e
            rospy.logerr(errmsg)
            result = RobotArmMoveResult(success=False)
            self.srv_lowlevel_left.set_aborted(result, errmsg)
            return

        r = self.bin2global(bin = goal.bin, point=goal.target)
        if not r.is_calibrated:
            raise util.RobotNotCalibrated("left", str(goal.target))
        t = r.point
        pos = [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]
        print 'go to', pos
        try:
            self.left.send_msg_and_wait(pos,goal.target.fut000,goal.target.kakujiku)
        except Exception as e:
            errmsg = "exception during send_msg_and_wait: %s" % e
            rospy.logerr(errmsg)
            result = RobotArmMoveResult(success=False)
            self.srv_lowlevel_left.set_aborted(result, errmsg)
            return
        if r.success == True:
            result = RobotArmMoveResult(success=True)
            self.srv_lowlevel_left.set_succeeded(result)
        else:
            result = RobotArmMoveResult(success=False)
            self.srv_lowlevel_left.set_aborted(result,"Cannot convert position")

    #RobotArmMoveGlobal target(twist)->success
    def cb_move_to_left_global(self, goal):
        print "move_to_left_global:", self.twist2str(goal.target)
        try:
            self.left_preparation()
        except Exception as e:
            errmsg = "exception during left_preparation: %s" % e
            rospy.logerr(errmsg)
            result = RobotArmMoveGlobalResult(success=False)
            self.srv_lowlevel_left_g.set_aborted(result, errmsg)
            return

        t = goal.target
        pos = [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]
        try:
            self.left.send_msg_and_wait(pos,goal.target.fut000,goal.target.kakujiku)
        except Exception as e:
            errmsg = "exception during send_msg_and_wait: %s" % e
            rospy.logerr(errmsg)
            result = RobotArmMoveGlobalResult(success=False)
            self.srv_lowlevel_left_g.set_aborted(result, errmsg)
            return
        result = RobotArmMoveGlobalResult(success=True)
        self.srv_lowlevel_left_g.set_succeeded(result)
    
    #BinToteMove bin,position(string)->success,position(twist),is_calibrated,globaL_position
    def cb_move_to_bin_left(self, goal):
        try:
            self.left_preparation()
        except Exception as e:
            errmsg = "exception during left_preparation: %s" % e
            rospy.logerr(errmsg)
            result = BinToteMoveResult(success=False)
            self.srv_highlevel_left.set_aborted(result, errmsg)
            return

        if 'tote' not in goal.bin and goal.bin not in ['bin_'+ j for j in 'ABCDEFGHIJKL']:
            result = BinToteMoveResult(success=False)
            self.srv_highlevel_left.set_aborted(result,'Invalid bin name')
        else:
            if 'tote' in goal.bin:
                try:
                    if goal.position in ['photo', 'photo_down','photo_xm','photo_xp','photo_ym','photo_yp']:
                        pos = self.left.preset_position['tote_' + goal.position]
                        self.left.move_position('tote_' + goal.position, goal.fut000, goal.kakujiku)
                    else:
                        pos = self.left.preset_position['tote']
                        self.left.move_position('tote', goal.fut000, goal.kakujiku)
                except Exception as e:
                    errmsg = "exception during move_position: %s" % e
                    rospy.logerr(errmsg)
                    result = BinToteMoveResult(success=False)
                    self.srv_highlevel_left.set_aborted(result, errmsg)
                    return
                
                p = Vector3(pos[0], pos[1], pos[2])
                r = Vector3(pos[3], pos[4], pos[5])
                is_calib = self.checkcalib()
                result = BinToteMoveResult(success=True, global_position=Twist(p, r),is_calibrated=is_calib.success, position=Twist(p, r))
                self.srv_highlevel_left.set_succeeded(result)
                
            else:
                if goal.position in ['photo', 'pre', 'rpre', 'photo_l', 'photo_r', 'front', 'right', 'left', 'pull']:
                    if goal.position == 'left':
                        po = 'photo_l'
                    elif goal.position == 'right':
                        po = 'photo_r'
                    else:
                        po = goal.position
                else:
                    po = "photo"
                name = goal.bin + '_' + po
                pos = self.left.preset_position[name]
                print "looking up position for %s/%s" % (goal.bin, pos)
                p = Vector3(pos[0], pos[1], pos[2])
                r = Vector3(pos[3], pos[4], pos[5])
                res = self.adjustglobal(bin='',point=Twist(p,r))
                if res.is_calibrated == False:
                    result = BinToteMoveResult(success=False)
                    self.srv_highlevel_left.set_aborted(result, 'No calibration yet')
                else:
                    apos = [res.point.linear.x, res.point.linear.y, res.point.linear.z, res.point.angular.x, res.point.angular.y, res.point.angular.z]
                    try:
                        self.left.send_msg_and_wait(apos,goal.fut000,goal.kakujiku)
                    except Exception as e:
                        errmsg = "exception during send_msg_and_wait: %s" % e
                        rospy.logerr(errmsg)
                        result = BinToteMoveResult(success=False)
                        self.srv_highlevel_left.set_aborted(result, errmsg)
                        return
                    p = Vector3(apos[0], apos[1], apos[2])
                    r = Vector3(apos[3], apos[4], apos[5])
                    pos = self.global2bin(bin=goal.bin, point=Twist(p,r))
                    result = BinToteMoveResult(success=True, global_position=Twist(p, r),is_calibrated=pos.is_calibrated, position=pos.point)
                    self.srv_highlevel_left.set_succeeded(result)
    
    #BinToteMove bin, position->success, position 
    def cb_move_to_bin_left_global(self, goal):
        try:
            self.left_preparation()
        except Exception as e:
            errmsg = "exception during left_preparation: %s" % e
            rospy.logerr(errmsg)
            result = BinToteMoveResult(success=False)
            self.srv_highlevel_left_g.set_aborted(result, errmsg)
            return

        try:
            if 'tote' in goal.bin:
                if goal.position in ['photo', 'photo_down','photo_xm','photo_xp','photo_ym','photo_yp']:
                    pos = self.left.preset_position['tote_' + goal.position]
                    self.left.move_position('tote_' + goal.position, goal.fut000, goal.kakujiku)
                else:
                    pos = self.left.preset_position['tote']
                    self.left.move_position('tote', goal.fut000, goal.kakujiku)
                p = Vector3(pos[0], pos[1], pos[2])
                r = Vector3(pos[3], pos[4], pos[5])
                is_calib = self.check_calib()
                result = BinToteMoveResult(success=True, global_position=Twist(p, r),is_calibrated=is_calib.success, position=Twist(p, r))
                self.srv_highlevel_left.set_succeeded(result)
 
            else:
                if goal.position in ['photo', 'pre', 'rpre', 'photo_l', 'photo_r', 'front', 'right', 'left', 'pull']:
                    if goal.position == 'left':
                        po = 'photo_l'
                    elif goal.position == 'right':
                        po = 'photo_r'
                    else:
                        po = goal.position
                else:
                    po = "photo"
                name = goal.bin + '_' + po
                pos = self.left.preset_position[name]
                self.left.move_position(goal.bin, goal.fut000, goal.kakujiku, po)
        except Exception as e:
            errmsg = "exception during move_position: %s" % e
            rospy.logerr(errmsg)
            result = BinToteMoveResult(success=False)
            self.srv_highlevel_left_g.set_aborted(result, errmsg)
            return

        print "looking up position for %s/%s" % (goal.bin, pos)
        p = Vector3(pos[0], pos[1], pos[2])
        r = Vector3(pos[3], pos[4], pos[5])

        result = BinToteMoveResult(success=True, global_position=Twist(p, r))
        self.srv_highlevel_left_g.set_succeeded(result)
 
 #RobotArmMove bin, target(twist) -> success
    def cb_move_to_right(self, goal):
        print "move_to_right:", self.twist2str(goal.target)
        try:
            self.right_preparation()
        except Exception as e:
            errmsg = "exception during right_preparation: %s" % e
            rospy.logerr(errmsg)
            result = RobotArmMoveResult(success=False)
            self.srv_lowlevel_right.set_aborted(result, errmsg)
            return

        r = self.bin2global(bin = goal.bin, point=goal.target)
        
        if not r.is_calibrated:
            raise util.RobotNotCalibrated("left", str(goal.target))
            
        t = r.point
        pos = [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]
        print 'go to', pos
        try:
            self.right.send_msg_and_wait(pos,goal.target.fut000,goal.target.kakujiku)
        except Exception as e:
            errmsg = "exception during send_msg_and_wait: %s" % e
            rospy.logerr(errmsg)
            result = RobotArmMoveResult(success=False)
            self.srv_lowlevel_right.set_aborted(result, errmsg)
            return
        if r.success == True:
            result = RobotArmMoveResult(success=True)
            self.srv_lowlevel_right.set_succeeded(result)
        else:
            result = RobotArmMoveResult(success=False)
            self.srv_lowlevel_right.set_aborted(result,"Cannot convert position")

    #RobotArmMoveGlobal target(twist)->success
    def cb_move_to_right_global(self, goal):
        print "move_to_right_global:", self.twist2str(goal.target)
        try:
            self.right_preparation()
        except Exception as e:
            errmsg = "exception during right_preparation: %s" % e
            rospy.logerr(errmsg)
            result = RobotArmMoveGlobalResult(success=False)
            self.srv_lowlevel_right_g.set_aborted(result, errmsg)
            return

        t = goal.target
        pos = [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]
        try:
            self.right.send_msg_and_wait(pos,goal.target.fut000,goal.target.kakujiku)
        except Exception as e:
            errmsg = "exception during send_msg_and_wait: %s" % e
            rospy.logerr(errmsg)
            result = RobotArmMoveGlobalResult(success=False)
            self.srv_lowlevel_right_g.set_aborted(result, errmsg)
            return
        result = RobotArmMoveGlobalResult(success=True)
        self.srv_lowlevel_right_g.set_succeeded(result)
    
    #BinToteMove bin,position(string)->success,position(twist),is_calibrated,globaL_position
    def cb_move_to_bin_right(self, goal):
        try:
            self.right_preparation()
        except Exception as e:
            errmsg = "exception during right_preparation: %s" % e
            rospy.logerr(errmsg)
            result = BinToteMoveResult(success=False)
            self.srv_highlevel_right.set_aborted(result, errmsg)
            return

        if 'tote' not in goal.bin and goal.bin not in ['bin_'+ j for j in 'ABCDEFGHIJKL']:
            result = BinToteMoveResult(success=False)
            self.srv_highlevel_right.set_aborted(result,'Invalid bin name')
        else:
            if 'tote' in goal.bin:
                try:
                    if goal.position in ['photo', 'photo_down','photo_xm','photo_xp','photo_ym','photo_yp']:
                        pos = self.right.preset_position['tote_' + goal.position] 
                        self.right.move_position('tote_' + goal.position, goal.fut000, goal.kakujiku)
                    else:
                        pos = self.right.preset_position['tote']
                        self.right.move_position('tote', goal.fut000, goal.kakujiku)
                except Exception as e:
                    errmsg = "exception during move_position: %s" % e
                    rospy.logerr(errmsg)
                    result = BinToteMoveResult(success=False)
                    self.srv_highlevel_right.set_aborted(result, errmsg)
                    return

                p = Vector3(pos[0], pos[1], pos[2])
                r = Vector3(pos[3], pos[4], pos[5])
                is_calib = self.checkcalib()
                result = BinToteMoveResult(success=True, global_position=Twist(p, r),is_calibrated=is_calib.success, position=Twist(p, r))
                self.srv_highlevel_right.set_succeeded(result)

            else:
                if goal.position in ['photo', 'pre', 'rpre', 'photo_l', 'photo_r', 'front', 'right', 'left', 'pull']:
                    if goal.position == 'left':
                        po = 'photo_l'
                    elif goal.position == 'right':
                        po = 'photo_r'
                    else:
                        po = goal.position
                else:
                    po = "photo"
                name = goal.bin + '_' + po
                pos = self.right.preset_position[name]
                print "looking up position for %s/%s" % (goal.bin, pos)
                p = Vector3(pos[0], pos[1], pos[2])
                r = Vector3(pos[3], pos[4], pos[5])
                res = self.adjustglobal(bin='',point=Twist(p,r))
                if res.is_calibrated == False:
                    result = BinToteMoveResult(success=False)
                    self.srv_highlevel_right.set_aborted(result, 'Invalid bin name')
                else:
                    apos = [res.point.linear.x, res.point.linear.y, res.point.linear.z, res.point.angular.x, res.point.angular.y, res.point.angular.z]
                    try:
                        self.right.send_msg_and_wait(apos,goal.fut000,goal.kakujiku)
                    except Exception as e:
                        errmsg = "exception during send_msg_and_wait: %s" % e
                        rospy.logerr(errmsg)
                        result = BinToteMoveResult(success=False)
                        self.srv_highlevel_right.set_aborted(result, errmsg)
                        return

                    print "looking up position for %s/%s" % (goal.bin, pos)
                    p = Vector3(apos[0], apos[1], apos[2])
                    r = Vector3(apos[3], apos[4], apos[5])
                    pos = self.global2bin(bin=goal.bin, point=Twist(p,r))
                    result = BinToteMoveResult(success=True, global_position=Twist(p, r),is_calibrated=pos.is_calibrated, position=pos.point)
                    self.srv_highlevel_right.set_succeeded(result)
    
    #BinToteMove bin, position->success, position 
    def cb_move_to_bin_right_global(self, goal):
        print 'goal = ', goal
        print "moving away left arm, then moving right arm:"
        try:
            self.right_preparation()
        except Exception as e:
            errmsg = "exception during right_preparation: %s" % e
            rospy.logerr(errmsg)
            result = BinToteMoveResult(success=False)
            self.srv_highlevel_right_g.set_aborted(result, errmsg)
            return

        try:
            if 'tote' in goal.bin:
                if goal.position in ['photo', 'photo_down','photo_xm','photo_xp','photo_ym','photo_yp']:
                    pos = self.right.preset_position['tote_' + goal.position] 
                    self.right.move_position('tote_' + goal.position, goal.fut000, goal.kakujiku)
                else:
                    pos = self.right.preset_position['tote']
                    self.right.move_position('tote', goal.fut000, goal.kakujiku)
                p = Vector3(pos[0], pos[1], pos[2])
                r = Vector3(pos[3], pos[4], pos[5])
                is_calib = self.checkcalib()
                result = BinToteMoveResult(success=True, global_position=Twist(p, r),is_calibrated=is_calib.success, position=Twist(p, r))
                self.srv_highlevel_right_g.set_succeeded(result)

            else:
                if goal.position in ['photo', 'pre', 'rpre', 'photo_l', 'photo_r', 'front', 'right', 'left', 'pull']:
                    if goal.position == 'left':
                        po = 'photo_l'
                    elif goal.position == 'right':
                        po = 'photo_r'
                    else:
                        po = goal.position
                else:
                    po = "photo"
                name = goal.bin + '_' + po
                pos = self.right.preset_position[name]
                self.right.move_position(goal.bin, goal.fut000, goal.kakujiku, po)
        except Exception as e:
            errmsg = "exception during move_position: %s" % e
            rospy.logerr(errmsg)
            result = BinToteMoveResult(success=False)
            self.srv_highlevel_right_g.set_aborted(result, errmsg)
            return

        print "looking up position for %s/%s" % (goal.bin, pos)
        p = Vector3(pos[0], pos[1], pos[2])
        r = Vector3(pos[3], pos[4], pos[5])

        result = BinToteMoveResult(success=True, global_position=Twist(p, r))
        self.srv_highlevel_right_g.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node("arm_control", anonymous=True)
    a = ArmControl()
    rospy.spin()
