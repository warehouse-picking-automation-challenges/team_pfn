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
import sys
import rospy
import threading
import copy
import math
import actionlib
import arm_control_wrapper
import util
from geometry_msgs.msg import Twist, Vector3, Point
import numpy as np
from apc2016.srv import *
from apc2016.msg import *
import math



class arm_inst:
    
    fut_dict = {'current':0, 'fut000':1, 'nut000':2}
    kak_dict = {'kakujiku':1, 'chokusen':0}
    #keitai = 'fut000'
    keitai = 'nut000'
    #kakujiku='chokusen'
    kakujiku = 'kakujiku'
    
    
    def __init__(self):
        self.which = ''
        self.sensor_offset_x = 0
        self.sensor_offset_y = 0
        self.sensor_offset_z = 0
        
        self.hand_angle_x = 0
        self.hand_angle_y = 0
        self.hand_angle_z = 0
        
        self.arm_cl = None
        self.snsr_sv = None
        
    def arm_move(self, p):
        gl = RobotArmMoveGlobalGoal()
        gl.target = util.array2ftwist(p,self.fut_dict[self.keitai], self.kak_dict[self.kakujiku])
        self.arm_cl.send_goal(gl)
        
    def wait_arm_move(self):
        self.arm_cl.wait_for_result(rospy.Duration.from_sec(320.0))
        
        

class calib_params:
    def __init__(self):
        self.scan_z = 0
        self.scan_y = 0
        self.scan_x_f = 0
        self.scan_x_e = 0
        
        self.scan_x_step1 = 0   #rough
        self.scan_x_step2 = 0   #detail



class calib_arm_thread(threading.Thread):
    def __init__(self, arm_inst, calib_params):
        threading.Thread.__init__(self)
        
        self.arm_inst = arm_inst
        self.calib_params = calib_params
        
        self.res_x = -1
        self.res_y = -1
        
        self.success = False
        
    def scan_proc(self, x_f, x_e, stp, fwd):
        
        z = self.calib_params.scan_z
        y = self.calib_params.scan_y
        
        
        ist = self.arm_inst
        
        ox = self.arm_inst.sensor_offset_x
        oy = self.arm_inst.sensor_offset_y
        oz = self.arm_inst.sensor_offset_z
        
        ax = self.arm_inst.hand_angle_x
        ay = self.arm_inst.hand_angle_y
        az = self.arm_inst.hand_angle_z
        
        y_ = y-oy
        z_ = z-oz
        
        pos = np.asarray([0,y_,z_,ax,ay,az])
        
        print 'pos:'
        print pos
        print z, y, ox, oy, oz
        
        print x_f , x_e, stp
        #raw_input('stop')
        
        list1 = []
        #rough iki
        for x in range(x_f, x_e, stp):

            print' '
            print x
            #move
            pos[0] = x - ox
            print pos
            ist.arm_move(pos)
            ist.wait_arm_move()
            print 'moved'
            #time.sleep(0.3)

            #read and valid check
            vn = 4
            vl = []
            vavg=0.0
            for c in range(vn):
                print 'dt call'
                
                while True:
                    res = self.arm_inst.snsr_sv()  #get dist sensor value
                    #res = snsr_r()
                    v_ = res.dist
                    
                    if v_ != -1:
                        break
                
                
                rospy.loginfo('%s v:%d' % (self.arm_inst.which, v_))
                
                vavg = vavg + v_
                vl.append(v_)
            vavg = vavg / vn
            rospy.loginfo('%s vavg:%lf' % (self.arm_inst.which, vavg))
            
            valid = False
            for c in range(vn):
                if vl[c] >= 200:    #overflow
                    break
                    
                if vl[0] != vl[c]:
                    valid = True
            
            if valid == True:
                list1.append((vavg, x))
                if fwd == True:
                    break
            else:
                list1.append((-1, x))
                if fwd == False:
                    break
                
            rospy.loginfo('%s fwd:%s (%d,%d)' % (self.arm_inst.which, 'True' if fwd==True else 'False', list1[-1][0], list1[-1][1]))
            
        
        #decide range
        if fwd == True:
            if list1[0][0] >= 0:    #can't detect no shelf pos
                rospy.loginfo('%s cant detect no shelf pos' % (self.arm_inst.which))
                return -1, -1, -1, -1
        else:
            if list1[0][0] == -1:    #can't detect no shelf pos
                rospy.loginfo('%s cant detect shelf pos' % (self.arm_inst.which))
                return -1, -1, -1, -1
                
        if fwd == True:    
            for i, v in enumerate(list1):
                if i+1 == len(list1):   #can't detect shelf pos
                    rospy.loginfo('%s cant detect shelf pos' % (self.arm_inst.which))
                    return -1, -1, -1, -1
                    
                if list1[i+1][0] >= 0:
                    rx_f = list1[i][1]
                    v_f  = list1[i][0]
                    rx_e = list1[i+1][1]
                    v_e  = list1[i+1][0]
                    
                    return rx_f, v_f, rx_e, v_e
                    
        else:

            n = 0

            ax = 0.0
            ax2 = 0.0
            
            #more detail mode (fwd == false)
            for i, v in enumerate(list1):
                if i+1 == len(list1):   #can't detect shelf pos
                    rospy.loginfo('%s cant detect no shelf pos' % (self.arm_inst.which))
                    return -1, -1, -1, -1
                   
                v2 = list1[i+1]
                
                stde = 1000
                d = 0
                if v[0] > 0:
                    d = v[0]
                    n = n+1
                    ax = ax + d
                    ax2 = ax2 + d*d
                    
                    stde = math.sqrt( ax2/n - (ax/n)*(ax/n))
                    

                rospy.loginfo('proc2 %s v2:%lf d:%lf avg:%lf stde:%lf' % (self.arm_inst.which, v2[0], v2[0]-ax/n, ax/n, stde))
                
                if v2[0] == -1 or (n>=3 and v2[0] > 0 and abs(v2[0]-ax/n) > 3*stde):

                    rx_f = v[1]
                    v_f  = v[0]
                    rx_e = v2[1]
                    v_e  = v2[0]

                    
                    return rx_f, v_f, rx_e, v_e

        
                    
                    
                    
    def move_to_first_pos(self):
        z = self.calib_params.scan_z
        y = self.calib_params.scan_y
        
        
        ox = self.arm_inst.sensor_offset_x
        oy = self.arm_inst.sensor_offset_y
        oz = self.arm_inst.sensor_offset_z
        
        ax = self.arm_inst.hand_angle_x
        ay = self.arm_inst.hand_angle_y
        az = self.arm_inst.hand_angle_z
        
        x_ = 800 - ox
        y_ = y-oy
        z_ = z-oz
        
        print 'move to first'
        pos = np.asarray([x_,y_,z_,ax,ay,az])
        self.arm_inst.arm_move(pos)
        self.arm_inst.wait_arm_move()
        
        
    def run(self):
        
        x_f = self.calib_params.scan_x_f
        x_e = self.calib_params.scan_x_e
        stp1 = self.calib_params.scan_x_step1
        stp2 = self.calib_params.scan_x_step2
        
        rospy.loginfo('proc1 %s' % self.arm_inst.which)
        #raw_input('stop')
        rx_f, v_f, rx_e, v_e = self.scan_proc(x_f, x_e, stp1, True)
        
        rospy.loginfo('proc1 %s, %lf %lf %lf %lf' % (self.arm_inst.which , rx_f, v_f, rx_e, v_e))
        #raw_input('stop')
        
        if v_e < 0:
            rospy.loginfo('%s no shelf edge in scan area' % (self.arm_inst.which))
            self.move_to_first_pos()
            return
            
        rospy.loginfo('proc2 %s' % self.arm_inst.which)
        res = self.scan_proc(rx_e+stp1, rx_f-stp1, -stp2, False)
        print res
        rx_f2, v_f2, rx_e2, v_e2 = res
        
        rospy.loginfo('proc2 %s, %lf %lf %lf %lf' % (self.arm_inst.which , rx_f2, v_f2, rx_e2, v_e2))
        
        if v_f2 != -1:
            self.res_x = rx_f2
            if self.arm_inst.which == 'right':
                self.res_y = self.calib_params.scan_y + v_f2
            else:
                self.res_y = self.calib_params.scan_y - v_f2
        
        rospy.loginfo('%s res: %lf %lf (sv:%d)' % (self.arm_inst.which , self.res_x , self.res_y, v_f2))


        self.move_to_first_pos()
        
        
        self.success = True
        

class Calibration(object):

    def __init__(self):

        self.arm_r = actionlib.SimpleActionClient('move_to_right_global', RobotArmMoveGlobalAction)
        self.arm_l = actionlib.SimpleActionClient('move_to_left_global', RobotArmMoveGlobalAction)
        
        rospy.loginfo('wait for distance_sensor_right')
        rospy.wait_for_service('distance_sensor_right')
        rospy.loginfo('wait for distance_sensor_left')
        rospy.wait_for_service('distance_sensor_left')
        self.snsr_r = rospy.ServiceProxy('distance_sensor_right', distance)
        self.snsr_l = rospy.ServiceProxy('distance_sensor_left', distance)
        
        rospy.loginfo('wait for move_to_right_global')
        self.arm_r.wait_for_server()
        rospy.loginfo('wait for move_to_left_global')
        self.arm_l.wait_for_server()
        rospy.loginfo('ok!')
        
        
        #arm insts
        self.arm_inst_r = arm_inst()
        self.arm_inst_r.which = 'right'
        self.arm_inst_r.sensor_offset_x = 203
        self.arm_inst_r.sensor_offset_y = 86
        self.arm_inst_r.sensor_offset_z = -5    #not used
        self.arm_inst_r.hand_angle_x = 180
        self.arm_inst_r.hand_angle_y = 0
        self.arm_inst_r.hand_angle_z = 180
        self.arm_inst_r.arm_cl = self.arm_r
        self.arm_inst_r.snsr_sv = self.snsr_r
        
        self.arm_inst_l = arm_inst()
        self.arm_inst_l.which = 'left'
        self.arm_inst_l.sensor_offset_x = 178
        self.arm_inst_l.sensor_offset_y = -86
        self.arm_inst_l.sensor_offset_z = 3
        self.arm_inst_l.hand_angle_x = 180
        self.arm_inst_l.hand_angle_y = 0
        self.arm_inst_l.hand_angle_z = 180
        self.arm_inst_l.arm_cl = self.arm_l
        self.arm_inst_l.snsr_sv = self.snsr_l
        
        
        #calib params
        self.calib_params_r = calib_params()
        self.calib_params_r.scan_x_f = 1170 #1250 center
        self.calib_params_r.scan_x_e = 1330
        self.calib_params_r.scan_x_step1 = 10
        self.calib_params_r.scan_x_step2 = 2
        
        self.calib_params_l = copy.deepcopy(self.calib_params_r)   #takasa okuyuki_hanni onaji 
        
        self.calib_params_r.scan_z = 235    #takasa
        self.calib_params_r.scan_y = -835 - 70      #shelf W = 870
        self.calib_params_l.scan_z = 256    #takasa
        self.calib_params_l.scan_y = 35 + 70
        
        
        #start service
        self.calib_server = actionlib.SimpleActionServer(
            'calibration', CalibrationAction,
            execute_cb=self.calibrate,
            auto_start=False)
        self.calib_server.start()
        rospy.loginfo('calibration server start')
        
    def calibrate(self, goal):
        
        rospy.loginfo('calibration')
        rospy.loginfo('start both move')
        arm_control_wrapper.move_both_hand_start()
        
        th_r = calib_arm_thread(self.arm_inst_r, self.calib_params_r)
        th_l = calib_arm_thread(self.arm_inst_l, self.calib_params_l)
        
        th_r.start()
        th_l.start()
        
        th_r.join()
        th_l.join()
        
        arm_control_wrapper.move_both_hand_end()
        rospy.loginfo('stop both move')
        
        if th_r.success == True and th_l.success == True:
            
            result = CalibrationResult(success=True,
                upper_left=Point(th_l.res_x, th_l.res_y, 0),
                upper_right=Point(th_r.res_x, th_r.res_y, 0))
        else:
            result = CalibrationResult(success=False,
                upper_left=Point(0, 0, 0),
                upper_right=Point(0, 0, 0))
                   
        self.calib_server.set_succeeded(result)
        
        rospy.loginfo('finish calibration')
        
        
if __name__ == '__main__':
    rospy.init_node("calibration", anonymous=True)
    Calibration()
    rospy.spin()