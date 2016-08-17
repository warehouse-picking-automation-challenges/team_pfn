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

from util import camera_to_global, item_string_to_id

import chainer
from chainer import cuda
from chainer import serializers
from chainer import Variable
import chainer.functions as F
import chainer.links as L




from util import camera_to_global, item_string_to_id

from gripper_sensing import *
from gripper_motion import *




class GripperGrabbing(interface.GrabStrategy):
    """GripperGrabbing uses the right hand and the attached hand
    to take an item."""
    
    def __init__(self):
        
        self.grabbing_item = ''
    

        self.hp = hand_params()

        self.gs = gripper_sensing()
        self.gm = gripper_motion( self.hp )
        
        
        self.res = None
        
        self.cpu_grip_type = -1
        
    
    def _scan(self, bin, item):
    
        for i in range(3):
            #flg, res = self.gs.sensing_normal(bin, item)
            flg, res = self.gs.sensing_normal2(bin, item)
        
            if flg == True:
                break;
            else:
                if i == 2:
                    rospy.loginfo('gripper -- invalid sensing')
                    return False, None
                    
                    
        return True, res
                    
                    
    def _scan_pencilcup(self, bin, item):
    
        for i in range(3):
            #flg, res = self.gs.sensing_normal(bin, item)
            flg, res = self.gs.sensing_norma_for_pencilcup(bin, item)
        
            if flg == True:
                break;
            else:
                if i == 2:
                    rospy.loginfo('gripper -- invalid sensing')
                    return False, None
                    
        return True, res
    
    
    
    def can_grab_item(self, item, bin, items_in_bin):
        #item : str
        #bin : str  e.g.'bin_A'
        #items_in_bin : list of item str
        #move arm and 
        
        if item == 'fitness_gear_3lb_dumbbell':
        
            flg, res = self._scan(bin, item)        
               
            if flg == False:
                return False
                
                
            x,y,z, ax, ay, az = res
            
            rospy.loginfo('gripper -- dumbbell scan res flg:%s x:%d y:%d z:%d ax:%lf ay:%lf az:%lf' % ( 'True' if flg==True else 'False' , x, y, z, ax, ay, az) )
            
            if flg == False:
                return False
                
            
            if (abs(ay)+0.5) > abs(ax) and (abs(ay)+0.5) > abs(az):
                self.res = res
                return True
                
            #if abs(ax) > 0.8 and x < 100:      #flat tate 
            #    return True
                
            if (abs(az)+0.5) > abs(ax) and (abs(az)+0.5) > abs(ay): #stand
                self.res = res
                return True
                
            
        elif item == 'rolodex_jumbo_pencil_cup':
        
            valitem = ['rolodex_jumbo_pencil_cup' , 'oral_b_toothbrush_red' , 'oral_b_toothbrush_green' , 'fiskars_scissors_red',\
             'staples_index_cards' , 'womens_knit_gloves' , 'platinum_pets_dog_bowl' , 'safety_first_outlet_plugs' , 'dove_beauty_bar' ,\
              'command_hooks' , 'woods_extension_cord' , 'cool_shot_glue_sticks']
        
            
            flg, res = self._scan_pencilcup(bin, item)   

            x,y,z, ax, ay, az = res
                        
            rospy.loginfo('gripper -- pencilcup scan res flg:%s x:%d y:%d z:%d ax:%lf ay:%lf az:%lf' % ( 'True' if flg==True else 'False' , x, y, z, ax, ay, az) )
        
            print res
            print flg
        
            if flg == True:
            
                if abs(az) > abs(ay):   #tate
                    self.res = res
                    
                    
                    r = set(items_in_bin) - set(valitem)

                    print r
                    print len(r)
                    if len(r) == 0:
                    
                        self.cpu_grip_type = 1
                        
                    else:
                    
                        self.cpu_grip_type = 2
                        
                    return True       
            
        return False
        

    def grab_from_shelf(self, item, bin):
        # TODO: implement this
        rospy.loginfo( "trying to grab item \"%s\" from \"%s\"" % (item, bin))
        
        res = self.res
        
        print res
        
        x, y, z, ax, ay, az = res[0], res[1], res[2], res[3], res[4], res[5]
        
        print 'ax ay az'
        print ax, ay, az
        
        if item == 'fitness_gear_3lb_dumbbell':
        
            if (abs(ay)+0.5) > abs(ax) and (abs(ay)+0.5) > abs(az) :
                self.gm.grip_dumbbell_horizontal(bin, item, res, 'right')
                
            #elif abs(ax) > abs(ay) and abs(ax) > abs(az):
            #    if x < 100:
            #        self.gm.grip_dumbbell_vertical(bin, item, res)           
            #    else:
            #        return False
            elif (abs(az)+0.5) > abs(ax) and (abs(az)+0.5) > abs(ay):
                self.gm.grip_dumbbell_vertical_stand(bin, item, res)
            else:
                return False    
            
            
        elif item == 'rolodex_jumbo_pencil_cup':
        
            if self.cpu_grip_type == 1:
                self.gm.grip_pencilcup_one(bin,item,res)
            if self.cpu_grip_type == 2:
                self.gm.grip_pencilcup_w_someitems(bin,item,res)
                
            return True
            
            
        else:
            return False
        
        
        self.grabbing_item = item
        
        
        return True
        


    def grab_from_tote(self, item):
        # TODO: implement this
        print "trying to grab item \"%s\" from tote" % item
        
        
        flg, respose = self.gs.sensing_tote(item)
        
        if flg == False:
            rospy.loginfo('gripper -- invalid sensing')
            return False
            
        self.gm.grip_from_tote(respose)
        
        
        
        self.grabbing_item = item


        return True



    def stow_in_shelf(self, target_bin, target_position, item=None):
        
        item = self.grabbing_item
        
        self.gm.stow_in_shelf(target_bin, target_position)
        
        
        return True


    def stow_in_tote(self, target_position, item=None):
        
        item = self.grabbing_item
        
        
        self.gm.stow_in_tote(target_position)
        
        
        return True
        
        
        
