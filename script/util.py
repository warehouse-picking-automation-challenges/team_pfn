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
import math
import csv
import numpy as np
import os
import rospy
from geometry_msgs.msg import Twist,Vector3
from apc2016.msg import *


bin_3side = {
    "bin_A": [420,250,240],
    "bin_B": [420,300,240],
    "bin_C": [420,250,240],
    
    "bin_D": [420,250,200],
    "bin_E": [420,300,200],
    "bin_F": [420,250,200],
    
    "bin_G": [420,250,200],
    "bin_H": [420,300,200],
    "bin_I": [420,250,200],
    
    "bin_J": [420,250,240],
    "bin_K": [420,300,240],
    "bin_L": [420,250,240]
}

force_retry_items = ["oral_b_toothbrush_red", "oral_b_toothbrush_green", "fiskars_scissors_red"]
picopico = ["cloud_b_plush_bear", "womens_knit_gloves", "cherokee_easy_tee_shirt"]


class RobotTimeoutException(Exception):
    def __init__(self, arm, goal, *args, **kwargs):
        Exception.__init__(self,
                           "timeout while moving %s arm to %s" % (arm, goal),
                           *args,
                           **kwargs)


class RobotActionFailure(Exception):
    def __init__(self, arm, goal, status, *args, **kwargs):
        Exception.__init__(self,
                           "action failure (status=%d) while moving %s arm to %s" %
                           (status, arm, goal),
                           *args,
                           **kwargs)


class RobotMoveFailure(Exception):
    def __init__(self, arm, goal, *args, **kwargs):
        Exception.__init__(self,
                           "failure while moving %s arm to %s" %
                           (arm, goal),
                           *args,
                           **kwargs)


class RobotNotCalibrated(Exception):
    def __init__(self, arm, goal, *args, **kwargs):
        Exception.__init__(self,
                           "failure while moving %s arm to %s" %
                           (arm, goal),
                           *args,
                           **kwargs)


class TooCrowdedTryLater(Exception):
    pass


CSV_FILE = "%s/../item_data.csv" % os.path.dirname(os.path.abspath(__file__))
CSV_DATA = list(csv.reader(open(CSV_FILE, "r")))


# zatsu
def read_item_csv(item_id, col_idx):
    cnt = 0
    for row in CSV_DATA:
        if cnt==item_id:
            return row[col_idx]
        cnt += 1


def item_string_to_id(item):
    cnt = 0
    for row in CSV_DATA:
        if row[1] == item:
            return cnt
        cnt += 1
    raise ValueError("item %s not found" % item)
    
def item_id_to_string(item_id):
    return read_item_csv(item_id, 1)

def item_id_to_volume(item_id):
    return float(read_item_csv(item_id, 5))

def item_id_to_3side(item_id):
    return [ float(read_item_csv(item_id, 6)), float(read_item_csv(item_id, 7)), float(read_item_csv(item_id, 8)) ]

def item_id_to_volume_with_margin(item_id,margin):
    l = item_id_to_3side(item_id)
    return (l[0]+1) * (l[1]+1) * ( l[2] + margin )

def item_string_to_3side(item_string):
    return item_id_to_3side( item_string_to_id(item_string) )

def item_string_to_volume(item_string):
    return item_id_to_volume(item_string_to_id(item_string))

def item_string_to_volume_with_margin(item_string,margin):
    return item_id_to_volume_with_margin(item_string_to_id(item_string), margin)
    
def get_smallest_item(items):
    return min(items,
               key=lambda x: item_string_to_volume(x))

def item_string_to_bonus(item_string):
    return int(read_item_csv(item_string_to_id(item_string), 4))

def item_string_to_difficulty(item_string):
    return int(read_item_csv(item_string_to_id(item_string), 9))

def get_smallest_item_with_margin(items,margin=5,not_stow=["fitness_gear_3lb_dumbbell"] ):
    item_list = items[:]
    for not_stow_item in not_stow:
        if not_stow_item in item_list:
            item_list.remove(not_stow_item)
    if "scotch_duct_tape" in item_list:
        if len(item_list)==1:
            return item_list[0]
        else:
            item_list.remove("scotch_duct_tape")
    return min(item_list,
               key=lambda x: item_string_to_volume_with_margin(x,margin))

def get_try_smallest_item_with_margin(tries_items,margin=5,not_stow=["fitness_gear_3lb_dumbbell"] ):
    item_list = []
    for (try_num, item) in tries_items:
        if not item in not_stow:
            item_list.append((try_num, item))
    return min(item_list,
               key=lambda x: (x[0], item_string_to_volume_with_margin(x[1],margin)))

def get_largest_item(items):
    return min(items,
               key=lambda x: -item_string_to_volume(x))
 

def get_item_dict():
    cnt = 0
    item_name = {}
    for row in CSV_DATA:
        item_name[row[1]] = cnt
        cnt += 1
    return item_name
    

def all_items():
    item_name = [row[1] for row in CSV_DATA]
    return item_name[1:]
    
    
def applyM(x):
    mat = np.asarray([[0,0,-1], [1,0,0], [0,1,0]]).astype('f')
    return np.dot(mat, x)
    
def applyWPR(x, w, p, r):
    w = w * 2.0 * math.pi / 360
    p = p * 2.0 * math.pi / 360
    r = r * 2.0 * math.pi / 360
    mat_w = np.asarray([[1,0,0], [0,math.cos(w),-math.sin(w)], [0,math.sin(w),math.cos(w)]]).astype('f')
    mat_p = np.asarray([[math.cos(p),0,math.sin(p)], [0,1,0], [-math.sin(p),0,math.cos(p)]]).astype('f')
    mat_r = np.asarray([[math.cos(r),-math.sin(r),0], [math.sin(r),math.cos(r),0], [0,0,1]]).astype('f')

    return np.dot(mat_r,np.dot(mat_p,np.dot(mat_w,x)))

def camera_to_global(xyz_im, xyzwpr_robot):
    #print 'camera_to_global', xyz_im, xyzwpr_robot
    oc = np.asarray([15, 0, 160]).astype('f')   # defference between flange and realsense
    x_robot, y_robot, z_robot, w_robot, p_robot, r_robot = xyzwpr_robot
    rx = np.asarray([x_robot, y_robot, z_robot]).astype('f')
    if xyz_im.ndim==2:
        oc = oc.reshape((3,1))
        rx = rx.reshape((3,1))
    gx = applyWPR(applyM(xyz_im+oc), w_robot, p_robot, r_robot) + rx
    
    #hosei_x, hosei_y, hosei_z = 673-1121+50-128,-141+143+33,395-509+36
    return gx

def camera_to_global_right(xyz_im, xyzwpr_robot):
    #print 'camera_to_global', xyz_im, xyzwpr_robot
    oc = np.asarray([0, 0, 205]).astype('f')   # defference between flange and realsense
    x_robot, y_robot, z_robot, w_robot, p_robot, r_robot = xyzwpr_robot
    rx = np.asarray([x_robot, y_robot, z_robot]).astype('f')
    if xyz_im.ndim==2:
        oc = oc.reshape((3,1))
        rx = rx.reshape((3,1))
    gx = applyWPR(applyM(xyz_im+oc), w_robot, p_robot, r_robot) + rx
    
    #hosei_x, hosei_y, hosei_z = 673-1121+50-128,-141+143+33,395-509+36
    return gx
# TODO faster impl

def camera_to_global_all(xyz, xyzwpr_robot):
    #print xyzwpr_robot
    #print type(xyzwpr_robot)
    # xyz: (3, 480, 640)
    
    ret = camera_to_global(xyz.reshape((3, 480*640)), xyzwpr_robot).reshape((3,480,640))
    return ret

def camera_to_global_right_all(xyz, xyzwpr_robot):
    #print xyzwpr_robot
    #print type(xyzwpr_robot)
    # xyz: (3, n)
    
    ret = camera_to_global_right(xyz, xyzwpr_robot)
    return ret

def twist2array(t):
    return np.asarray([t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z])

def array2twist(a):
    return Twist(Vector3(a[0],a[1],a[2]), Vector3(a[3],a[4],a[5]))

def array2ftwist(a,b,c):
    return FanucTwist(Vector3(a[0],a[1],a[2]), Vector3(a[3],a[4],a[5]),b ,c)

def get_loggers(prefix):
    debug = lambda s: rospy.logdebug("[%s] %s" % (prefix, s))
    info = lambda s: rospy.loginfo("[%s] %s" % (prefix, s))
    warn = lambda s: rospy.logwarn("[%s] %s" % (prefix, s))
    err = lambda s: rospy.logerr("[%s] %s" % (prefix, s))
    return (debug, info, warn, err)

def get_elapsed_time(start):
    elapsed = time.time() - start
    minutes = int(elapsed) / 60
    seconds = int(elapsed) % 60
    return "%d:%02d" % (minutes, seconds)

def write_output_file(pos_info, out_file):
    # write output file
    result = pos_info.to_json()
    if out_file == "-":
        print result
    else:
        with open(out_file, "w") as f:
            f.write(result + "\n")

_use_gripper = True

def is_gripper_enabled():
    global _use_gripper
    return _use_gripper

def set_gripper_enabled(new_val):
    global _use_gripper
    if new_val:
        rospy.loginfo("using gripper, too")
    else:
        rospy.loginfo("disabled gripper")
    _use_gripper = new_val
    _use_gripper
