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

import sys
import rospy
import actionlib
import arm_control_wrapper
from geometry_msgs.msg import Twist, Vector3,Point
from item_locations import ItemLocations
import get_item
import numpy as np
from grab.vacuum import VacuumGrabbing
from grab.gripper import GripperGrabbing
from tote_stow_locator.random_location import RandomLocationStow
from shelf_stow_locator.always_a import AlwaysAStow
from shelf_stow_locator.random_location import RandomLocationStow as ShelfRandomLocationStow
from apc2016.srv import *
from apc2016.msg import *

def test_calibration():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted calibration process because not safe")
        print "abort"
        return
    arm_control_wrapper.move_both_hand_start()
    print "wait for calibration service"
    client = actionlib.SimpleActionClient('calibration',
                                          CalibrationAction)
    client.wait_for_server()
    print "calibrating shelf position"
    client.send_goal(CalibrationGoal())
    client.wait_for_result(rospy.Duration.from_sec(120.0))
    state = client.get_state()
    result = client.get_result()
    print "state = %s, result = %s" % (state, result)
    rospy.wait_for_service('update_calibration')
    s = rospy.ServiceProxy('update_calibration',CalibrationUpdate)
    response = s(upper_left=result.upper_left, upper_right=result.upper_right)
    #response = s(upper_left=Point(1290,52,0), upper_right=Point(1320,-843,0))
    arm_control_wrapper.move_both_hand_end()
    #result = arm_control_wrapper.move_left_arm_to_bin("bin_A","pre",keitai="nut000",kakujiku="chokusen")
    #result = arm_control_wrapper.move_left_arm_to_bin("bin_A","front",keitai="fut000",kakujiku="kakujiku")
    #for bin_key in "AL":
    #    for pos in [np.asarray([-700,0,0,90.0,-90.0,90.0])]:
    #        print "move to bin_%s%s" % (bin_key, pos)
    #        result = arm_control_wrapper.move_left_arm_local("bin_" + bin_key, pos, keitai = 'fut000', kakujiku = 'chokusen', speed = 'normal')
    #        print "result:", result
    raw_input("press enter to continue or ctrl-d to quit")

def test_arm_control_lowlevel2():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    rospy.wait_for_service('update_calibration')
    s = rospy.ServiceProxy('update_calibration',CalibrationUpdate)
    response = s(upper_left=Point(1250,35,1037), upper_right=Point(1250,-835,1037))
    # test all bins
    for bin_key in "BCDEFGHIIJKL":
        #for pos in [np.asarray([0,0,-660,89,89,89]),np.asarray([100,100,-560,89,89,89])]:
        for pos in [np.asarray([-600,-100,100,89,-89,89])]:
        #for pos in [Twist(Vector3(0,0,100),Vector3()),Twist(Vector3(100,100,150),Vector3(90,-90,90))]:
            print "move to bin_%s/%s" % (bin_key, pos)
            result = arm_control_wrapper.move_left_arm_local("bin_" + bin_key, pos)
            print "result:", result
            raw_input("press enter to continue or ctrl-c to quit ")

def test_arm_control_highlevel():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    # test all bins
    for bin_key in "IJKL":
        for pos in ["pre","photo", "left", "right","pre", "front","pre"]:
            if bin_key == 'C' and pos == 'right':
                continue;
            print "move to bin_%s/%s" % (bin_key, pos)
            result = arm_control_wrapper.move_left_arm_to_bin("bin_" + bin_key, pos)
            print "result:", result
            if raw_input("press enter to continue or q to quit: ") == "q":
                return
    # test tote
    for pos in ["photo", "photo_down",'']:
        print "move to tote/%s" % pos
        result = arm_control_wrapper.move_left_arm_to_bin("tote", pos)
        print "result:", result
        if raw_input("press enter to continue or q to quit: ") == "q":
            return

def test_arm_control_highlevel_right():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    # test all bins
    for bin_key in "ABCDEFGHIJKL":
        for pos in ["pre", "photo", "left", "right","pre", "front","pre"]:
            print "move to bin_%s/%s" % (bin_key, pos)
            result = arm_control_wrapper.move_right_arm_to_bin("bin_" + bin_key, pos)
            print "result:", result
            if raw_input("press enter to continue or q to quit: ") == "q":
                return
    # test tote
    for pos in ["photo", "photo_down",'']:
        print "move to tote/%s" % pos
        result = arm_control_wrapper.move_right_arm_to_bin("tote", pos)
        print "result:", result
        if raw_input("press enter to continue or q to quit: ") == "q":
            return
        
        
def test_arm_control_highlevel_bad_input():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    # test bad bin
    # not implemented yet
    #bin = "hoge"
    #pos = "photo"
    #print "move to %s/%s" % (bin, pos)
    #result = arm_control_wrapper.move_left_arm_to_bin(bin, pos)
    #print "result:", result
    #raw_input("press enter to continue or ctrl-c to quit ")
    # test bad position
    bin = "bin_A"
    pos = "foobar"
    print "move to %s/%s" % (bin, pos)
    result = arm_control_wrapper.move_left_arm_to_bin(bin, pos)
    print "result:", result

def test_arm_control_highlevel_bad_input_right():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    # test bad bin
    # not implemented yet
    #bin = "hoge"
    #pos = "photo"
    #print "move to %s/%s" % (bin, pos)
    #result = arm_control_wrapper.move_right_arm_to_bin(bin, pos)
    #print "result:", result
    #raw_input("press enter to continue or ctrl-c to quit ")
    # test bad position
    bin = "bin_A"
    pos = "foobar"
    print "move to %s/%s" % (bin, pos)
    result = arm_control_wrapper.move_right_arm_to_bin(bin, pos)
    print "result:", result

def test_arm_control_lowlevel_local():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    # test all bins
    for bin_key in "ABCDEFGHIJKL":
        pos = np.asarray([150,150,-200,0,0,0])
        print "move to bin_%s/%s" % (bin_key, pos)
        result = arm_control_wrapper.move_left_arm_local("bin_" + bin_key, pos)
        print "result:", result
        if raw_input("press enter to continue or q to quit: ") == "q":
            return

def test_arm_control_lowlevel_local_right():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    # test all bins
    for bin_key in "ABCDEFGHIJKL":
        pos = np.asarray([150,150,-200,0,0,0])
        print "move to bin_%s/%s" % (bin_key, pos)
        result = arm_control_wrapper.move_right_arm_local("bin_" + bin_key, pos)
        print "result:", result
        if raw_input("press enter to continue or q to quit: ") == "q":
            return
    
def test_arm_control_lowlevel():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    # some position (should be bin_A/photo)
    target = np.asarray([725,-102,720,180,-20,180])
    print "move to %s" % (target,)
    result = arm_control_wrapper.move_left_arm_global(target)
    print "result:", result


def test_arm_control_lowlevel_right():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    # some position (should be bin_A/photo)
    target = np.asarray([725,-102,720,180,-20,180])
    print "move to %s" % (target,)
    result = arm_control_wrapper.move_right_arm_global(target)
    print "result:", result




def test_vacuum_control():
    v = VacuumGrabbing()
    for (desc, fun) in [("on", v.vacuum_on), ("normal", v.vacuum_normal)]:
        print "switch vacuum to %s" % desc
        fun()
        if raw_input("press enter to continue or q to quit: ") == "q":
            return

def test_pipe_control():
    v = VacuumGrabbing()
    for (deg, fun) in [("0", v.pipe_0_deg), ("90", v.pipe_90_deg), ("0", v.pipe_0_deg)]:
        print "switch pipe to %s degrees" % deg
        fun()
        if raw_input("press enter to continue or q to quit: ") == "q":
            return

def test_can_grab_item():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    rospy.wait_for_service('update_calibration')
    s = rospy.ServiceProxy('update_calibration',CalibrationUpdate)
    response = s(upper_left=Point(1300,35,1037), upper_right=Point(1250,-835,1037))
    v = VacuumGrabbing()
    # get into position
    bin = "bin_G"
    pos = "photo"
    print "move to %s/%s" % (bin, pos)
    result = arm_control_wrapper.move_left_arm_to_bin(bin, pos, keitai='nut000', kakujiku='chokusen')
    print "result:", result
    # check if we can grab various items
    correct_inventory = ["dasani_water_bottle","elmers_washable_no_run_school_glue", "peva_shower_curtain_liner", "i_am_a_bunny_book", "dr_browns_bottle_brush", "staples_index_cards"]#"ticonderoga_12_pencils", "rawlings_baseball", "easter_turtle_sippy_cup", "fiskars_scissors_red"]
    extended_inventory = ["elmers_washable_no_run_school_glue", "dasani_water_bottle"]
    wrong_inventory = ["dasani_water_bottle"]
    for inventory in [correct_inventory, extended_inventory, wrong_inventory]:
        for item in ["elmers_washable_no_run_school_glue", "dasani_water_bottle"]:
            print "check if we can grab %s from %s" % (item, inventory)
            result = v.can_grab_item(item, bin, inventory)
            print "result:", result
            if raw_input("press enter to continue or q to quit: ") == "q":
                return
    # check how completely bogus item names work
    print "check if we can grab %s from %s" % ("hogehoge", correct_inventory)
    result = v.can_grab_item("hogehoge", bin, correct_inventory)
    print "result:", result

def test_can_grab_item_right():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    rospy.wait_for_service('update_calibration')
    s = rospy.ServiceProxy('update_calibration',CalibrationUpdate)
    response = s(upper_left=Point(1300,35,1037), upper_right=Point(1250,-835,1037))
    g = GripperGrabbing()
    # get into position
    bin = "bin_G"
    pos = "photo"
    print "move to %s/%s" % (bin, pos)
    result = arm_control_wrapper.move_right_arm_to_bin(bin, pos, keitai='nut000', kakujiku='chokusen')
    print "result:", result
    # check if we can grab various items
    correct_inventory = ["fitness_gear_3lb_dumbbell", "dasani_water_bottle"]
    print "we assume that %s contains %s" % (bin, correct_inventory)
    extended_inventory = ["dasani_water_bottle","fitness_gear_3lb_dumbbell", "peva_shower_curtain_liner", "i_am_a_bunny_book", "dr_browns_bottle_brush", "staples_index_cards"]
    wrong_inventory = ["dasani_water_bottle"]
    for inventory in [correct_inventory, extended_inventory, wrong_inventory]:
        for item in ["fitness_gear_3lb_dumbbell", "dasani_water_bottle"]:
            print "check if we can grab %s from %s" % (item, inventory)
            result = g.can_grab_item(item, bin, inventory)
            print "result:", result
            if raw_input("press enter to continue or q to quit: ") == "q":
                return
    # check how completely bogus item names work
    print "check if we can grab %s from %s" % ("hogehoge", correct_inventory)
    result = g.can_grab_item("hogehoge", bin, correct_inventory)
    print "result:", result

def test_pick():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    rospy.wait_for_service('update_calibration')
    s = rospy.ServiceProxy('update_calibration',CalibrationUpdate)
    response = s(upper_left=Point(1250,35,1037), upper_right=Point(1250,-835,1037))
    
    r = RandomLocationStow("pos_info_dummy")
    item = "jane_eyre_dvd"
    print "try to find a location for %s" % item
    target = r.find_location(item)
    print "result:", target
    if raw_input("press enter to continue or q to quit: ") == "q":
        return
    #arm_control_wrapper.move_right_arm_global(target)
    if raw_input("press enter to continue or q to quit: ") == "q":
        return
    # get into position
    bin = "bin_G"
    pos = "photo"
    print "move to %s/%s" % (bin, pos)
    #result = arm_control_wrapper.move_left_arm_global([698,-356,-260,179,0,179])
    #print "result:", result
    result = arm_control_wrapper.move_left_arm_to_bin(bin, pos, keitai='nut000', kakujiku='chokusen')
    print "result:", result
    if raw_input("press enter to continue or q to quit: ") == "q":
        return
    # check grabbability
    v = VacuumGrabbing()
    print "check if we can grab %s from %s" % (item, ["elmers_washable_no_run_school_glue", "dasani_water_bottle", "jane_eyre_dvd"])
    result = v.can_grab_item(item, bin, ["elmers_washable_no_run_school_glue", "dasani_water_bottle", "jane_eyre_dvd"])
    print "result:", result
    if raw_input("press enter to continue or q to quit: ") == "q":
        return
    # take the item
    print "take item"
    result = v.grab_from_shelf(item, bin)
    print "result:", result
    if raw_input("press enter to continue or q to quit: ") == "q":
        return
    # put it into the box
    print "stow it"
    result = v.stow_in_tote(target)
    print "result:", result

def test_pick_right():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    rospy.wait_for_service('update_calibration')
    s = rospy.ServiceProxy('update_calibration',CalibrationUpdate)
    response = s(upper_left=Point(1250,35,1037), upper_right=Point(1250,-835,1037))

    r = RandomLocationStow("pos_info_dummy")
    item = "fitness_gear_3lb_dumbbell"
    print "try to find a location for %s" % item
    target = r.find_location(item)
    print "result:", target
    if raw_input("press enter to continue or q to quit: ") == "q":
        return
    # get into position
    bin = "bin_G"
    pos = "photo"
    print "move to %s/%s" % (bin, pos)
    result = arm_control_wrapper.move_right_arm_to_bin(bin, pos, keitai='nut000', kakujiku='chokusen')
    print "result:", result
    if raw_input("press enter to continue or q to quit: ") == "q":
        return
    # check grabbability
    g = GripperGrabbing()
    all_items = ["elmers_washable_no_run_school_glue", "dasani_water_bottle", "fitness_gear_3lb_dumbbell"]
    print "check if we can grab %s from %s" % (item, all_items)
    result = g.can_grab_item(item, bin, all_items)
    print "result:", result
    if raw_input("press enter to continue or q to quit: ") == "q":
        return
    # take the item
    print "take item"
    result = g.grab_from_shelf(item, bin)
    print "result:", result
    if raw_input("press enter to continue or q to quit: ") == "q":
        return
    # put it into the box
    print "stow it"
    result = g.stow_in_tote(target)
    print "result:", result

def test_pick_bin2bin():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    arm_control_wrapper.move_both_hand_start()
    #rospy.wait_for_service('update_calibration')
    #s = rospy.ServiceProxy('update_calibration',CalibrationUpdate)
    #response = s(upper_left=Point(1250,35,1037), upper_right=Point(1250,-835,1037))
    
    items = ["laugh_out_loud_joke_book", "hanes_tube_socks", "crayola_24_ct", "oral_b_toothbrush_green"]
    #items = ["crayola_24_ct", "jane_eyre_dvd","rawlings_baseball","scotch_duct_tape","scotch_duct_tape","command_hooks","folgers_classic_roast_coffee","kyjen_squeakin_eggs_plush_puppies","cool_shot_glue_sticks"]
    #items = ["soft_white_lightbulb", "expo_dry_erase_board_eraser", "womens_knit_gloves", "dove_beauty_bar", "elmers_washable_no_run_school_glue"]#["rawlings_baseball", "staples_index_cards", "kleenex_tissue_box"] #["dasani_water_bottle","expo_dry_erase_board_eraser", "creativity_chenille_stems"] #  "elmers_washable_no_run_school_glue", "dove_beauty_bar"
    pos_info = ItemLocations({"bin_A": items},[])
    get_item_strat = get_item.MoveUntilGrabbablePicking(pos_info)

    r = RandomLocationStow("pos_info_dummy")
    item = items[0]
    print "try to find a location for %s" % item
    target = r.find_location(item)
    print "result:", target
    if raw_input("press enter to continue or q to quit: ") == "q":
        return

    # get into position
    bin = "bin_D"
    pos = "pre"
    print "move to %s/%s" % (bin, pos)
    result = arm_control_wrapper.move_left_arm_to_bin(bin, pos, keitai='nut000', kakujiku='kakujiku')
    print "result:", result
    if raw_input("press enter to continue or q to quit: ") == "q":
        return

    # now do everything to grab that item
    (success, strategy,
     moved_items) = get_item_strat.get_item(item, "bin_D", items)

    print success, strategy, moved_items
    if not success:
        print "failed to take the item"
        return

    # put it into the box
    if raw_input("press enter to continue or q to quit: ") == "q":
        return
    result = strategy.stow_in_tote(target)
    print "result:", result
    print pos_info.to_json()
    arm_control_wrapper.move_both_hand_end()

def test_stow():
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted pick process because not safe")
        print "abort"
        return
    #rospy.wait_for_service('update_calibration')
    #s = rospy.ServiceProxy('update_calibration',CalibrationUpdate)
    #response = s(upper_left=Point(1250,35,1037), upper_right=Point(1250,-835,1037))
    
    items = ["jane_eyre_dvd"]
    pos_info = ItemLocations({"bin_G": ["elmers_washable_no_run_school_glue"]}, items)

    # find stow location
    r = ShelfRandomLocationStow(pos_info)
    item = "jane_eyre_dvd"
    print "try to find a location for %s" % item
    (target_bin, target_pos) = r.find_location(item)
    print "result:", (target_bin, target_pos)
    if raw_input("press enter to continue or q to quit: ") == "q":
        return

    # move to tote
    #result = arm_control_wrapper.move_left_arm_to_bin('bin_D', 'pre', keitai='nut000', kakujiku='kakujiku')
    result = arm_control_wrapper.move_left_arm_to_bin("tote", "photo", keitai='fut000', kakujiku='kakujiku')
    if raw_input("press enter to continue or q to quit: ") == "q":
        return

    # check grabbability
    v = VacuumGrabbing()
    print "check if we can grab %s from %s" % (item, "tote")
    result = v.can_grab_item(item, "tote", items)
    print "result:", result
    if not result:
        print "did not detect item"
        return
    if raw_input("press enter to continue or q to quit: ") == "q":
        return

    # take the item
    print "take item"
    result = v.grab_from_tote(item)
    print "result:", result
    if not result:
        print "did not take item"
        return
    if raw_input("press enter to continue or q to quit: ") == "q":
        return

    # put it into the shelf
    print "stow it"
    result = v.stow_in_shelf(target_bin, target_pos)
    print "result:", result

if __name__ == "__main__":
    rospy.init_node('test_run')
    for fun in [#test_arm_control_highlevel,
    			#test_arm_control_highlevel_right,
                #test_arm_control_highlevel_bad_input,
                #test_arm_control_highlevel_bad_input_right,
                #test_arm_control_lowlevel,
                #test_arm_control_lowlevel_right,
                #test_calibration,
                test_vacuum_control,
                #test_pipe_control,
                #test_can_grab_item,
                #test_can_grab_item_right,
                #test_pick,
                #test_pick_right,
                #test_pick_bin2bin,
                #test_pick,
                #test_stow
                ]:
        #answer = raw_input("execute %s? [yN] " % fun.__name__)
        #if not answer.lower() == "y":
        #    continue
        fun()


