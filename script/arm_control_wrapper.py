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

# In this file are functions that are a proxy to the arm_control
# node for convenience.

import sys

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, Vector3
from apc2016.msg import FanucTwist
from apc2016.msg import BinToteMoveAction, BinToteMoveGoal
from apc2016.msg import RobotArmMoveAction, RobotArmMoveGoal
from apc2016.msg import RobotArmMoveGlobalAction, RobotArmMoveGlobalGoal
import util
from std_msgs.msg import String

sp_dict = {'fast':1, 'normal':0}
fut_dict = {'current':0, 'fut000':1, 'nut000':2}
kak_dict = {'kakujiku':1, 'chokusen':0}

def move_both_hand_start():
    pub_l = rospy.Publisher('left_state',String, queue_size=10, latch=True)
    msg = String()
    msg.data = "calibration"
    pub_l.publish(msg)
    pub_r = rospy.Publisher('right_state',String, queue_size=10, latch=True)
    msg = String()
    msg.data = "calibration"
    pub_r.publish(msg)


def move_both_hand_end():
    pub_l = rospy.Publisher('left_state',String, queue_size=10, latch=True)
    msg = String()
    msg.data = "workspace"
    pub_l.publish(msg)
    pub_r = rospy.Publisher('right_state',String, queue_size=10, latch=True)
    msg = String()
    msg.data = "workspace"
    pub_r.publish(msg)


def change_fut(keitai, speed):
    return sp_dict[speed] * 10 + fut_dict[keitai]

def move_left_arm_to_bin(bin, position, keitai, kakujiku, speed = 'normal'):
    """Wrapper for the arm_control/move_to_bin_left action.
    After maybe moving away the right arm, move the left
    arm to the specified bin's relative position and return
    (success, bin_coord, is_calibrated, global_coord) tuple.
    If the is_calibrated flag is False, the bin coordinates
    may not be accurate. If success is False, the other
    values must not be used, they are all None."""
    return _move_arm_to_bin("left", bin, position,keitai,kakujiku,speed)


def move_right_arm_to_bin(bin, position, keitai = 'fut000',kakujiku = 'chokusen', speed = 'normal'):
    """Wrapper for the arm_control/move_to_bin_right action.
    After maybe moving away the left arm, move the right
    arm to the specified bin's relative position and return
    (success, bin_coord, is_calibrated, global_coord) tuple.
    If the is_calibrated flag is False, the bin coordinates
    may not be accurate. If success is False, the other
    values must not be used, they are all None."""
    return _move_arm_to_bin("right", bin, position,keitai,kakujiku, speed)


def _move_arm_to_bin(which, bin, position,keitai, kakujiku, speed = 'normal'):
    # create and send the goal
    client = actionlib.SimpleActionClient('move_to_bin_' + which,
                                          BinToteMoveAction)
    client.wait_for_server()
    goal = BinToteMoveGoal(bin=bin, position=position,
                           fut000=change_fut(keitai, speed), kakujiku=kak_dict[kakujiku])
    client.send_goal(goal)
    done_before_timeout = client.wait_for_result(rospy.Duration.from_sec(1600.0))

    # error check
    if not done_before_timeout:
        rospy.logerr("robot arm move did not finish in time")
        raise util.RobotTimeoutException(which, goal)

    state = client.get_state()
    if state != GoalStatus.SUCCEEDED:
        rospy.logerr("robot move action did not finish correctly")
        raise util.RobotActionFailure(which, goal, state)

    result = client.get_result()
    if not result.success:
        rospy.logerr("robot arm did not move correctly")
        raise util.RobotMoveFailure(which, goal)

    return (result.success,
            util.twist2array(result.position),
            result.is_calibrated,
            util.twist2array(result.global_position))

def move_left_arm_to_bin_global(bin, position,keitai,kakujiku, speed = 'normal'):
    """Wrapper for the arm_control/move_to_bin_left action.
    After maybe moving away the right arm, move the left
    arm to the specified bin's relative position and return
    (success, bin_coord, is_calibrated, global_coord) tuple.
    If the is_calibrated flag is False, the bin coordinates
    may not be accurate. If success is False, the other
    values must not be used, they are all None."""
    return _move_arm_to_bin_global("left", bin, position,keitai,kakujiku, speed)


def move_right_arm_to_bin_global(bin, position, keitai = 'fut000',kakujiku = 'chokusen',speed = 'normal'):
    """Wrapper for the arm_control/move_to_bin_right action.
    After maybe moving away the left arm, move the right
    arm to the specified bin's relative position and return
    (success, bin_coord, is_calibrated, global_coord) tuple.
    If the is_calibrated flag is False, the bin coordinates
    may not be accurate. If success is False, the other
    values must not be used, they are all None."""
    return _move_arm_to_bin_global("right", bin, position,keitai,kakujiku, speed)


def _move_arm_to_bin_global(which, bin, position,keitai, kakujiku, speed):
    # create and send the goal
    client = actionlib.SimpleActionClient('move_to_bin_' + which + '_global',
                                          BinToteMoveAction)
    client.wait_for_server()
    goal = BinToteMoveGoal(bin=bin, position=position,
                           fut000=change_fut(keitai, speed), kakujiku=kak_dict[kakujiku])
    client.send_goal(goal)
    done_before_timeout = client.wait_for_result(rospy.Duration.from_sec(1600.0))

    # error check
    if not done_before_timeout:
        rospy.logerr("robot arm move did not finish in time")
        raise util.RobotTimeoutException(which, goal)

    state = client.get_state()
    if state != GoalStatus.SUCCEEDED:
        rospy.logerr("robot move action did not finish correctly")
        raise util.RobotActionFailure(which, goal, state)

    result = client.get_result()
    if not result.success:
        rospy.logerr("robot arm did not move correctly")
        raise util.RobotMoveFailure(which, goal)

    return (result.success,
            util.twist2array(result.position),
            result.is_calibrated,
            util.twist2array(result.global_position))


def move_left_arm_global(p,keitai,kakujiku, speed = 'normal'):
    """Wrapper for the arm_control/move_to_global_left action.
    After maybe moving away the right arm, move the left
    arm to the specified global coordinates IN THE LEFT ARM'S
    COORDINATE SYSTEM and return a boolean success flag."""
    return _move_arm_global("left", p, keitai, kakujiku, speed)


def move_right_arm_global(p,keitai = 'fut000',kakujiku='chokusen', speed = 'normal'):
    """Wrapper for the arm_control/move_to_global_right action.
    After maybe moving away the left arm, move the right
    arm to the specified global coordinates IN THE RIGHT ARM'S
    COORDINATE SYSTEM and return a boolean success flag."""
    return _move_arm_global("right", p, keitai, kakujiku, speed)


def _move_arm_global(which, p, keitai, kakujiku, speed):
    rospy.loginfo("move %s arm to %s" % (which, p))
    # create and send the goal
    client = actionlib.SimpleActionClient('move_to_%s_global'%which,
                                          RobotArmMoveGlobalAction)
    client.wait_for_server()
    goal = RobotArmMoveGlobalGoal()
    goal.target = util.array2ftwist(p, change_fut(keitai, speed), kak_dict[kakujiku])
    client.send_goal(goal)
    done_before_timeout = client.wait_for_result(rospy.Duration.from_sec(1600.0))

    # error check
    if not done_before_timeout:
        rospy.logerr("robot arm move did not finish in time")
        raise util.RobotTimeoutException(which, goal)

    state = client.get_state()
    if state != GoalStatus.SUCCEEDED:
        rospy.logerr("robot move action did not finish correctly")
        raise util.RobotActionFailure(which, goal, state)

    result = client.get_result()
    if not result.success:
        rospy.logerr("robot arm did not move correctly")
        raise util.RobotMoveFailure(which, goal)

    return True


def move_left_arm_local(bin, p, keitai, kakujiku, speed = 'normal'):
    """Wrapper for the arm_control/move_to_left action.
    After maybe moving away the right arm, move the left
    arm to the specified bin's local coordinates and
    return a boolean success flag."""
    return _move_arm_local("left", bin, p, keitai, kakujiku, speed = 'normal')


def move_right_arm_local(bin, p, keitai = 'fut000', kakujiku = 'chokusen', speed = 'normal'):
    """Wrapper for the arm_control/move_to_right action.
    After maybe moving away the left arm, move the right
    arm to the specified bin's local coordinates and
    return a boolean success flag."""
    return _move_arm_local("right", bin, p, keitai, kakujiku, speed = 'normal')


def _move_arm_local(which, bin, p, keitai, kakujiku, speed):
    rospy.loginfo("move %s arm to %s's %s" % (which, bin, p))
    # create and send the goal
    client = actionlib.SimpleActionClient('move_to_' + which,
                                          RobotArmMoveAction)
    client.wait_for_server()
    goal = RobotArmMoveGoal()
    goal.bin = bin
    goal.target = util.array2ftwist(p, change_fut(keitai, speed), kak_dict[kakujiku])
    client.send_goal(goal)
    done_before_timeout = client.wait_for_result(rospy.Duration.from_sec(1600.0))

    # error check
    if not done_before_timeout:
        rospy.logerr("robot arm move did not finish in time")
        raise util.RobotTimeoutException(which, goal)

    state = client.get_state()
    if state != GoalStatus.SUCCEEDED:
        rospy.logerr("robot move action did not finish correctly")
        raise util.RobotActionFailure(which, goal, state)

    result = client.get_result()
    if not result.success:
        rospy.logerr("robot arm did not move correctly")
        raise util.RobotMoveFailure(which, goal)

    return True
