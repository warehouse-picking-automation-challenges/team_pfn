#! /usr/bin/env python

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

# For documentation of ros actionlib server, see
# - <http://wiki.ros.org/actionlib#Python_SimpleActionServer>
# - <http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29>
# - <http://docs.ros.org/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html>

import sys
import time

import roslib
import rospy
import actionlib
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

from apc2016.msg import *

import controller


class TurnPipeServer:
    def __init__(self, controller):
        self.controller = controller
        self.server = actionlib.SimpleActionServer('turn_pipe',
                                                   GenericAction,
                                                   self.execute,
                                                   False)
        self.server.start()
        # move pipe to straight position after start
        time.sleep(1)
        self.controller.move_to_preset(1)

    def execute(self, goal):
        log = lambda s: rospy.loginfo("<PIPE> " + s)
        log("got %s" % goal)

        # unpack
        try:
            IN_pos_idx = int(goal.target)
        except Exception as e:
            res = GenericResult(error="target must be an integer value")
            self.server.set_aborted(res)
            return

        # check parameter validity
        if IN_pos_idx == -1:
            return self.servo_off(log)
        elif IN_pos_idx == -2:
            return self.servo_on(log)
        elif not IN_pos_idx in range(7):
            res = GenericResult(error="pos_idx must be in %s" % range(7))
            self.server.set_aborted(res)
            return

        # move actuator
        try:
            log("moving actuator to %s" % IN_pos_idx)
            status = self.controller.move_to_preset(IN_pos_idx)
            log("status: " + str(status))
        except Exception as e:
            res = GenericResult(error="exception while moving: %s" % e)
            self.server.set_aborted(res)
            return

        error = ""

        # wait while one of the actuators is moving
        MOVE = status["MOVE"]
        while MOVE:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                log("goal preempted")
                return

            # check whether actuator is still moving
            try:
                status = self.controller.status()
            except Exception as e:
                error = "while waiting for move to finish: %s" % e
                # at this point, we cannot determine whether
                # the servo is still moving or not. internally
                # there have already been a number of retries,
                # so we can assume communication is somehow
                # broken and return with an error
                break
            MOVE = status["MOVE"]

            # emit feedback
            try:
                PNOW = self.controller.read_PNOW()
                FB_moving = MOVE
                FB_pos = PNOW.int*0.01
                fb = GenericFeedback(status=("%d %.2f" % (FB_moving, FB_pos)))
                self.server.publish_feedback(fb)
            except Exception as e:
                log("got exception \"%s\", skip feedback" % e)

            # sleep a bit
            time.sleep(0.1)

        # get current position
        pos_idx = 0
        pos = 0.0
        try:
            POSR = self.controller.read_POSR()
            pos_idx = POSR.uint
            PNOW = self.controller.read_PNOW()
            pos = PNOW.int*0.01
        except Exception as e:
            error = "while getting final position: %s" % e
        res = GenericResult(result=("%d %.2f" % (pos_idx, pos)),
                             error=error)

        if status["ERR"] or error:
            log("returning with error")
        if status["ERR"]:
            res.error = "error after moving: %s" % status["ALC0"].upper()
            self.server.set_aborted(res)
        else:
            log("returning with success")
            self.server.set_succeeded(res)


    def servo_off(self, log):
        log("switching off servo")
        self.controller.write_SON(False)
        # wait for servo to go off
        for i in range(5):
            stat = self.controller.status()
            if stat["SV"]:  # still on
                time.sleep(0.1)
                continue
            break
        else:
            # if the servo did not go off, it is an error
            res = GenericResult(result="0 0.0",
                                error="failed to switch off servo")
            log("returning with error: %s" % res)
            self.server.set_aborted(res)
            return

        # if we arrive here, the servo is off
        res = GenericResult(result="-1 0.0")
        log("returning with success: %s" % res.result)
        self.server.set_succeeded(res)


    def servo_on(self, log):
        log("switching on servo")
        self.controller.write_SON(True)
        # wait for servo to go on
        for i in range(20):
            stat = self.controller.status()
            if not stat["SV"]:  # still off
                time.sleep(0.1)
                continue
            break
        else:
            # if the servo did not go on, it is an error
            res = GenericResult(result="0 0.0",
                                error="failed to switch on servo")
            log("returning with error: %s" % res)
            self.server.set_aborted(res)
            return

        # if we arrive here, the servo is on, now go HOME
        self.controller.write_HOME(True)
        self.controller.write_HOME(False)
        # wait for completion
        for i in range(30):
            stat = self.controller.status()
            if not stat["HEND"]:  # still not at home
                time.sleep(0.1)
                continue
            break
        else:
            res = GenericResult(result="0 0.0",
                                error="failed to return to HOME position")
            log("returning with error: %s" % res)
            self.server.set_aborted(res)
            return

        # get current position
        pos = 0.0
        try:
            PNOW = self.controller.read_PNOW()
            pos = PNOW.int*0.01
        except Exception as e:
            res = GenericResult(result="0 0.0",
                                error="failed to get final position")
            self.server.set_aborted(res)
            return
        res = GenericResult(result=("-2 %.2f" % pos))
        log("returning with success: %s" % res.result)
        self.server.set_succeeded(res)

    def shutdown(self):
        rospy.loginfo("shutting down %s" % self.controller)
        self.controller.shutdown()


class GripperAngleServer:
    def __init__(self, controller):
        self.controller = controller
        self.server = actionlib.SimpleActionServer('gripper_angle',
                                                   GenericAction,
                                                   self.execute,
                                                   False)
        self.server.start()
        # get current angle from file
        log = lambda s: rospy.loginfo("<GRIP> " + s)
        try:
            self.last_angle = float(open("gripper_angle").read())
            log("read initial angle %s from file" % self.last_angle)
        except Exception as e:
            log("exception %s while reading angle from file, start with 0.0"
                % e)
            self.last_angle = 0.0

        # we need to find out in which direction we went home
        if (self.last_angle % 360) < 330:
            # this will always go *back*
            self.last_angle = (int(self.last_angle) / 360) * 360
            log("assumed back move")
        else:
            # this only goes back *sometimes*, so check if we
            # crossed 180 degrees while going home
            went_back = False
            MOVE = True
            while MOVE:
                status = self.controller.status()
                MOVE = status["MOVE"]
                PNOW = self.controller.read_PNOW()
                pos = PNOW.int * 0.01
                log("servo is still going HOME @ " + str(pos))
                if pos < 270 and pos > 90:
                    if not went_back:
                        log("detected back move")
                    went_back = True
                time.sleep(0.2)
            if went_back:
                self.last_angle = (int(self.last_angle) / 360) * 360
            else:
                log("apparently HOME went forward")
                self.last_angle = (int(self.last_angle) / 360 + 1) * 360
        log("adjusted initial angle to %s after HOME cmd" % self.last_angle)

    def execute(self, goal):
        log = lambda s: rospy.loginfo("<GRIP> " + s)
        log2 = lambda s: rospy.logdebug("<PIPE> " + s)
        log("got %s" % goal)

        # unpack
        try:
            IN_target_angle = float(goal.target)
        except Exception as e:
            res = GenericResult(error="target must be a float value")
            self.server.set_aborted(res)
            return

        # check parameter validity
        if int(IN_target_angle) == -1:
            return self.servo_off(log)
        elif int(IN_target_angle) == -2:
            return self.servo_on(log)
        elif IN_target_angle < 0:
            res = GenericResult(error="target_angle must be positive, -1 or -2")
            self.server.set_aborted(res)
            return

        # compute all the goals that we need to visit
        goals = []
        if IN_target_angle > self.last_angle:
            next = (int(self.last_angle) / 120 + 1) * 120
            while next < IN_target_angle:
                goals.append(next)
                next += 120
            mode_increasing = True
        else:
            next = (int(self.last_angle) / 120) * 120
            while next > IN_target_angle:
                goals.append(next)
                next -= 120
            mode_increasing = False
        final_pos = IN_target_angle % 360

        pos_idxs = [(deg / 120) % 3 for deg in goals]
        pretty_goals = ["(%s)" % self.last_angle] + goals + [IN_target_angle]
        log("will move via %s, i.e., %s" % (goals, pos_idxs))

        num_fullrounds = int(self.last_angle) / 360
        PNOW_prev = (self.last_angle % 360) * 100
        log("start with PNOW_prev = %s" % (PNOW_prev*0.01))

        # move the required big steps
        for i, pos_idx in enumerate(pos_idxs):
            # move actuator
            try:
                log2("moving actuator to %s (= %s deg)" % (pos_idx, goals[i]))
                status = self.controller.move_to_preset(pos_idx)
                #log("status: " + str(status))
            except Exception as e:
                res = GenericResult(error="exception while moving: %s" % e)
                self.server.set_aborted(res)
                return

            error = ""

            # wait while one of the actuators is moving
            MOVE = status["MOVE"]
            if not MOVE:
                # we are maybe not moving here, because we have
                # been too close to the goal already. if that is
                # the case, change num_fullrounds
                if mode_increasing and PNOW_prev > 27000 and pos_idx == 0:
                    log("increasing fullrounds we are close to 0")
                    num_fullrounds += 1
                elif not mode_increasing and PNOW_prev < 9000 and pos_idx == 0:
                    log("decreasing fullrounds we are close to 0")
                    num_fullrounds -= 1

            while MOVE:
                if self.server.is_preempt_requested():
                    self.server.set_preempted()
                    log("goal preempted")
                    return

                # check whether actuator is still moving
                try:
                    status = self.controller.status()
                except Exception as e:
                    error = "while waiting for move to finish: %s" % e
                    # at this point, we cannot determine whether
                    # the servo is still moving or not. internally
                    # there have already been a number of retries,
                    # so we can assume communication is somehow
                    # broken and return with an error
                    break
                MOVE = status["MOVE"]

                # emit feedback
                try:
                    PNOW = self.controller.read_PNOW()

                    # adjust the round count
                    if mode_increasing and PNOW.int < PNOW_prev - 15000:
                        log("increasing fullrounds because now=%s, prev=%s" %
                            (PNOW.int*0.01, PNOW_prev*0.01))
                        num_fullrounds += 1
                    elif not mode_increasing and PNOW.int > PNOW_prev + 15000:
                        log("decreasing fullrounds because now=%s, prev=%s" %
                            (PNOW.int*0.01, PNOW_prev*0.01))
                        num_fullrounds -= 1
                    PNOW_prev = PNOW.int

                    FB_moving = MOVE
                    FB_pos = num_fullrounds * 360 + PNOW.int*0.01
                    # set last position in case we fail to complete
                    self.last_angle = FB_pos
                    fb = GenericFeedback(status=("%d %.2f" % (FB_moving, FB_pos)))
                    self.server.publish_feedback(fb)

                    # if we get close to the next point, stop
                    # querying and send the next command
                    can_skip_inc = i == 0 or (i > 0 and FB_pos > goals[i-1])
                    can_skip_dec = i == 0 or (i > 0 and FB_pos < goals[i-1])
                    #if mode_increasing and can_skip_inc and FB_pos % 120 > 10:
                    #    log("get close to target point, send next command")
                    #    break
                    #elif not mode_increasing and can_skip_dec and (FB_pos % 120 < 10):
                    #    log("get close to target point, send next command")
                    #    break

                except Exception as e:
                    log("got exception \"%s\", skip feedback" % e)

                # sleep a bit
                time.sleep(0.1)

        # now move to the final position
        try:
            log("moving actuator to %s (= %s deg)" %
                (final_pos, IN_target_angle))
            status = self.controller.move_to_position(final_pos)
            log("status: " + str(status))
        except Exception as e:
            res = GenericResult(error="exception while moving: %s" % e)
            self.server.set_aborted(res)
            return

        error = ""

        # wait while one of the actuators is moving
        MOVE = status["MOVE"]
        while MOVE:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                log("goal preempted")
                return

            # check whether actuator is still moving
            try:
                status = self.controller.status()
            except Exception as e:
                error = "while waiting for move to finish: %s" % e
                # at this point, we cannot determine whether
                # the servo is still moving or not. internally
                # there have already been a number of retries,
                # so we can assume communication is somehow
                # broken and return with an error
                break
            MOVE = status["MOVE"]

            # emit feedback
            try:
                PNOW = self.controller.read_PNOW()
                FB_moving = MOVE
                FB_pos = num_fullrounds * 360 + PNOW.int*0.01
                # set last position in case we fail to complete
                self.last_angle = FB_pos
                fb = GenericFeedback(status=("%d %.2f" % (FB_moving, FB_pos)))
                self.server.publish_feedback(fb)
            except Exception as e:
                log("got exception \"%s\", skip feedback" % e)

            # sleep a bit
            time.sleep(0.1)

        self.last_angle = IN_target_angle

        # get current position
        pos_idx = 0
        pos = 0.0
        try:
            POSR = self.controller.read_POSR()
            pos_idx = POSR.uint
            PNOW = self.controller.read_PNOW()
            # adjusting num_fullrounds is very hard around
            # the zero area, so we just pretend to be right
            #pos = num_fullrounds * 360 + PNOW.int*0.01
            pos = IN_target_angle
        except Exception as e:
            error = "while getting final position: %s" % e
        res = GenericResult(result=("%d %.2f" % (pos_idx, pos)),
                            error=error)

        if status["ERR"] or error:
            log("returning with error")
        if status["ERR"]:
            res.error = "error after moving: %s" % status["ALC0"].upper()
            self.server.set_aborted(res)
        else:
            log("returning with success")
            self.server.set_succeeded(res)


    def servo_off(self, log):
        log("switching off servo")
        self.controller.write_SON(False)
        # wait for servo to go off
        for i in range(5):
            stat = self.controller.status()
            if stat["SV"]:  # still on
                time.sleep(0.1)
                continue
            break
        else:
            # if the servo did not go off, it is an error
            res = GenericResult(result="0 0.0",
                                error="failed to switch off servo")
            log("returning with error: %s" % res)
            self.server.set_aborted(res)
            return

        # if we arrive here, the servo is off
        res = GenericResult(result="-1 0.0")
        log("returning with success: %s" % res.result)
        self.server.set_succeeded(res)


    def servo_on(self, log):
        log("switching on servo")
        self.controller.write_SON(True)
        # wait for servo to go on
        for i in range(20):
            stat = self.controller.status()
            if not stat["SV"]:  # still off
                time.sleep(0.1)
                continue
            break
        else:
            # if the servo did not go on, it is an error
            res = GenericResult(result="0 0.0",
                                error="failed to switch on servo")
            log("returning with error: %s" % res)
            self.server.set_aborted(res)
            return

        # if we arrive here, the servo is on, now go HOME
        self.controller.write_HOME(True)
        self.controller.write_HOME(False)
        # wait for completion
        for i in range(30):
            stat = self.controller.status()
            if not stat["HEND"]:  # still not at home
                time.sleep(0.1)
                continue
            break
        else:
            res = GenericResult(result="0 0.0",
                                error="failed to return to HOME position")
            log("returning with error: %s" % res)
            self.server.set_aborted(res)
            return

        # get current position
        pos = 0.0
        try:
            PNOW = self.controller.read_PNOW()
            pos = PNOW.int*0.01
        except Exception as e:
            res = GenericResult(result="0 0.0",
                                error="failed to get final position")
            self.server.set_aborted(res)
            return
        res = GenericResult(result=("-2 %.2f" % pos))
        log("returning with success: %s" % res.result)
        self.server.set_succeeded(res)

    def shutdown(self):
        open("gripper_angle", "w").write(str(self.last_angle))
        rospy.loginfo("shutting down %s" % self.controller)
        self.controller.shutdown()


if __name__ == '__main__':
    rospy.init_node('servo_server')

    # connect to controllers
    rospy.loginfo("creating Modbus connection")
    mbclient = ModbusClient(method='ascii', port='/dev/ttyUSB0',
                            timeout=0.01, baudrate=230400)
    mbclient.connect()

    lin_ctrl = controller.LinearServoController(mbclient, 0x04)
    rot_ctrl3 = controller.RotaryServoController(mbclient, 0x01,
        lower_lim=0.0, upper_lim=360.0)
    all_ctrls = [lin_ctrl, rot_ctrl3]
    try:
        for i, c in enumerate(all_ctrls):
            rospy.loginfo("initializing %s" % c)
            stat = c.initialize()
            if stat["ERR"]:
                rospy.logwarn("alarm @ %s: %s" % (c, stat["ALC0"].upper()))
    except Exception as e:
        # shut down all initialized controllers
        for c in all_ctrls[:i]:
            c.shutdown()
        mbclient.close()
        raise e

    # initialize node and create action servers
    pipe_server = TurnPipeServer(lin_ctrl)
    grip_server = GripperAngleServer(rot_ctrl3)
    all_servers = [pipe_server, grip_server]

    # go
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerror(str(e))
        raise e
    finally:
        for s in all_servers:
            rospy.loginfo("shutting down %s" % s)
            s.shutdown()
        rospy.loginfo("closing Modbus connection")
        mbclient.close()
