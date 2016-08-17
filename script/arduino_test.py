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

import rospy
from apc2016.srv import *
import time
import actionlib
from apc2016.msg import *
from std_msgs.msg import Float32

class ard_test:
    def __init__(self):
        self.sub = rospy.Subscriber('pressure', Float32, self.callback)
        self.ok = [0,0,0,0]
        self.good_pressure = 0
    def callback(self, message):
        if message.data < 700 or message.data > 1100:
            rospy.loginfo("pressure sensor error. please reconnect arduino")
        elif self.good_pressure == 0:
            rospy.loginfo("pressure sensor arduino ok: data %s" %message.data)
            self.ok[0] = 1
            self.good_pressure = 1
    def right_distance(self):
        try:
            rospy.wait_for_service('distance_sensor_right', 5)
        except:
            rospy.loginfo('right distance sensor error. please reconnect arduino')
        s = rospy.ServiceProxy('distance_sensor_right', distance)
        ok = 0
        for i in range(10):
            response = s()
            if response.dist > 1 and ok == 0:
                rospy.loginfo('right distance arduino ok!')
                ok = 1
                self.ok[1] = 1
        if ok == 0:
            rospy.loginfo('right distance sensor error. please reconnect')

    def left_distance(self):
        try:
            rospy.wait_for_service('distance_sensor_left', 5)
        except:
            rospy.loginfo('left distance sensor error. please reconnect arduino')
        s = rospy.ServiceProxy('distance_sensor_left', distance)
        ok = 0
        for i in range(10):
            response = s()
            if response.dist > 1 and ok == 0:
                rospy.loginfo('left distance arduino ok!')
                ok = 1
                self.ok[2] = 1
        if ok == 0:
            rospy.loginfo('left distance sensor error. please reconnect')
 
    def vac_on(self):
        c = actionlib.SimpleActionClient('vacuum_switch',VacuumAction)
        c.wait_for_server()
        vac_goal = VacuumGoal()
        vac_goal.status = 'vacuum'
        c.send_goal(vac_goal)
        c.wait_for_result(rospy.Duration.from_sec(3.0))

    def vac_normal(self):
        c = actionlib.SimpleActionClient('vacuum_switch',VacuumAction)
        c.wait_for_server()
        vac_goal = VacuumGoal()
        vac_goal.status = 'normal'
        c.send_goal(vac_goal)
        c.wait_for_result(rospy.Duration.from_sec(3.0))

    def vac_off(self):
        c = actionlib.SimpleActionClient('vacuum_switch',VacuumAction)
        c.wait_for_server()
        vac_goal = VacuumGoal()
        vac_goal.status = 'off'
        c.send_goal(vac_goal)
        c.wait_for_result(rospy.Duration.from_sec(3.0))

    def checkvac(self):
        rospy.loginfo('vac on')
        self.vac_on()
        time.sleep(10)
        rospy.loginfo('vac off')
        self.vac_off()
        time.sleep(5)
        rospy.loginfo('vac normal')
        self.vac_normal()
        rospy.loginfo('vacuum control arduino ok.')
        self.ok[3] = 1
if __name__ == '__main__':
    rospy.init_node('arduino_test')
    a = ard_test()
    rospy.loginfo("Arduino test start")
    rospy.loginfo("check vacuum on/ off")
    a.checkvac()
    a.right_distance()
    a.left_distance()
    if sum(a.ok) == 4:
        rospy.loginfo("All arduino OK!")
    else:
        if a.ok[0] == 0:
            rospy.loginfo("PRESSURE SENSOR ERROR")
        if a.ok[1] == 0:
            rospy.loginfo("RIGHT DISTANCE SENSOR ERROR")
        if a.ok[2] == 0:
            rospy.loginfo("LEFT DISTANCE SENSOR ERROR")
        if a.ok[3] == 0:
            rospy.loginfo("RELAY ERROR")
        rospy.loginfo("Please check messages. Reconnect arduino and kill and restart arduino node")
