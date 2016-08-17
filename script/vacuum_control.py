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

## Node Name: vacuum_control
## Action Name: vacuum_switch

## Action Type: Vacuum
## input: string ("normal", "vacuum", "off", "pump" or "double")
## v: vacuum cleaner valve a: air valve p: pump valve s: vacuum cleaner switch
## C: closed O: open
##  state | v | a | p |  s  | comment
## normal | C | C | C | OFF | normal mode // maybe change to C/O/C/C
## vacuum | O | C | C | ON  | when it starts picking motion
## off    | C | O | C | OFF | when it starts putting objects
## pump   | C | C | O | OFF | when you want to use vacuum pump
## double | O | C | O | ON  | when you want to use both vacuum cleaner and vacuum pump
##
## output: bool

## Publish topic: "relay_command" to control valves
## Use service "vac_on" and "vac_off" to control vacuum cleaner switch
## provide action to control valves and switch

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_srvs.srv import Empty
import actionlib
from apc2016.msg import VacuumAction
from apc2016.msg import VacuumResult
from apc2016.msg import VacuumFeedback

class Vacuum_Action(object):
    def __init__(self):
        rospy.loginfo('vacuum control start')
        self.pub = rospy.Publisher('relay_command', Int16, queue_size = 10)
        rospy.loginfo('reraly commad')
        self.acs = actionlib.SimpleActionServer('vacuum_switch', VacuumAction, execute_cb=self.callback, auto_start=False)
        self.acs.start()
        rospy.loginfo('action lib')

    def call_service(self, status):
        srv = 'vac_off'
        if status == 'on':
            srv = 'vac_on'
        try:
            service = rospy.ServiceProxy(srv, Empty)
            response = service()
        except rospy.ServiceException, e:
            rospy.loginfo('Service failed %s', e)

    def callback(self, goal):
        rospy.loginfo('callback called')
        res = VacuumResult(success = True)
        if goal.status == 'normal':
            self.pub.publish(10)
            self.pub.publish(20)
            self.pub.publish(30)
            self.pub.publish(40)
        elif goal.status == 'vacuum':
            self.pub.publish(11)
            self.pub.publish(20)
            self.pub.publish(30)
            self.pub.publish(41)
        elif goal.status == 'off':
            self.pub.publish(10)
            self.pub.publish(21)
            self.pub.publish(30)
            self.pub.publish(40)
        elif goal.status == 'pump':
            self.pub.publish(10)
            self.pub.publish(20)
            self.pub.publish(31)
            self.pub.publish(40)
        elif goal.status == 'double':
            self.pub.publish(11)
            self.pub.publish(20)
            self.pub.publish(31)
            self.pub.publish(41)
        else:
            rospy.loginfo('Invalid Status')
            res = VacuumResult(success = False)
        self.acs.set_succeeded(res)

if __name__ == "__main__":
    rospy.init_node('vacuum_control')
    vacuum = Vacuum_Action()
    rospy.spin()
