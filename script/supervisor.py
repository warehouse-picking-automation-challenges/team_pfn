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

import os
import time

import rospy
from std_msgs.msg import String, Float32


class NodeSupervisor(object):
    def __init__(self):
        self.last_sender = ("", 0)
        rospy.Subscriber("/txt_me_and_die", String, self.callback)
        rospy.Subscriber("/pressure", Float32, self.update_life_signal)

    def callback(self, msg):
        if msg.data == "keep_alive":
            return

        # check which node sent the message and kill it
        sender = msg._connection_header['callerid'].lstrip("/")
        rospy.logwarn("got message from node '%s', kill it" % sender)

        # assume that node was started with roslaunch,
        # so it has its node name as a parameter
        res = os.system('pkill -f "__name:=%s"' % sender)
        rospy.loginfo("pkill returned with %s" % res)

    def update_life_signal(self, msg):
        # update internal state
        sender = msg._connection_header['callerid'].lstrip("/")
        if not self.last_sender[0]:
            rospy.loginfo("got first sign of life from %s" % sender)
        now = time.time() + 1  # give some grace time to start
        self.last_sender = (sender, now)

    def check_life_signal(self):
        sender, last_update = self.last_sender
        if not sender:
            return
        timeout = 3  # number of seconds we wait for a timeout
        if time.time() - last_update > timeout:
            rospy.loginfo("trying to kill %s which has not sent data for %s seconds" %
                          (sender, timeout))
            self.last_sender = ("", 0)

            # assume that node was started with roslaunch,
            # so it has its node name as a parameter
            res = os.system('pkill -f "__name:=%s"' % sender)
            rospy.loginfo("pkill returned with %s" % res)


if __name__ == '__main__':
    rospy.init_node("supervisor", anonymous=True)
    pub = rospy.Publisher('/txt_me_and_die', String, queue_size=10)
    pub.publish("keep_alive")

    n = NodeSupervisor()

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
            pub.publish("keep_alive")
            n.check_life_signal()
            rate.sleep()
