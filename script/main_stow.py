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

import argparse
import json

import rospy

import util
from item_locations import ItemLocations


def main(task_file, out_file):
    # import here so that we can check util.is_gripper_enabled() in import
    import stow_process

    task = json.load(open(task_file))
    rospy.loginfo("starting stow task from %s" % task_file)
    # create object to maintain item locations
    pos_info = ItemLocations(task["bin_contents"], task["tote_contents"])
    # pick the best overall strategy
    main_strategy = stow_process.select_strategy(pos_info)
    # confirm the area is safe
    if not raw_input("This will send commands to the robot. " +
                     "Is the area safe? [yN] ").lower() == "y":
        rospy.loginfo("Aborted stow process because not safe")
        print "abort"
        return
    # start run
    try:
        main_strategy.execute()
    except KeyboardInterrupt:
        rospy.logwarn("interrupted by Ctrl-C")
    except Exception:
        import sys
        exc_info = sys.exc_info()
        util.write_output_file(pos_info, out_file)
        rospy.logerr("got exception during main run")
        raise exc_info[1], None, exc_info[2]
    # write output file
    util.write_output_file(pos_info, out_file)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file",
                        help="input file for the stow task")
    parser.add_argument("--out", default="-",
                        help="output file for the result")
    parser.add_argument('--no-gripper', dest='use_gripper',
                        action='store_false',
                        help="disable the gripper for this run")
    parser.set_defaults(use_gripper=True)
    args = parser.parse_args()
    util.set_gripper_enabled(args.use_gripper)
    rospy.init_node("main_node", anonymous=True)
    main(args.file, args.out)
