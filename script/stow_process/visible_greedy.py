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

import random
import time

import interface
import util
import shelf_stow_locator
import grab
import arm_control_wrapper


(logdebug, loginfo, logwarn, logerr) = util.get_loggers("VisibleGreedyStowing")

class ItemChoiceMode:
    RANDOM, SMALLEST, LARGEST, VISIBLE = range(4)

CHOICE_MODE = ItemChoiceMode.VISIBLE


class VisibleGreedyStowing(interface.StowProcessStrategy):
    """VisibleGreedyStowing scans the tote and takes the item with the
    highest score to the shelf."""
    def execute(self):
        # get a stratey to determine free space
        stow_strategy = shelf_stow_locator.select_tote2bin_strategy(self.pos_info)
        loginfo("ShelfStowLocatorStrategy: %s" % stow_strategy)

        calib_ok = super(VisibleGreedyStowing, self).calibrate()
        if not calib_ok:
            logerr("calibration failed")
            return False

        secondary_item_priorities = []

        checkpoint = time.time()

        try_count = 0
        success_count = 0
        items_in_tote = self.pos_info.tote[:]

        item_list = []

        for item in items_in_tote:
            if item == "fitness_gear_3lb_dumbbell":
                # treat the dumbbell as if it was not in the box
                continue
            elif item == "scotch_duct_tape":
                # take the tape later
                item_list.append((3, item))
            #elif item in ["fiskars_scissors_red","oral_b_toothbrush_red","oral_b_toothbrush_green"]:
            #    item_list.append((2, item))
            else:
                item_list.append((1, item))

        # make item_list a list with the count of tries
        item_list = zip([1]*len(items_in_tote), items_in_tote[:])
        while item_list and try_count < 100:
            if try_count != 0:
                loginfo("completed %d-th item in %s (total: %s)" %
                        (try_count,
                         util.get_elapsed_time(checkpoint),
                         util.get_elapsed_time(self.start)))
                checkpoint = time.time()

            try_count += 1

            loginfo("remaining items (unordered): %s" % item_list)

            if CHOICE_MODE == ItemChoiceMode.RANDOM:
                (num_tries, next_item) = random.choice(item_list)
            elif CHOICE_MODE == ItemChoiceMode.SMALLEST:
                (num_tries, next_item) = \
                    util.get_try_smallest_item_with_margin(item_list)
            elif CHOICE_MODE == ItemChoiceMode.LARGEST:
                raise NotImplementedError("largest item order is not implemented")
            elif CHOICE_MODE == ItemChoiceMode.VISIBLE:
                # if we have a list of priorities from the previous run,
                # take the one with the lowest index in that list that
                # we haven't tried yet before
                if secondary_item_priorities:
                    (num_tries, next_item) = \
                        min(item_list,
                            key=lambda x: (x[0], secondary_item_priorities.index(util.item_string_to_id(x[1]))))
                else:
                    # otherwise, take the smallest one
                    (num_tries, next_item) = \
                        util.get_try_smallest_item_with_margin(item_list)


            loginfo("======================================")
            loginfo("attempt to take item %s from tote" % next_item)

            # first, find a suitable bin
            loginfo("find a location to move that item to")
            (target_bin, target_pos) = \
                stow_strategy.find_location(next_item)
            loginfo("if we succeed to take %s, we will put it to %s/%s" %
                    (next_item, target_bin, target_pos))

            # move the arm back to tote
            try:
                (mv_success, _, _, _) = arm_control_wrapper.move_left_arm_to_bin("tote",
                                                                                 "", keitai='nut000', kakujiku='kakujiku')
            except Exception as e:
                logerr("error while moving arm to tote: %s" % e)

            # try all possible grab strategies
            tried_strategies = 0
            for grab_strategy in grab.select_strategies(next_item,
                                                        "tote"):
                # check if this strategy can locate the item
                try:
                    can_grab = grab_strategy.can_grab_item(next_item, "tote",
                                                           items_in_tote)
                except Exception as e:
                    logerr("error while checking for grabability: %s" % e)
                    continue
                if not can_grab:
                    loginfo("strategy %s cannot grab item %s" %
                            (grab_strategy, next_item))
                    # if we can, we should find out which items are grabbable
                    # from here and then use this as priority list
                    if CHOICE_MODE == ItemChoiceMode.VISIBLE and \
                            isinstance(grab_strategy, grab.vacuum.VacuumGrabbing):
                        try:
                            secondary_item_priorities = grab_strategy.sort_item_priority_tote().tolist()
                        except Exception as e:
                            logerr("error during sort_item_priority_tote(): %s" % e)
                            raise e
                            secondary_item_priorities = []
                    continue
                else:
                    secondary_item_priorities = []
                loginfo("can_grab_item was True, try strategy %s for item %s" %
                        (grab_strategy, next_item))

                tried_strategies += 1

                # try to grab the item with this strategy
                try:
                    success = grab_strategy.grab_from_tote(next_item)
                except Exception as e:
                    logerr("error while taking item %s: %s" % (next_item, e))
                    # NB. this is not good, we have no idea what happened here;
                    # the arm is in an undefined state
                    # TODO: need to reset the state somehow
                    continue
                if not success:
                    loginfo("failed to take %s with %s" % (next_item, grab_strategy))
                    continue
                else:
                    loginfo("succeeded to take item %s" % next_item)

                try:
                    success = grab_strategy.stow_in_shelf(target_bin, target_pos)
                except Exception as e:
                    logerr("error while stowing item %s: %s" % (next_item, e))
                    # uh, this is not good, we removed the item from the shelf,
                    # but we failed to put it somewhere.
                    success = False
                if not success:
                    # same as exception case before
                    arm_control_wrapper.move_left_arm_to_bin("bin_H", "rpre", keitai='nut000', kakujiku='kakujiku', speed="fast")
                    logwarn("we failed to stow item %s in shelf" % next_item)
                    #self.pos_info.take_from_tote(next_item)
                    #self.pos_info.drop(next_item)
                    continue
                else:
                    # if we arrive here, we put the item to the shelf successfully
                    self.pos_info.take_from_tote(next_item)
                    self.pos_info.put_into_bin(target_bin, next_item)
                
                try:
                    (mv_success, _, _, _) = arm_control_wrapper.move_left_arm_to_bin("bin_H",
                                                                                     "rpre", keitai='nut000', kakujiku='kakujiku')
                except Exception as e:
                    logerr("error while moving arm to tote: %s" % e)
                    #raise e

                # write intermediate output file
                util.write_output_file(self.pos_info, "current_stow_status.json")

                items_in_tote.remove(next_item)
                item_list.remove((num_tries, next_item))
                break
            else:
                if tried_strategies > 0:
                    logwarn("we failed to take %s with any strategy" % next_item)
                    item_list.remove((num_tries, next_item))
                    item_list.append((num_tries + 1, next_item))
                else:
                    logwarn("we failed to detect %s with any strategy" % next_item)
                    item_list.remove((num_tries, next_item))
                    item_list.append((num_tries + 1, next_item))
