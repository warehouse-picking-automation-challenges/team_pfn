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

import rospy

import interface
import get_item

import shelf_stow_locator
import tote_stow_locator
import arm_control_wrapper
import grab
import util

import time

(logdebug, loginfo, logwarn, logerr) = util.get_loggers("MoveUntilGrabbablePicking")

class DifficultyOrderPicking(interface.PickProcessStrategy):

    def execute(self, items):
        # get the strategy for how to prepare the actual pick
        get_item_strategy = get_item.select_strategy(self.pos_info)
        loginfo("GetItemFromBinStrategy: %s" % get_item_strategy)

        # get the strategy for where to store items
        target_locator = tote_stow_locator.select_strategy(self.pos_info)
        loginfo("ToteStowLocatorStrategy: %s" % target_locator)
        
        calib_ok = super(DifficultyOrderPicking, self).calibrate()
        if not calib_ok:
            logerr("calibration failed")
            return False
        arm_control_wrapper.move_left_arm_to_bin('bin_A','pre', keitai='nut000', kakujiku='kakujiku', speed="fast")
        
        checkpoint = time.time()
        
        #calc score and sort 
        scores = self.pos_info.score_to_pick()
        scores_with_try_num = []
        for bin, num_items_score in scores.items():
            for item, bin_of_item in items:
                if bin_of_item == bin:
                    item_name = item
                    break
            if item_name in ["rolodex_jumbo_pencil_cup", "fitness_gear_3lb_dumbbell"]:
                # try them after we have tried everything else twice
                try_num = 10
            elif item_name in ["dasani_water_bottle"]:
                # try them after we have tried everything else twice
                try_num = 30
            else:
                try_num = 1
            # we re-evaluate the score for the order here. we have:
            # - num_items_score: 9 (one item in bin) ~ 0 (ten items in bin)
            # - item_bonus: 0 (simple item) ~ 3 (hard item)
            # - item_difficulty: 0 (simple item) ~ 3 (hard item)
            # - pick_points: 10 (1/2 items), 15 (3/4 items), 20 (5+ items)
            item_bonus = util.item_string_to_bonus(item_name)
            item_difficulty = util.item_string_to_difficulty(item_name)
            pick_points = self.pos_info.bin_pick_points(bin)
            # compute some reasonable the-lower-the-better score
            # - try easy items first.
            # - if equally easy, take the emptier bin first.
            score = 0*item_difficulty - num_items_score
            scores_with_try_num.append((bin, score, item_name, try_num))

        # we sort by try_num (asc), then by score (desc). that is, first
        # we try all the items, then we try earlier failures again
        sorted_scores = sorted(scores_with_try_num,
                               key=lambda x: (x[3], x[1]))

        i = 0
        while sorted_scores:
            # sort list in case earlier iterations have appended items
            sorted_scores.sort(key=lambda x: (x[3], x[1]))
            bin, score, item_name, try_num = sorted_scores.pop(0)

            if i != 0:
                loginfo("completed %d-th item in %s (total: %s)" %
                        (i,
                         util.get_elapsed_time(checkpoint),
                         util.get_elapsed_time(self.start)))
                checkpoint = time.time()
            i += 1
            if i > 100:
                logwarn("exiting after 1000 main iterations")
                break

            loginfo("======================================")
            loginfo("attempt to pick item %s from %s (attempt %d, iteration %d)" %
                    (item_name, bin, try_num, i))
            items_in_bin = [str(s) for s in self.pos_info.shelf[bin]]
            loginfo("items in that bin: %s" % items_in_bin)

            # compute the target_position for the item we want to move
            # (this may use any of the arms and move it to the tote!)
            try:
                target_position = target_locator.find_location(item_name)
            except Exception as e:
                logerr("failed to find a stow location for %s: %s" %
                       (item_name, e))
                # we cannot continue with that item, proceed with the next one
                continue
            loginfo("target position for %s in the tote: %s" %
                    (item_name, target_position))

            # take the item
            try:
                (pick_success, grab_strategy, moved_items) = \
                    get_item_strategy.get_item(item_name, bin, items_in_bin, try_num)
            except util.TooCrowdedTryLater:
                logwarn("couldn't find item %s in a crowded bin, try again later" % item_name)
                sorted_scores.append((bin, score, item_name, try_num + len(items_in_bin)))
                continue
            except Exception as e:
                logerr("an exception occured during get_item(%s, %s, %s, %d): %s" %
                       (item_name, bin, items_in_bin, try_num, e))
                continue

            # NB. we used to update the moved_items here, but this
            #     was moved into the get_item_strategy

            # check if our pick was successful
            if not pick_success:
                logwarn("we failed to pick the item %s" % item_name)
                try:
                    # clear cache
                    logwarn("clear cache!")
                    if grab_strategy.final_mode[bin]:
                        grab_strategy.strategy_cache[bin] = []
                        grab_strategy.final_mode[bin] = False
                    else:
                        grab_strategy.final_mode[bin] = True
                except Exception as e:
                    logwarn("clear cache tried but not vacuumgrabbing %s"%e)
                sorted_scores.append((bin, score, item_name, try_num + 5 + len(items_in_bin)))
                continue

            # put the item to the tote
            try:
                grab_strategy.stow_in_tote(target_position, item=item_name)
            except Exception as e:
                logerr("failed to put item to box: %s" % e)

            try:
                # update the locations
                self.pos_info.take_from_bin(bin, item_name)
                self.pos_info.put_into_tote(item_name)
            except Exception as e:
                logerr("failed to update item locations: %s" % e)

            # write intermediate output file
            util.write_output_file(self.pos_info, "current_pick_status.json")
