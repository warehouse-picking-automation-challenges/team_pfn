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

import shelf_stow_locator
import arm_control_wrapper
import grab
import util

import interface

(logdebug, loginfo, logwarn, logerr) = util.get_loggers("MoveUntilGrabbablePicking")


class GrabResult:
    SUCCESS, CANNOT_GRAB, FAILED_GRAB = range(3)


class MoveUntilGrabbablePicking(interface.GetItemFromBinStrategy):
    # MoveUntilGrabbablePicking moves items that are obstacles to a different
    # bin until the difficulty to grab the target item is low enough.
    def __init__(self, pos_info):
        self.pos_info = pos_info

    def get_max_tries(self, item, strategy):
        # number of attempts to take an item using grab_from_shelf
        if isinstance(strategy, grab.vacuum.VacuumGrabbing):
            # max limits for vauum
            if item in ["fitness_gear_3lb_dumbbell",
                        "rolodex_jumbo_pencil_cup",
                        "dasani_water_bottle",
                        "kleenex_paper_towels"]:
                # these are items we would rather give up on
                # than try again
                return 1
            else:
                return 2
        else:
            # max limits for gripper
            if item == "fitness_gear_3lb_dumbbell":
                return 1
            else:
                return 1

    def get_item(self, main_item, source_bin, items_in_bin, try_count=1):
        FAIL_MODE = (False, None, [])

        # check input data plausability
        if not items_in_bin:
            logerr("according to the inventory, there are no items " +
                   "in the bin, but we tried to grab %s" % main_item)
            return FAIL_MODE
        elif main_item not in items_in_bin:
            logerr(("according to the inventory, the items in %s are " +
                    "%s, but we tried to grab %s") % (source_bin,
                                                            items_in_bin,
                                                            main_item))
            return FAIL_MODE

        # move the left arm to the bin at hand (actually this is not
        # exactly necessary, as every grab strategy will move the
        # arm as required)
        try:
            (success, _, _, _) = arm_control_wrapper.move_left_arm_to_bin(source_bin,
                                                                          "pre",
                                                                          keitai='nut000',
                                                                          kakujiku='kakujiku', speed="fast")
        except Exception as e:
            logerr("error while moving arm to bin: %s" % e)
            return FAIL_MODE
        if not success:
            return FAIL_MODE

        # first, we try to take the item at hand (that is,
        # detect whether we can take it)
        (result, strategy, other_item_priorities) = \
            self.__try_take_item(main_item, source_bin, items_in_bin)
        main_item_strategy = strategy
        if result == GrabResult.SUCCESS:
            # great, we got it
            return (True, strategy, [])
        elif result == GrabResult.FAILED_GRAB:
            # all strategies that declared that they can
            # grab the item failed to do so
            return (False, strategy, []) #FAIL_MODE
        elif result == GrabResult.CANNOT_GRAB:
            if len(items_in_bin) > try_count*3:
                # we cannot see this item currently and the box is
                # quite crowded (at least four items at the first
                # try, at least seven items at the second), therefore
                # we skip this for now
                raise util.TooCrowdedTryLater()

        # if we arrive here, none of the strategies claimed the
        # item to be grabbable, so we move away some other object
        logwarn(("none of the known strategies could take %s, " +
                 "we will move away other items") % main_item)

        # get a strategy to discover place for intermediate items
        stow_strategy = shelf_stow_locator.select_bin2bin_strategy(self.pos_info)

        # due to the plausability checks on top, we know that items_in_bin
        # is non-empty and that it contains main_item
        other_items = items_in_bin[:]
        other_items.remove(main_item)
        
        # remove items not good for bin2bin
        notgood = ["cherokee_easy_tee_shirt", "creativity_chenille_stems", "kleenex_paper_towels", "scotch_bubble_mailer", "hanes_tube_socks", "fiskars_scissors_red", "oral_b_toothbrush_red", "oral_b_toothbrush_green", "cool_shot_glue_sticks", "dasani_water_bottle"]
        for ng in notgood:
            if ng in other_items:
                loginfo("remove %s from the list of potential bin2bin candidates" % ng)
                other_items.remove(ng)

        # if we are lucky, `other_item_priorities` contains all known
        # item_ids in an easy-to-move order
        try:
            other_item_priorities = other_item_priorities.tolist()
        except:
            other_item_priorities = []
        if other_item_priorities:
            other_items.sort(key=lambda x: other_item_priorities.index(util.item_string_to_id(x)))
        
        if len(other_items) == 0:
            # there is no item to move away, we simply failed to detect
            # the item in the bin
            logwarn("we failed to detect %s with all strategies, and " %
                    main_item + "there were no other (movable) items :-(")
            return (False, main_item_strategy, []) #FAIL_MODE
        else:
            moved_items = []
            for try_item in other_items:
                loginfo("we try to move %s out of the way" % try_item)
                # find stow position in case we can take this item
                loginfo("find a location to move that item to")
                (stow_bin, stow_pos) = stow_strategy.find_location(try_item,
                                                                   forbidden_bin=source_bin)
                loginfo("if we succeed to take %s, we will put it to %s/%s" %
                        (try_item, stow_bin, stow_pos))
                # maybe find_location has moved the arm away, but
                # moving back is the responsibility of can_grab_item,
                # so we do not explicitly do that here
                # move the arm back

                # now try to take that item
                (result, strategy, _) = self.__try_take_item(try_item, source_bin,
                                                             items_in_bin)
                if result == GrabResult.SUCCESS:
                    break
                else:
                    loginfo("failed to take %s, try next item" % try_item)
            else:
                # if we arrive here, we were not able to grab a single
                # item from that bin. this means failure
                return (False, main_item_strategy, [])

            # if we arrive here, we were able to take try_item from the bin
            try:
                loginfo("trying to stow %s in %s" % (try_item, stow_bin))
                stow_success = strategy.stow_in_shelf(stow_bin, stow_pos, item=try_item)
            except Exception as e:
                # uh, this is not good, we removed the item from the shelf,
                # but we failed to put it somewhere.
                logerr("error while stowing item %s in shelf: %s" % (try_item, e))
                self.pos_info.take_from_bin(source_bin, try_item)
                self.pos_info.drop(try_item)
                return (False, main_item_strategy, [])
            if not stow_success:
                # we failed to put the item in the shelf, but still it
                # is removed out of the previous shelf, so we basically
                # achieved our intermediate goal, even if we will get
                # minus points. proceed like normal
                logwarn("we failed to stow item %s in shelf" % try_item)
                self.pos_info.take_from_bin(source_bin, try_item)
                self.pos_info.drop(try_item)
            else:
                loginfo("succeeded to stow item %s in shelf" % try_item)
                self.pos_info.take_from_bin(source_bin, try_item)
                self.pos_info.put_into_bin(stow_bin, try_item)

            # write intermediate output file
            util.write_output_file(self.pos_info, "current_pick_status.json")

            # now that we moved one item away, start over again
            remaining_items_in_bin = items_in_bin[:]
            remaining_items_in_bin.remove(try_item)
            (success, strategy,
             sub_moved_items) = self.get_item(main_item, source_bin,
                                          remaining_items_in_bin)
            return (success, strategy, moved_items + sub_moved_items)


    def __try_take_item(self, item, bin, items_in_bin):
        strategies = grab.select_strategies(item, bin)
        loginfo("available strategies for %s in %s: %s" % (item, bin, strategies))
        tried_strategies = 0

        # store in this list the order in which we
        # would like to move items away, if necessary
        secondary_item_priorities = []

        # try all possible grab strategies
        for i, grab_strategy in enumerate(strategies):
            if i < len(strategies) - 1:
                strat_suffix = "trying next strategy"
            else:
                strat_suffix = "giving up"

            max_tries = self.get_max_tries(item, grab_strategy)
            for k in range(max_tries):
                # check if this strategy can locate the item
                try:
                    can_grab = grab_strategy.can_grab_item(item, bin,
                                                           items_in_bin)
                except Exception as e:
                    logerr("error while checking for grabability: %s" % e)
                    raise e

                if not can_grab:
                    loginfo("strategy %s cannot grab item %s; %s" %
                            (grab_strategy, item, strat_suffix))
                    # if this is the VacuumGrabbing strategy, we use
                    # the chance to get a list of items we would like
                    # to move away in some priority order
                    if isinstance(grab_strategy, grab.vacuum.VacuumGrabbing):
                        secondary_item_priorities = grab_strategy.sort_item_priority_bin()
                    # it makes no sense to retry can_grab_item,
                    # so we switch to the next strategy
                    break
                loginfo("can_grab_item was True, try strategy %s for item %s" %
                        (grab_strategy, item))

                if k == 0:
                    # increate this only once per strategy, even
                    # if we tried multiple times
                    tried_strategies += 1

                # try to grab the item with this strategy
                try:
                    success = grab_strategy.grab_from_shelf(item, bin)
                except Exception as e:
                    logerr("error while taking item %s: %s" % (item, e))
                    # NB. this is not good, we have no idea what happened here;
                    # the arm is in an undefined state
                    raise e

                if success:
                    loginfo("succeeded to take item %s" % item)
                    # NB. We only make attempts at items at the front, so
                    # the last parameter is empty.
                    return (GrabResult.SUCCESS, grab_strategy, secondary_item_priorities)
                else:
                    if k < max_tries - 1:
                        retry_msg = "; trying again"
                    else:
                        retry_msg = "(max retries reached); " + strat_suffix
                    loginfo("failed to take item %s using %s%s" %
                            (item, grab_strategy, retry_msg))

        # if we arrive here, none of the strategies succeeded
        # to grab the item. if tried_strategies is 0, then this is
        # because can_grab was always False; otherwise we tried at
        # least one strategy, but all of them failed.
        if tried_strategies > 0:
            return (GrabResult.FAILED_GRAB, grab_strategy, secondary_item_priorities)
        else:
            return (GrabResult.CANNOT_GRAB, grab_strategy, secondary_item_priorities)
