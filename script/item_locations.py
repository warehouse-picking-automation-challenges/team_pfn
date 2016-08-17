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

import json
from collections import OrderedDict

import util

(logdebug, loginfo, logwarn, logerr) = util.get_loggers("ItemLocations")


class ItemLocations:
    allowed_items = set([
        "i_am_a_bunny_book",
        "laugh_out_loud_joke_book",
        "rawlings_baseball",
        "folgers_classic_roast_coffee",
        "scotch_bubble_mailer",
        "elmers_washable_no_run_school_glue",
        "hanes_tube_socks",
        "womens_knit_gloves",
        "cloud_b_plush_bear",
        "kyjen_squeakin_eggs_plush_puppies",
        "creativity_chenille_stems",
        "oral_b_toothbrush_red",
        "cool_shot_glue_sticks",
        "dr_browns_bottle_brush",
        "fiskars_scissors_red",
        "platinum_pets_dog_bowl",
        "fitness_gear_3lb_dumbbell",
        "rolodex_jumbo_pencil_cup",
        "expo_dry_erase_board_eraser",
        "kleenex_tissue_box",
        "crayola_24_ct",
        "dove_beauty_bar",
        "staples_index_cards",
        "up_glucose_bottle",
        "dasani_water_bottle",
        "cherokee_easy_tee_shirt",
        "peva_shower_curtain_liner",
        "barkely_hide_bones",
        "soft_white_lightbulb",
        "safety_first_outlet_plugs",
        "command_hooks",
        "easter_turtle_sippy_cup",
        "scotch_duct_tape",
        "woods_extension_cord",
        "clorox_utility_brush",
        "kleenex_paper_towels",
        "ticonderoga_12_pencils",
        "jane_eyre_dvd",
        "oral_b_toothbrush_green"
    ])

    bin_volume = {
        "bin_A": 26*22*42,
        "bin_B": 30*22*42,
        "bin_C": 26*22*42,

        "bin_D": 26*17*42,
        "bin_E": 30*17*42,
        "bin_F": 26*17*42,

        "bin_G": 26*17*42,
        "bin_H": 30*17*42,
        "bin_I": 26*17*42,

        "bin_J": 26*22*42,
        "bin_K": 30*22*42,
        "bin_L": 26*22*42
    }

    def __init__(self, shelf_contents, tote_contents, work_order ={}):
        loginfo("initializing item locations")
        # shelf_contents is a dictionary
        #   {"bin_X": ["itemA", "itemB", ...], ...}
        # tote_contents is a list
        #  ["itemA", "itemB", ...]

        # shelf items
        self.shelf = {}
        # shelf items in the beginning (required for
        # point computation)
        self.shelf_origin = {}
        for bin, items in shelf_contents.iteritems():
            unknown_items = set(items) - ItemLocations.allowed_items
            if unknown_items:
                raise ValueError("unknown item types: %s" % unknown_items)
            self.shelf[bin] = items[:]
            self.shelf_origin[bin] = items[:]

        # tote items
        unknown_tote_items = set(tote_contents) - ItemLocations.allowed_items
        if unknown_tote_items:
            raise ValueError("unknown item types: %s" % unknown_items)
        self.tote = tote_contents[:]
        
        self.task_items = {}
        for obj in work_order:
            self.task_items[obj["bin"]] = obj["item"]

        self.pending = []

    def __check_item__(self, item):
        if item not in ItemLocations.allowed_items:
            raise ValueError("unknown item type: %s" % item)

    def __check_bin__(self, where):
        if where not in self.shelf:
            raise ValueError("unknown bin: %s" % where)

    def take_from_bin(self, where, item):
        self.__check_item__(item)
        self.__check_bin__(where)
        if item not in self.shelf[where]:
            raise ValueError("item %s not present in bin %s" % (item, where))
        self.shelf[where].remove(item)
        self.pending.append(item)
        loginfo("took item %s from bin %s" % (item, where))

    def put_into_bin(self, where, item):
        self.__check_item__(item)
        self.__check_bin__(where)
        if item not in self.pending:
            raise ValueError("item %s was not picked up before" % item)
        self.pending.remove(item)
        self.shelf[where].append(item)
        loginfo("put item %s into bin %s" % (item, where))

    def take_from_tote(self, item):
        self.__check_item__(item)
        if item not in self.tote:
            raise ValueError("item %s not present in tote" % item)
        self.tote.remove(item)
        self.pending.append(item)
        loginfo("took item %s from tote" % item)

    def put_into_tote(self, item):
        self.__check_item__(item)
        if item not in self.pending:
            raise ValueError("item %s was not picked up before" % item)
        self.pending.remove(item)
        self.tote.append(item)
        loginfo("put item %s into tote" % item)

    def drop(self, item):
        self.__check_item__(item)
        if item not in self.pending:
            raise ValueError("item %s was not picked up before" % item)
        self.pending.remove(item)


    def to_json(self):
        shelf = OrderedDict(sorted(self.shelf.items(), key=lambda t: t[0]))
        return json.dumps({"bin_contents": shelf,
                           "tote_contents": self.tote},
                          indent=4, sort_keys=True)

    def used_volume_in_bin(self, where , margin=0):
        # compute the total volume of all items in the given bin
        self.__check_bin__(where)
        volume = 0
        for item in self.shelf[where]:
            #volume += util.item_string_to_volume(item)
            volume += util.item_string_to_volume_with_margin( item ,margin)
        return volume

    def free_volume_after_add(self, item, where, margin=[1, 4]):
        # compute the percentage of free space in the given bin
        # if we add the given item to the current situation.
        # the margin parameter accounts for the fact that we cannot
        # fit all items perfectly (some kind of "safety factor").
        self.__check_bin__(where)
        return float(self.bin_volume[where] -
                     (self.used_volume_in_bin(where,margin[0]) +
                         util.item_string_to_volume_with_margin(item,margin[1]) )) / \
            self.bin_volume[where]

    def bin_stow_points(self, where):
        # return how many points we will got for storing an item
        # in the given bin
        self.__check_bin__(where)
        num = len(self.shelf_origin[where])
        if num >= 5:
            return 20
        if num >= 3:
            return 15
        if num >= 1:
            return 10
        return 0

    def bin_pick_points(self, where):
        # points for picking from bin are the same as stowing
        # into bin
        return self.bin_stow_points(where)
    
    # point * free space    
    def score_to_stow_1(self, item ):
        score = {}
        for bin in self.shelf.keys():
            score[bin] = self.free_volume_after_add(item,bin) * self.bin_stow_points(bin)
        return score
    
    # choose large space , +1 if task_item is not in bin    
    def score_to_temp_stow(self, item ):
        score = {}
        for bin , contents in self.shelf.items():
            score[bin] = self.free_volume_after_add(item,bin)
            if not (bin in self.task_items) or self.task_items[bin] not in contents:
               score[bin] += 1
        return score    
    
    # choose small contents number bin
    def score_to_pick(self,  not_pick = []):
        score = {}
        for bin , contents in self.shelf.items():
            if not bin in self.task_items:
                continue
            score[bin] = 10 - len(contents)
            if self.task_items[bin] in not_pick:
               score[bin] -= 10
        return score   
       
if __name__ == '__main__':
    task = json.load(open("../json-examples/apc_pick_task.json"))
    pos_info = ItemLocations(task["bin_contents"], task["tote_contents"],task["work_order"])
    score = pos_info.score_to_stow_1(util.item_id_to_string(39))
    print "score to stow"
    print sorted(score.items(), key=lambda x: x[1])
    
   
    pos_info.task_items["bin_E"] = ""
    score2 = pos_info.score_to_temp_stow(util.item_id_to_string(39))
    print "score to temp stow"
    print sorted(score2.items(), key=lambda x: x[1])
    print max(score2.items(), key=lambda x:x[1])[0]
    
    
    score3 = pos_info.score_to_pick()
    print "score to pick"
    print sorted(score3.items(), key=lambda x: x[1])
    
