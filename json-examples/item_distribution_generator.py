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

import numpy as np
import json

items = [
            "kleenex_tissue_box",
            "creativity_chenille_stems",
            "elmers_washable_no_run_school_glue",
            "scotch_bubble_mailer",
            "elmers_washable_no_run_school_glue",
            "scotch_bubble_mailer",
            "dasani_water_bottle",
            "womens_knit_gloves",
            "up_glucose_bottle",
            "folgers_classic_roast_coffee",
            "kyjen_squeakin_eggs_plush_puppies",
            "i_am_a_bunny_book",
            "cherokee_easy_tee_shirt",
            "kleenex_paper_towels",
            "easter_turtle_sippy_cup",
            "command_hooks",
            "jane_eyre_dvd",
            "laugh_out_loud_joke_book",
            "oral_b_toothbrush_red",
            "rolodex_jumbo_pencil_cup",
            "scotch_duct_tape",
            "platinum_pets_dog_bowl",
            "barkely_hide_bones",
            "rawlings_baseball",
            "fiskars_scissors_red",
            "fitness_gear_3lb_dumbbell",
            "oral_b_toothbrush_green",
            "cool_shot_glue_sticks",
            "staples_index_cards",
            "dove_beauty_bar",
            "staples_index_cards",
            "crayola_24_ct",
            "dasani_water_bottle",
            "creativity_chenille_stems",
            "safety_first_outlet_plugs",
            "woods_extension_cord",
            "soft_white_lightbulb",
            "cloud_b_plush_bear",
            "peva_shower_curtain_liner",
            "scotch_duct_tape",
            "hanes_tube_socks",
            "clorox_utility_brush",
            "ticonderoga_12_pencils",
            "expo_dry_erase_board_eraser",
            "expo_dry_erase_board_eraser",
            "dr_browns_bottle_brush"
]

bins = ["bin_A","bin_B","bin_C","bin_D","bin_E","bin_F","bin_G","bin_H","bin_I","bin_J","bin_K","bin_L"]

nums = [2,7,4,1,4,3,2,9,5,2,3,4]


for piyo in range(100):
    items = np.random.permutation(items)
    nums = np.random.permutation(nums)

    hoge = {}
    hoge['bin_contents'] = {}

    cnt = 0
    for i in range(12):
        hoge['bin_contents'][bins[i]] = items[cnt:(cnt+nums[i])].tolist()
        cnt += nums[i]

    json.dump(hoge, open('generated_imput_%d.json'%piyo,'w'), indent=2, sort_keys=True)
