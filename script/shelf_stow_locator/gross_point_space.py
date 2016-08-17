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

import interface

class GrossPointSpaceStow(interface.ShelfStowLocatorStrategy):
    def find_location(self, item, forbidden_bin=None):
        scores = self.pos_info.score_to_stow_1(item)
        
        if item=="folgers_classic_roast_coffee" or item=="hanes_tube_socks":
            bin_list = scores
            bin_list["bin_D"] -= 10
            bin_list["bin_E"] -= 10
            bin_list["bin_F"] -= 10
            bin_list["bin_G"] -= 10
            bin_list["bin_H"] -= 10
            bin_list["bin_I"] -= 10
        elif item=="scotch_bubble_mailer" or item=="creativity_chenille_stems" or item=="peva_shower_curtain_liner" or item=="kleenex_paper_towels" or item=="dr_browns_bottle_brush" or item=="dasani_water_bottle":
            bin_list = scores
            bin_list["bin_A"] -= 10
            bin_list["bin_C"] -= 10
            bin_list["bin_D"] -= 10
            bin_list["bin_F"] -= 10
            bin_list["bin_G"] -= 10
            bin_list["bin_I"] -= 10
            bin_list["bin_J"] -= 10
            bin_list["bin_L"] -= 10
        elif item=="cherokee_easy_tee_shirt":
            bin_list = scores
            bin_list["bin_A"] -= 20
            bin_list["bin_B"] -= 20
            bin_list["bin_C"] -= 20
            bin_list["bin_D"] -= 20
            bin_list["bin_E"] -= 20
            bin_list["bin_F"] -= 20
            bin_list["bin_G"] -= 20
            bin_list["bin_H"] -= 20
            bin_list["bin_I"] -= 20
        else:
            bin_list = scores
        
        bin = max(bin_list.items(), key=lambda x:x[1])[0]
        
        return ( bin , (0, 0) )
        
        
