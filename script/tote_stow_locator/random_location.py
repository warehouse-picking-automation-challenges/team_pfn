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

import interface
import numpy
import random
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../')
import util

class RandomLocationStow(interface.ToteStowLocatorStrategy):
    def find_location(self, item):
        self.tote_x = [680,740] #680.780 
        self.tote_y = [-680, -160] #use only 1/3
        self.tote_z = -300
        left_margin = 120
        margin = 50
        margin_l = 100
        #Roast Coffee(4), Hanes Tube Socks(7), Kame-chan(10)
        #Kleenex(20) & Kleenex(36)
        if util.item_string_to_id(item) in [4,7,10,20,36]:
            l = numpy.asarray([random.randint(self.tote_x[0],self.tote_x[1]),random.randint(self.tote_y[0]+margin_l,self.tote_y[1]-margin_l-left_margin),self.tote_z + margin_l,165,0,-90])
        # water (25) and bottle_brush(14)
        elif util.item_string_to_id(item) in [25,14,5]:
            l = numpy.asarray([random.randint(self.tote_x[0],self.tote_x[1]),random.randint(self.tote_y[0]+margin_l,self.tote_y[1]-margin_l-left_margin),self.tote_z,165,-0.0,-90])
        else:
            l = numpy.asarray([random.randint(self.tote_x[0],self.tote_x[1]),random.randint(self.tote_y[0]+margin,self.tote_y[1]-margin-left_margin),self.tote_z,165,-0,-90])
        print l
        return l

if __name__ == '__main__':
    a = RandomLocationStow(1)
    for i in range(10):
        a.find_location('i_am_a_bunny_book')
        a.find_location('folgers_classic_roast_coffee')
        a.find_location('dr_browns_bottle_brush')
