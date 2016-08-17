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

import interface


class RandomLocationStow(interface.ShelfStowLocatorStrategy):
    def find_location(self, item, forbidden_bin=None):
        bin = random.choice(self.pos_info.shelf.keys())
        while bin == forbidden_bin:
            bin = random.choice(self.pos_info.shelf.keys())
        return (bin, (0.5, 0.5))
