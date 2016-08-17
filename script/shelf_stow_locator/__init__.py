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

from temp_stow import TempStow
from gross_point_space import GrossPointSpaceStow


def select_bin2bin_strategy(pos_info):
    # select a strategy that returns the emptiest bin
    # from the history
    return TempStow(pos_info)


def select_tote2bin_strategy(pos_info):
    # select a strategy that weights points we get and
    # free space
    return GrossPointSpaceStow(pos_info)
