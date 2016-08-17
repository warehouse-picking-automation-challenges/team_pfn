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

from vacuum import VacuumGrabbing

import util
if util.is_gripper_enabled():
    from gripper import GripperGrabbing


_vac_grab = None
_grip_grab = None


def select_strategies(item, bin):
    global _vac_grab, _grip_grab
    if not _vac_grab:
        _vac_grab = VacuumGrabbing()
    if util.is_gripper_enabled() and not _grip_grab:
        _grip_grab = GripperGrabbing()

    if not util.is_gripper_enabled():
        return [_vac_grab]

    if bin == "tote":
        # for picking items from the tote,
        # always choose vacuum for now
        return [_vac_grab]

    if item in ["fitness_gear_3lb_dumbbell"]:
        # only gripper
        return [_grip_grab]

    elif item in ["rolodex_jumbo_pencil_cup"]:
        # gripper, then vacuum
        return [_grip_grab, _vac_grab]

    elif item in ["scotch_bubble_mailer_DISABLED"]:
        # vacuum, then gripper
        return [_vac_grab, _grip_grab]

    # only vacuum
    return [_vac_grab]
