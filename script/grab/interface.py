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


class GrabStrategy(object):
    def can_grab_item(self, item, bin, items_in_bin):
        """Check whether this strategy can probably grab the
        given item in the given bin/tote, doing localization
        where necessary. (That is, it is visible and we are
        confident that we can take it.)"""
        raise NotImplementedError("override this")

    def grab_from_shelf(self, item, bin):
        """Compute a good position to grab the given
        item from the given bin and try to take it."""
        raise NotImplementedError("override this")

    def grab_from_tote(self, item):
        """Compute a good position to grab the given
        item from the tote and try to take it."""
        raise NotImplementedError("override this")

    def stow_in_shelf(self, target_bin, target_position):
        """Move the arm to the given bin and drop
        the item currently grabbed at the given position."""
        raise NotImplementedError("override this")

    def stow_in_tote(self, target_position):
        """Move the arm above the tote and
        drop the item currently grabbed."""
        raise NotImplementedError("override this")
