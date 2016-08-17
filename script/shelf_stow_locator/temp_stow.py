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

class TempStow(interface.ShelfStowLocatorStrategy):
    def find_location(self, item, forbidden_bin=None):
        scores = self.pos_info.score_to_temp_stow(item)
        if forbidden_bin :
            scores[forbidden_bin] -= 10
        bin = max(scores.items(), key=lambda x:x[1])[0]
        
        return ( bin , (0, 0) )
        
        
