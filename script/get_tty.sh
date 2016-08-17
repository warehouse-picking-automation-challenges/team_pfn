#!/bin/bash

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

function get_tty {

  (
  flock 200

  touch /tmp/usedttys.txt

  # check if there is a free ttyACM file
  for tty in /dev/ttyACM*
  do
    if grep -Fxq "$tty" /tmp/usedttys.txt
    then
      continue
    else
      # append to file and print it if not found
      echo "$tty" >> /tmp/usedttys.txt
      echo "$tty"
      break
    fi
  done

  ) 200>/tmp/ttylock

}

function release_tty {

  (
  flock 200

  touch /tmp/usedttys.txt

  # remove the device from the file
  grep -v "$1" /tmp/usedttys.txt > /tmp/current_tty
  mv /tmp/current_tty /tmp/usedttys.txt

  ) 200>/tmp/ttylock

}
