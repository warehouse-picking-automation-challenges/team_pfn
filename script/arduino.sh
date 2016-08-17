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

if [ -z "$1" ]
  then
    echo "must pass an integer as argument"
  exit 1
fi

DIR="$(dirname "$0")"

source "$DIR/get_tty.sh"

MY_TTY=$(get_tty)

if [ -z "$MY_TTY" ]
  then
    echo "no free TTY present"
  exit 2
fi

python "$DIR/run_arduino.py" "arduino$1" "$MY_TTY"

release_tty "$MY_TTY"

