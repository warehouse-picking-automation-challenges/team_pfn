This file describes the licenses for the files found in this repository.

Most (plain text) files have a comment section at the top mentioning
the license for the respective file. In case where 

  1) there is no such comment in a file and
  2) that file is not mentioned in the paragraphs below,

the license is Apache 2.0 as described on
<http://www.apache.org/licenses/LICENSE-2.0>.

The files

  - `item_data.csv`
  - `models/*.blend`
  - `models/*.png`
  - `models/*.ipynb`
  - `test-tasks/*.json`

do not allow adding a comment header due to their file format. They
were originally created by Preferred Networks and also licensed under
the Apache 2.0 license.

The files

  - `json-examples/*.json`
  - `photos/*.png`

were provided by the APC organizers. They are only included here for
reference.

The files

  - `dashboard/eventemitter2.min.js`
  - `dashboard/roslib.min.js`

were included unmodified and do not carry a license header in the original
version. They are part of [roslibjs](http://wiki.ros.org/roslibjs) and
licensed under the three-clause BSD license.

The file

  - `dashboard/vis.min.css`

was included unmodified and does not carry a license header in the original
version. It is part of [vis.js](http://visjs.org/) and licensed under both
Apache 2.0 and MIT license.

The directory `win/win_roscpp` contains parts of the original ROS source
code and various files automatically generated from that source code for
simplified development on Windows. All code there is under the respective
original licenses. The directory `win/win_node/lib` contains the compiled
version of that code.

The directory `win/win_node/include` contains C++ header files from 3rd
party projects (mostly ROS) and files automatically generated from other
source files in the repository. All code there is under the respective
original licenses.

The directory `win/win_node/x64/Release` contains the final executables.
