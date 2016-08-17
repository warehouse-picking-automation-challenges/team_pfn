# apc2016

## Install Instructions

* Install ROS and `source` the setup scripts.
* Let `WS` be an environment that holds the path to your ROS workspace.  
  For example, to create a new workspace, run

      export WS=~/workspace/apc/ws
      mkdir -p $WS/src && cd $WS/src
      catkin_init_workspace
      cd ..
      catkin_make
      . devel/setup.bash
* Clone the code repository into the `$WS/src` directory:  
    `cd src/ && git clone ...`
* Compile the code to create message definitions etc:  
    `cd $WS && catkin_make`
* In the robot controllers, set the server IP to the IP address of
  the server running the `arm_control_left/right` nodes.

## Run Instructions

* In one terminal, run `roscore`.
* In another terminal, run `roslaunch launch-base.xml`.
* On a Windows machine, start the RealSense and FX8 `bat` files
  with the correct `ROS_MASTER_URI`.

Now all the services are running and you can run
`python script/main_pick.py path/to/taskfile.json`

## Restarting/Reloading Components

All (most?) nodes in the `launch-base.xml` file are respawned on
exit. This can be used to restart a single node, for example if
the code has changed or if there has been some network issue that can
be fixed most easily with a restart.

* To restart the `arm_control_left/right` node, run
  `pkill -f arm_control_left` or `pkill -f arm_control_right`,
  respectively.
* To restart all of the Arduino nodes (this may be necessary
  every couple of hours in order to recalibrate pressure values),
  run `pgrep -f run_arduino`.

If you want to shutdown a certain node temporarily because you want to
run it on another machine, one possible solution is to insert

    import time
    while True:
        time.sleep(5)

into the respective file before the actual program code and then kill
the node. That way the actual code is never executed, still the
other nodes are unaffected.

## Notes

* The Arduino components are usually available as `/dev/ttyACM0`,
  `/dev/ttyACM1`, and `/dev/ttyACM2`. However, there may be situations
  when these devices get renumbered (for example, to `...3`, `...4`,
  `...5`). To deal with that situation there is a wrapper script
  `arduino.sh` that picks "the first unused `ttyACM` device" and uses
  it. The list of devices that are
  currently in use is stored in `/tmp/usedttys.txt`. If a process
  crashes so hard that it is unable to remove the used device from
  that file after exit, it may become necessary to remove the device
  name manually.
* The `gripper_angle` that is responsible for opening and closing
  the gripper hand maintains a file `gripper_angle` in the
  `script/hand` folder that contains the total angle of the rotary
  servo (because the servo itself only remembers values in the range
  [0, 360]). That is, if the machine that executes this node changes,
  the angle must be recalibrated. In order to do so:

  * Shut down the `actuator_nodes.py` script if it is running.
  * Write a large number (such as 5000) to the `gripper_angle`
    file (create it if necessary).
  * Start the `actuator_nodes.py` script. When doing so, the
    rotary servo moves back (or forward) to the next multiple
    of 360, e.g., 5040 degrees.
  * Decrease the angle in steps of 360 degrees until the gripper
    is only slightly opened, i.e., the last step before the gripper
    closes.
  * Stop the `actuator_nodes.py` script.
  * Write `720` to the `gripper_angle` file.
  * Now, when you restart the `actuator_nodes.py` script, you will
    be able to send values between 0 (tightly closed) and something
    like 4000 (wide open).

## Conventions

The numbering of the shelf bins is as follows:

     A | B | C
    ---+---+---
     D | E | F
    ---+---+---
     G | H | I
    ---+---+---
     J | K | L

## About Item Names

The file `item_data.csv` contains the list of item types together with a `[a-z0-9_]+`-shaped string identifier and a numeric ID.
Whenever possible, when referring to an item type in code or data files, use these identifiers and avoid using other IDs.
