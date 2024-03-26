## Please only use the robot after a proper introduction from an existing user

List of instructed users:
* Michael Görner
* Yannick Jonetzko
* Hui Zhang

## HowTo Use the PR2:


* Boot Basestation
    * optionally: plug in network and power to PR2
* Boot PR2
    * ensure fuse status
    * deactivate run-stop
    * await 6 sounds (4 sequences, 1 single beep and 1 double beep)
* Connect to PR2:
    * `ssh c1` or `ssh pr2`. Either connects into the most current docker container (at the. time of writing c1-noetic)
      type `ssh c<TAB><TAB>` for a list of all the ways to connect to the pr2 c1/c2 servers
    * optionally but recommended: use `screen` or `tmux` (and get familiar with it if you are not)
* Launch:
    * Launch the PR2Dashboard, the scriptpanel, and RViz on the basestation
    * `roslaunch tams_pr2_bringup tams_pr2.launch`
        * ensure safety space around PR2 for calibration (PR2 will move up and down, the left arm and the head)
        * move right arm (with Shadow Hand) safely to the right side
        * activate run-stop
            * if the tilt laser is stuck on the bottom limit, move it by hand in the other direction until it releases (it found the resp. 0-crossing)
        * wait for calibration routine to finish
    * `rosrun tams_pr2_bringup calibrate_right_arm.py` (or use scriptpanel button)
        * confirm calibration of 5 joints in right arm, keep hand away from collisions

* VOCUS Demo
    * `roslaunch vocus2_ros pr2_vocus2.launch`
* Chinese counting Demo
    * use RViz to move hand into *open* configuration
    * `rosrun count_demo count_demo_node`
    * Pay attention to the hand movements, especially the thumb and support the motion if required
    * by default the demo velocity is limited by factor .2, to demonstrate maximum (configured) speed, run
        * `rosrun count_demo count_demo_node _velocity:=1.0`

#### Issues/Be careful

* familiarize yourself with the buttons in the Scriptpanel. They are all mapped to scripts in `~/scripts/`, so check the files if you don't understand what they do. Some examples are,
    * grow_tall.sh -- move up the spine
    * grow_small.sh -- move down the spine
    * move_tilt_scanner.sh -- enable the tilt laser
    * stop_tilt_scanner.sh -- disable the tilt laser
    * ...
* Controller
    * only run one script to command the head position. There is a central multiplexer (the `/look` node) that can be patched to support other modes.

#### Turning it off

* disable the run-stop
    * ATTENTION! Motors will turn off! Catch head and arms to avoid damage long-term
* <Ctrl>+C all scripts on the robot machines
* `pr2_shutdown.sh` on the base station(!)
    * wait for 4 sequences of beeps
    * turn off power switch
    * turn off switch on power outlet

#### If something is not working

Have you tried turning it off and on again? Still doesn't work? Call for an emergency developer meeting!
