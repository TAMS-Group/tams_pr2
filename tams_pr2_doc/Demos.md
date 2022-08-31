# !!!ONLY FOR INSTRUCTED USERS!!!

Don't know the password? Don't bother reading ahead.

List of instructed users:
* Michael Görner
* Yannick Jonetzko
* Shuang Li


## HowTo Demo on the PR2:


* Boot Basestation
    * optionally: plug in network and power on PR2
* Boot PR2
    * ensure fuse status
    * deactivate run-stop
    * await 6 sounds (4 sequences, 1 single beep and 1 double beep)
* Connect to PR2: `ssh c1`
    * optionally: use screen session
* Launch order (in single sessions/windows on c1):
    * `roslaunch tams_pr2_bringup tams_pr2.launch`
        * ensure safety space around PR2 for calibration (Torso will move up and down, the left arm and the head whirl around)
        * move away right arm (with Shadow Hand) to the right side
        * activate run-stop
            * if the left arm is stuck, help by pushing the arm upwards (due to counterweight calibration)
            * if the tilt laser is stuck on one of its limits, move it by hand in the other direction until it releases (it found the resp. 0-crossing)
        * wait for calibration routine to finish
        * include Shadow hand bringup preocess, so we do not need to launch on docker-c2 then separately run `roslaunch tams_motorhand right_biotac_hand_in_ns.launch` anymore
    * `rosrun tams_pr2_bringup calibrate_right_arm.py`
        * confirm calibration of 5 joints in right arm
        * use seconds person to assist in calibration
* Optionally: use RViz and Moveit to move arms or right hand
* VOCUS Demo
    * `roslaunch vocus2_ros pr2_vocus2.launch`
* Chinese counting Demo
    * use RViz to move hand into *open* configuration
    * `rosrun count_demo count_demo_node`
    * Pay attention to the hand movements, especially the thumb and support the motion if required
    * by default the demo velocity is limited by factor .2, to demonstrate maximum (configured) speed, run
        * `rosrun count_demo count_demo_node _velocity:=1.0`

#### Issues/Be careful
* Controller
    * when using the controller to move the head, there may be a conflict with the VOCUS demo. Do not use at the same time!
* Scripts (contains script to modify states on PR2)
    * grow_tall.sh -- move up the spine
    * grow_small.sh -- move down the spine
    * move_tilt_scanner.sh -- enable the tilt laser
    * stop_tilt_scanner.sh -- disable the tilt laser
    * ...
    * See the Scriptpanel to activate them from buttons

##### Turning it off
* disable the run-stop
    * ATTENTION! Motors will turn off! Catch head and arms if required to avoid damage
* Kill all sessions (in reverse order)
* `pr2_shutdown.sh` on the base station(!)
    * wait for 4 sequences of beeps
    * turn off power switch
    * unplug (wall side first) or turn off switch on power outlet

##### If something is not working:
Have you tried turning it off and on again? Still doesn't work? Call for an emergency developer meeting!
