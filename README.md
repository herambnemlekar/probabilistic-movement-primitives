# ProMP

## Description

Probabilistic Movement Primitives implementation for learning motion of basic tasks in robotics, based on the research of [Alexandros Paraschos, Christian Daniel, Jan Peters, and Gerhard Neumann at TU Darmstadt, Germany.](http://www.ias.tu-darmstadt.de/uploads/Publications/Paraschos_NIPS_2013a.pdf) 

## Installation

1. Install the Baxter workspace

2. Install baxter-pykdl

3. Enter your ROS workspace
 * `cd ~/ros_ws/src`

4. Clone the following repositories into your ros_ws/src folder
  * `git clone https://github.com/baxter-flowers/promplib.git`
  * `git clone https://github.com/baxter-flowers/baxter_commander.git`
  * `git clone https://github.com/baxter-flowers/trac_ik_baxter.git`
  * `git clone https://github.com/baxter-flowers/trac_ik.git`

5. Install the baxter_commander dependancies
  * `sudo apt-get install ros-indigo-moveit-full python-pip`
  * `sudo pip install xmltodict`

6. If you get _Invoking "make install -j8 -l8" failed_ due to _error: package directory 'src/trajectories' does not exist_
  * `cd ~ros_ws/src/baxter_commander/src`
  * `mkdir -p trajectories`
  * `cd ~ros_ws/`

7. Build
  * `catkin_make`
  * `catkin_make install`
  * `source devel/setup.bash`

### Installation / Runtime errors

1. _fatal error: nlopt.hpp: No such file or directory_
  * `sudo apt-get install libnlopt-dev`
  
2. _Import error: no module name ros_
  * `cd ~/ros_ws/src/promplib`
  * `gedit setup.py`
  * edit this line to be _d['packages'] = ['promp', 'promp.ros']_ and save and close the file
  * `sudo python setup.py install`
  * `cd ~/ros_ws | catkin_make | catkin_make install`

3. _OSError: [Errno 110] Right limb init failed to get current joint_states from robot/joint_states_
  * Run the code on Baxter's computer to allow access of joint angles publisher thread
  
4. If code is stuck at: *rospy.wait_for_service(self._kinematics_names['ik'][self._selected_ik])* or gives error: _rospy.exceptions.ROSInterruptException: rospy shutdown_
  * Comment line - of file.py

