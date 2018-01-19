# ProMP

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

1. If you get _fatal error: nlopt.hpp: No such file or directory_
  * `sudo apt-get install libnlopt-dev`
  
2. If you get _Import error: no module name ros_
  * `cd ~/ros_ws/src/promplib`
  * `gedit setup.py`
  * edit this line to be _d['packages'] = ['promp', 'promp.ros']_ and save and close the file
  * `sudo python setup.py install`
  * `cd ~/ros_ws | catkin_make | catkin_make install`
