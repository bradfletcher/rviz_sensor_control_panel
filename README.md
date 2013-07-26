Questions/Support: Brad Fletcher brad@udel.edu

Thanks to Grey at GT for allowing me to use the "hubo init" program as a model.
 
RViz plugin for working with sensors.

As was decided at the Drexel meeting in February, GUIs for operating Hubo on Linux
are going to be ROS-based (specifically using RViz).

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
_______________________________  IF  YOU  ARE  NEW  TO  ROS  _________________________________
I strongly recommend looking over the tutorials for ROS: http://www.ros.org/wiki/ROS/Tutorials
You will not technically need to understand ROS or how it works to use hubo_init, but
it is not difficult to learn and will certainly be worthwhile down the road. Regardless,
below I am providing instructions to install and run rviz_sensor_control_panel which should be sufficient
whether or not you are familiar with ROS.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
___________________________ PREREQUISITES ______________________________

This package depends on ROS.

To obtain ROS, follow the instructions here (BE SURE TO INSTALL GROOVY):
http://www.ros.org/wiki/groovy/Installation/Ubuntu

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

~~~~~~~~~~~~~~~~~~~~~~~~~~
_________________Installation (after satisfying the Prerequisites above):____________________

If you do not have a catkin workspace (or you do not know what a catkin workspace is), copy/paste
the following block into a terminal:

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

From now on, the directory ~/catkin_ws/src will be where you keep your ROS package code.

To install rviz_sensor_control_panel, copy/paste the following block into a terminal:

cd ~/catkin_ws/src
git clone https://github.com/bradfletcher/rviz_sensor_control_panel.git
cd ~/catkin_ws
catkin_make
~~~~~~~~~~~~~~~~~~~~~~~~~~~


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
____________________________ USAGE ___________________________________
To use rviz_sensor_control_panel, do the following:

1) Open a terminal and type in:
    $ roscore
   This will eat up the terminal. Minimize it or move it out of the way.

2) From your workstation (whatever desktop or laptop you use), run
    $ rosrun rviz rviz

3) If this is your first time using hubo_init, do the following:
    a) In the menu bar at the very top, click on 'Panels'
    b) Click on 'Add new panel'
    c) Select rviz_sensor_control_panel
    d) A panel should appear in rviz with options to send Hokuyo commands, Camera commands, IMU commands, and Dynamixel commands.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

