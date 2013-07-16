Questions/Support: brad@udel.edu

Thanks to Grey at GT for allowing me to modify portion of the GT 
'hubo init" program.  mxgrey@gatech.edu

RViz plugin for working with sensors.

As was decided at the Drexel meeting in February, GUIs for operating Hubo on Linux
are going to be ROS-based (specifically using RViz). This is the first step in making
that integration a reality.


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

This package depends on both ROS.

To obtain hubo-ach, follow the instructions here:
https://github.com/hubo/hubo-ach

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


To install hubo_init, copy/paste the following block into a terminal:

cd ~/catkin_ws/src
git clone https://github.com/bradfletcher/rviz_sensor_control_panel.git
cd ~/catkin_ws
catkin_make
~~~~~~~~~~~~~~~~~~~~~~~~~~~


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
____________________________ USAGE ___________________________________
To use hubo_init, do the following:

1) Open a terminal and type in:
    $ roscore
   This will eat up the terminal. Minimize it or move it out of the way.

2) From your workstation (whatever desktop or laptop you use), run
    $ rosrun rviz rviz

3) If this is your first time using hubo_init, do the following:
    a) In the menu bar at the very top, click on 'Panels'
    b) Click on 'Add new panel'
    c) Select ud_panel

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

