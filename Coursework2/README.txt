Running

To run the packages, in a terminal, ensure that setup.bash file is run first. The command is as follows:
$ source catkin_ws/devel/setup.bash

After that run the launch file from the package:
$ roslaunch my_robot my_robot.launch

A new terminal will open asking the user for input. It might be hidden behind Rviz so check the taskbar. 
The window in Rviz might show an error but it requires an input from a user first, before the all the joints can be initialized. 
After the first input, the Rviz will not show an error anymore. 
Close the new terminal for the input before running the launch file a second time.
