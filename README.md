# auton_sub
This repo is for the UWF HydroBotics Team
Run on Jetson Orin Nano Jetpack 6.1
Pixhawk 4
Wayfinder DVL

The main files that have been tested are anything related to prequal_launch.py
Currently the depth hold works, but the sub likes to rotate instead of driving forwards when I tell it to
You may want to check whether the x and y directions are correctly used in robot_control

The other mission files besides prequal.py and prequal_manual.py were written for a previous version of robot_control and need to be updated/have not been tested in the water
Missions folder contains all the different missions
Motion contains robot_control and the rosbag launcher
Utils contains small items such as arm/disarm/set guided mode
CV contains anything related to the image recognition and models contains the different image recognition files but they are not accurate based on the names.
Sensors contains the files for the dvl and leak sensor, these files should be good and probably dont need to be changed.

Some items require installs on the Jetson and I have added the things that need to be installed to the tops of the files in comments, but not on all of them.

