## PicknPlace-for-Open_Manipulator
This is a pick and place implementation for the Open Manipulator robot using Noetic and the keyboard teleoperation tutorial as the foundation

The base of the project is found in the Open Manipulator teleoperation tutorial. Link for the original project https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/
The repository of the original project link 
https://github.com/ROBOTIS-GIT/open_manipulator/tree/master/open_manipulator_teleop

Project for the Kinematics and Dynamics of Robots course at the Universidad de las Americas Puebla

## Instructions:
Install ROS Noetic in Ubuntu 20.04
Create a workspace and a package. This tasks are included in the wiki ros tutorials available at http://wiki.ros.org/ROS/Tutorials
Once having the package correctly, open a terminal
Move to the workspace folder
Move to the source folder 
Clone the repository with the git clone instruction and the link of the project: git clone https://github.com/JosueLara22/PicknPlace-for-Open_Manipulator.git
Go back to the workspace folder with cd ..
Perform a catkin_make to update the package 

Make the connections of the robot 
Use the next instruction to run the pick and place application: roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch

##Enjoy, have fun and improve the program!
