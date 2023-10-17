# Home Service Robot - Robotics Software Engineer Nanodegree Final Project (Udacity)

## This project builds a home service robot in ROS which delivers objects in an obstacle-filled environment.

## Workflow:
 1. Accept an initial navigation goal to a pickup location 
 2. Move towards the goal
 3. Pick up the object
 4. Accept a second navigational goal towards a dropoff point
 5. Dropoff the object
 
## Instructions:

 ### Prerequisites:

 The following ROS packages are required for running the project
 - turtlebot3_gazebo
 - turtlebot3_teleop
 - turtlebot3_slam
 - gmapping
 - dwa-local-planner

 Xterm is also required for the shell scripts:

 `sudo apt-get update`

 `sudo apt-get install xterm`

 *NOTE: This project is **not** intended for ROS distributions older than ROS Melodic.*

 Make sure the `TURTLEBOT3_MODEL` environment variable is set to a valid option in your .bashrc file:

 `echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc`

  `source ~/.bashrc`


 ### Cloning and building the project workspace
 `git clone https://github.com/Anna-LeeMcLean/home-service-robot`

 `cd home-service-robot`
 
 `catkin make`
 
 `source devel/setup.bash`
 
 ### Running the Project

 #### Test Mapping 

 The robot is able to map it's environment using the [ROS slam_gmapping package](http://wiki.ros.org/gmapping). 
 To see this in action, run the following code after cloning the project workspace and building the packages.
 
 Then run the following code:
 
 `cd src/scripts`
 
 `./test_slam.sh`

 You may need to make the test_slam.sh file executable using: 

 `chmod +x test_slam.sh`
 
 Drive the robot around using the teleop terminal which launches automatically. 
 
 #### Test Navigation
 The robot is able to locailze itself with the given pgm map of the environment. This is done using the [ROS amcl package](http://wiki.ros.org/amcl). 
 The initial position parameters were edited in this package to reflect the position of the robot when it is spawned in Gazebo. To see the localization in action, run the following script in the /srcipts folder.
 
 `./test_navigation.sh`
 
 After setting some 2D nav goals using the RVIZ gui, you will see the pose array which represents the robot's estimated pose converge towards the robot. You may need to make the test_navigation.sh file executable using:
 
 `chmod +x test_navigation.sh`
 
 #### Home Service in Action
 The robot is also able to navigate towards goals which are given through a script. This is done using the ROS navigation stack which creates a path for your robot based on Dijkstra's algorithm while avoiding collisions with obsatcles in the environment. The pick_objects C++ node sends two navigation goals to the ROS navigation stack via an `actionlib` client. These navigation goals represent the pickup/dropoff locations for an object and are set based on the object's initial location and desired dropoff location. After the pickup location is reached, the robot waits for 5 seconds to simulate picking up the object then moves to the dropoff location.
 
 The add_markers C++ node controls when the objects are added/deleted from the simulation. This node subscribes to the amcl pose of the robot to keep track of when the robot has reached a prescribed pickup/dropoff location. Once this happens, this node removes an object from the screen when a pickup location is reached or it adds an object to the screen when a dropoff location is reached.
 
 Run the following shell script to use the home service:
 
 `cd src/scripts`
 
 `./home_service`
 
 
 You may need to make the home_service.sh file executable using:
 
 `chmod +x home_service.sh`

 [Home Service Robot Video](https://drive.google.com/file/d/1DEjxUaPUiYFTJSc87Tr5GcejvSPp2mE2/view?usp=sharing)
