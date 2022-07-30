# Home Service Robot - Robotics Software Engineer Nanodegree Final Project (Udacity)

### This project builds a home service robot in ROS which delivers objects in a home.

### Workflow:
 1. Accept an iniitial navigation goal
 2. Move towards the goal
 3. Pick up the object
 4. Accept a second navigational goal towards a dropoff point
 5. Dropoff the object
 
 ## Cloning the project workspace
 `git clone https://github.com/Anna-LeeMcLean/home-service-robot`
 
 `catkin make`
 
 `source devel/setup.bash`
 
 ---
 The robot is able to map it's environment using the [ROS slam_gmapping package](http://wiki.ros.org/gmapping). 
 To see this in action, run the following code after cloning the project workspace.
 
 `cd src/scripts`
 
 `./test_slam.sh`
 
 Drive the robot around using the teleop terminal which launches automatically.
 
 ---
 The robot is able to locailze itself with the given pgm map of the environment. This is done using the [ROS amcl package](http://wiki.ros.org/amcl). 
 The initial position parameters were edited in this package to reflect the position of the robot when it is spawned in Gazebo. To see the localization in action,
 run tthe following code after cloning the project workspace.
 
 `cd src/scripts`
 
 `./test_navigation.sh`
 
 After setting some 2D nav goals using the RVIZ gui, you will see the pose array which represents the robot's estimated pose converge towards the robot.
 
 ---
 The robot is also able to navigate towards goals which are given through a script. This is done using the ROS navigation stack which creates a path for your robot 
 based on Dijkstra's algorithm while avoiding collisions with obsatcles in the environment. The pick_objects C++ node sends two navigation goals to the ROS navigation 
 stack via an `actionlib` client. These navigation goals are set based on the object locations in the room. After the first goal is reached, the robot waits for 5 seconds 
 to simulate picking up the object then moves to the second navigation goal.
 
 ---
 The add_markers C++ node controls when the objects are added/deleted from the simulation. This node subscribes to the amcl pose of the robot to keep track of when the
 robot has reached the presecribed marker location. Once this happens, this node removes an object form the screen when a pickup location is reached or it removes an 
 object from the screen when a dropoff location is reached.
 
 To see the home service robot in action, run the following commands after cloning the project workspace:
 
 `cd src/scripts`
 
 `./home_service`
 
 
 
