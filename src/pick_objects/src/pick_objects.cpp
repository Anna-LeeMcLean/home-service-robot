#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  // Initialize the marker_signal publisher node
  ros::NodeHandle n;
  //ros::Publisher pub1 = n.advertise<std_msgs::Float32MultiArray>("goal_locations", 10);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //Create two goals 
  move_base_msgs::MoveBaseGoal goal1;
  move_base_msgs::MoveBaseGoal goal2;

  // set up the frame parameters for 1st goal
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();

  // Define the first position and orientation for the robot to reach
  double goal_x1;
  double goal_y1;
  double goal_w1;
  n.getParam("marker_x1", goal_x1);
  n.getParam("marker_y1", goal_y1);
  n.getParam("marker_w1", goal_w1);
  goal1.target_pose.pose.position.x = goal_x1;
  goal1.target_pose.pose.position.y = goal_y1;
  goal1.target_pose.pose.orientation.w = goal_w1;

  // Wait for 5 seconds until the marker is placed on the screen
  ros::Duration(5).sleep();

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending first goal");
  ac.sendGoal(goal1);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached the first goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the base moved to the first goal");
    ROS_INFO("Picking up marker");
  }
  else
    ROS_INFO("The base failed to move to the first goal for some reason");


  // Wait for 5 seconds
  ros::Duration(5).sleep();

  // set up the frame parameters for the 2nd goal
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();

  // Define the second position and orientation for the robot to reach
  double goal_x2;
  double goal_y2;
  double goal_w2;
  n.getParam("marker_x2", goal_x2);
  n.getParam("marker_y2", goal_y2);
  n.getParam("marker_w2", goal_w2);
  goal2.target_pose.pose.position.x = goal_x2;
  goal2.target_pose.pose.position.y = goal_y2;
  goal2.target_pose.pose.orientation.w = goal_w2;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending second goal");
  ac.sendGoal(goal2);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached the first goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the base moved to the second goal");
    ROS_INFO("Dropping off marker");
  }
  else
    ROS_INFO("The base failed to move to the second goal for some reason");

  // Wait for 10 seconds
  ros::Duration(10).sleep();
  return 0;
}
