#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <vector>

double robot_x;
double robot_y;
double robot_w;

bool at_marker(double bot_x, double bot_y, double marker_x, double marker_y)
{
  bool atgoal = false;
  double distance_threshold = 0.3;
  double x_diff = marker_x - bot_x;
  double y_diff = marker_y - bot_y;
  double diff = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
  //ROS_INFO("difference: %f", diff);
  if (diff < distance_threshold)
    atgoal = true;

  return atgoal;
}

void robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg1)
{
  
  robot_x = msg1->pose.pose.position.x;
  robot_y = msg1->pose.pose.position.y;
  robot_w = msg1->pose.pose.orientation.w;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub1 = n.subscribe("/amcl_pose", 10, robot_pose_callback);

  

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the dropoff locations
  double marker_x1 = 1.0;
  double marker_y1 = 7.0;
  double marker_w1 = 0.4;
  n.setParam("marker_x1", marker_x1);
  n.setParam("marker_y1", marker_y1);
  n.setParam("marker_w1", marker_w1);

  // Set the pickup locations
  double marker_x2 = -4.5;
  double marker_y2 = -4.5;
  double marker_w2 = 0.4;
  n.setParam("marker_x2", marker_x2);
  n.setParam("marker_y2", marker_y2);
  n.setParam("marker_w2", marker_w2);

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    sleep(5);
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = marker_x1;
    marker.pose.position.y = marker_y1;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = marker_w1;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    //ROS_INFO("robotx: %f, roboty: %f", robot_x, robot_y);

    bool picked_up;
    bool at_the_goal1 = at_marker(robot_x, robot_y, marker_x1, marker_y1);
    bool at_the_goal2 = at_marker(robot_x, robot_y, marker_x2, marker_y2);
    if (at_the_goal1)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
      ROS_INFO("Marker picked up");
      sleep(5);
      picked_up = true;
    }
    if (at_the_goal2 && picked_up)
    {
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = marker_x2;
      marker.pose.position.y = marker_y2;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = marker_w2;
      marker_pub.publish(marker);
      ROS_INFO("Marker dropped off");
    }
    if (!picked_up)
    {
      marker_pub.publish(marker);
    }
   
    ros::spinOnce();
    r.sleep();
  }

}
