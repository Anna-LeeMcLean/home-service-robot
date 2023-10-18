#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv){
	
	ros::init(argc, argv, "set_robot_initial_position");
	ros::NodeHandle nh;
	ros::Rate r(1);
	
	ros::Publisher set_position_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100);
	
	while (ros::ok()){
	
		geometry_msgs::PoseWithCovarianceStamped init_position;
		init_position.header.frame_id = "map";
		init_position.header.stamp = ros::Time::now();
		init_position.pose.pose.position.x = -2.0;
		init_position.pose.pose.position.y = -0.5;
		init_position.pose.pose.position.z = 0.0;
		init_position.pose.pose.orientation.x = 0.0;
		init_position.pose.pose.orientation.y = 0.0;
		init_position.pose.pose.orientation.z = 0.0;
		init_position.pose.pose.orientation.w = 1.0;
		
	
		set_position_publisher.publish(init_position);
		ROS_INFO("Publishing initial position for turtlebot...");
		ros::spinOnce();
		r.sleep();
		
	}
	
}
