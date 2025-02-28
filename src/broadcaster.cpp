#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

ros::NodeHandle *n;
static double RobotPose[2] = {0, 0};
static double Orientation[4] = {0, 0, 0, 0};
ros::Time time_now;

void broadcastTransform() {
  	static tf::TransformBroadcaster br;
	tf::Transform transform;
  	transform.setOrigin(tf::Vector3(RobotPose[0], RobotPose[1], 0));
  	tf::Quaternion q(Orientation[0], Orientation[1], Orientation[2], Orientation[3]);
  	transform.setRotation(q);
  	br.sendTransform(tf::StampedTransform(transform, time_now, "odom", "base_link"));
	transform.setIdentity();
	transform.setRotation(tf::Quaternion(tf::Vector3(1, 0, 0), M_PI));
	br.sendTransform(tf::StampedTransform(transform, time_now, "base_link", "Tiago/Hokuyo_UTM_30LX"));
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
	RobotPose[0] = msg->pose.pose.position.x;
	RobotPose[1] = msg->pose.pose.position.y;
	Orientation[0] = msg->pose.pose.orientation.x;
	Orientation[1] = msg->pose.pose.orientation.y;
	Orientation[2] = msg->pose.pose.orientation.z; 
	Orientation[3] = msg->pose.pose.orientation.w;
	time_now = msg->header.stamp;
	broadcastTransform();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "broadcaster_node", ros::init_options::AnonymousName);
	n = new ros::NodeHandle;
	ros::Subscriber sub_odom;
	sub_odom = n->subscribe("/odom", 1, odom_callback);
	while (ros::ok()) {
		ros::spin();
	}
	return 0;
}
