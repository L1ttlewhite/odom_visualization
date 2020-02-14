#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <string.h>
#include <vector>

#include "CameraPoseVisualization.h"

using namespace std;
using namespace Eigen;

CameraPoseVisualization camerapose(0, 1, 0, 1);

vector<ros::Publisher> pub_path(2), pub_camera_pose(2);
vector<ros::Subscriber> sub_pose(2),sub_odom(2);
vector<nav_msgs::Path> path(2);
vector<string> pose_topic(2), odom_topic(2);


void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = msg->pose;
	path[i].header = msg->header;
    path[i].header.frame_id = "world";
    path[i].poses.push_back(pose_stamped);
    pub_path[i].publish(path[i]);
    Vector3d P;
    Quaterniond R;
    P.x() = msg->pose.position.x;
    P.y() = msg->pose.position.y;
    P.z() = msg->pose.position.z;
    R.x() = msg->pose.orientation.x;
    R.y() = msg->pose.orientation.y;
    R.z() = msg->pose.orientation.z;
    R.w() = msg->pose.orientation.w;
    camerapose.reset();
    camerapose.add_pose(P, R);
    if (i==1)
    {
        camerapose.setImageBoundaryColor(0,1,0,1);
        camerapose.setOpticalCenterConnectorColor(0,1,0,1);
    }
    else if (i==0)
    {
        camerapose.setImageBoundaryColor(1,0,0,1);
        camerapose.setOpticalCenterConnectorColor(1,0,0,1);
    }
    //"camera_pose_visual"
    camerapose.publish_by(pub_camera_pose[i], pose_stamped.header);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg, int i)
{
    ROS_INFO("Receive odom msg");
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = msg->pose.pose;
	path[i].header = msg->header;
    path[i].header.frame_id = "world";
    path[i].poses.push_back(pose_stamped);
    pub_path[i].publish(path[i]);
    Vector3d P;
    Quaterniond R;
    P.x() = msg->pose.pose.position.x;
    P.y() = msg->pose.pose.position.y;
    P.z() = msg->pose.pose.position.z;
    R.x() = msg->pose.pose.orientation.x;
    R.y() = msg->pose.pose.orientation.y;
    R.z() = msg->pose.pose.orientation.z;
    R.w() = msg->pose.pose.orientation.w;
    cout << R.x() << R.y() << R.z() << R.w()<< endl;
    camerapose.reset();
    camerapose.add_pose(P, R);
    if (i==1)
    {
        camerapose.setImageBoundaryColor(0,1,0,1);
        camerapose.setOpticalCenterConnectorColor(0,1,0,1);
    }
    else if (i==0)
    {
        camerapose.setImageBoundaryColor(1,0,0,1);
        camerapose.setOpticalCenterConnectorColor(1,0,0,1);
    }
    //"camera_pose_visual"
    camerapose.publish_by(pub_camera_pose[i], pose_stamped.header);
}

int main (int argc, char **argv)
{

	ros::init (argc, argv, "odom_visualization");
	ros::NodeHandle n;
    
    camerapose.setScale(0.1);
    camerapose.setLineWidth(0.01);
    int num_pose, num_odom;
    n.getParam("num_pose", num_pose);
    n.getParam("num_odom", num_odom);
    for (int i=0; i<max(num_odom, num_pose); i++)
    {
        pub_path[i] = n.advertise<nav_msgs::Path>("trajectory_"+to_string(i),100,true);
        pub_camera_pose[i] = n.advertise<visualization_msgs::MarkerArray>("camera_pose_"+to_string(i),100);
        ROS_INFO("Publish the trajectory of odom on topic: %s", ("trajectory_"+to_string(i)).c_str());
        ROS_INFO("Publish the camera_pose on topic: %s", ("camera_pose_"+to_string(i)).c_str());
    }
    if (num_pose<=2 && num_pose>=0){
        for (int i=0; i<num_pose; i++)
        {
            n.getParam("pose_topic_"+to_string(i+1), pose_topic[i]);
            sub_pose[i] = n.subscribe<geometry_msgs::PoseStamped>(pose_topic[i], 1000, boost::bind(&pose_callback,_1,i));
            ROS_INFO("Subscribe to the pose_topic: %s", pose_topic[i].c_str());
        }
    }
    else{
        ROS_WARN("Please edit the right num of the poses and odometries ( between 2 and 0, sum < 2 ).");
        return 0;
    }
    if (num_odom<=2-num_pose && num_odom>=0){
        for (int i=0; i<num_odom; i++)
        {
            n.getParam("odom_topic_"+to_string(i+1), odom_topic[i]);
            sub_odom[i] = n.subscribe<nav_msgs::Odometry>(odom_topic[i], 1000, boost::bind(&odom_callback,_1,i));
            ROS_INFO("Subscribe to the odom_topic: %s", odom_topic[i].c_str());
        }
    }
    else{
        ROS_WARN("Please edit the right num of the poses and odometries ( between 2 and 0, sum < 2 ).");
        return 0;
    }

    ros::spin();
	return 0;
}