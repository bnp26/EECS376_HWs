//path_client:
// illustrates how to send a request to the path_service service

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_4/moveAction.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

bool g_lidar_alarm=false; // global var for lidar alarm
int g_lidar_alarm_index = 0;

void doneCb(const actionlib::SimpleClientGoalState& state,
	    const assignment_4::moveResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    int diff = result->output - result->goal_stamp;
    ROS_INFO("got result output = %d; goal_stamp = %d; diff = %d", result->output, result->goal_stamp, diff);
}

void alarmCallback(const std_msgs::Bool& alarm_msg) 
{ 
    g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
    if (g_lidar_alarm) {
	ROS_INFO("LIDAR alarm received!"); 
    }
} 

void distanceCallback(const std_msgs::Int8& alarm_index)
{
    g_lidar_alarm_index = alarm_index.data;
    if (g_lidar_alarm) {
	ROS_INFO("alarm came from %i", g_lidar_alarm_index);
    }
}

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "action_client_node");
    int g_count = 0;
    assignment_4::moveGoal goal;
    actionlib::SimpleActionClient<assignment_4::moveAction> action_client("movement_action", true);
    ros::NodeHandle n;
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    ros::ServiceClient client = n.serviceClient<assignment_4::PathSrv>("path_service");
    ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCallback); 
            ros::Subscriber lidar_distance = n.subscribe("lidar_alarm_index",1,distanceCallback);
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    assignment_4::PathSrv path_srv;
   
    while (ros::ok())
    {
	geometry_msgs::PoseStamped pose_stamped;
	geometry_msgs::Pose pose;
	pose.position.x = 0.0; // say desired x-coord is 3
	pose.position.y = 12.0;
	pose.position.z = 0.0; // let's hope so!
	pose.orientation.x = 0.0; //always, for motion in horizontal plane
	pose.orientation.y = 0.0; // ditto
	pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
	pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    }

    while (g_lidar_alarm && ros::ok()) 
    {
		
    }

    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
    pose.position.x = 3.0; // say desired x-coord is 3
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    //go right 3 
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    ROS_INFO("Should have moved to coordinate (%f, %f)", pose.position.x, pose.position.y); 
    //go up 3 
    pose.position.x = 3;
    pose.position.y = 3;
    ROS_INFO("should have moved to coordinate (%f, %f)", pose.position.x, pose.position.y);
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    //go right 4
    pose.position.x = 7;
    pose.position.y = 3;
    ROS_INFO("should have moved to coordinate (%f, %f)", pose.position.x, pose.position.y);
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    //go up 2
    pose.position.x = 7;
    pose.position.y = 5.25;
    ROS_INFO("should have moved to coordinate (%f, %f)", pose.position.x, pose.position.y);
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    //go left 4
    pose.position.x = 3;
    pose.position.y = 5.25;
    ROS_INFO("should have moved to coordinate (%f, %f)", pose.position.x, pose.position.y);
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    //go up 7
    pose.position.x = 3;
    pose.position.y = 12;
    ROS_INFO("should have moved to coordinate (%f, %f)", pose.position.x, pose.position.y);
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    //go left 3
    pose.position.x = 0;
    pose.position.y = 12;
    ROS_INFO("should have moved to coordinate (%f, %f)", pose.position.x, pose.position.y);
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    // some more poses...
    // quat = convertPlanarPhi2Quaternion(1.57); // get a quaternion corresponding to this heading
    // pose_stamped.pose.orientation = quat;   
    // pose_stamped.pose.position.y=3.0; // say desired y-coord is 1.0
    // path_srv.request.nav_path.poses.push_back(pose_stamped);
    // quat = convertPlanarPhi2Quaternion(3.14);
    client.call(path_srv);

    return 0;
}
