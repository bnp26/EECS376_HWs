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
#include <std_msgs/Bool.h> // boolean message
#include <std_msgs/Int8.h>
using namespace std;

bool g_lidar_alarm=false; // global var for lidar alarm
int g_lidar_alarm_index = 0;
bool g_goal_active = false;

geometry_msgs::Pose current_pose, starting_pose;

void alarmCallback(const std_msgs::Bool& alarm_msg) 
{ 
    g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
    if (g_lidar_alarm) {
		ROS_INFO("LIDAR alarm received!"); 
    }
}//h 

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

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

void doneCb(const actionlib::SimpleClientGoalState& state,
			const assignment_4::moveResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
	geometry_msgs::Pose pose_result = result->pose_output;

	g_goal_active = false;
    ROS_INFO("RESULTS: ");
	ROS_INFO("goal position = (%f, %f, %f)", pose_result.position.x, pose_result.position.y, pose_result.position.z);
	ROS_INFO("goal heading = %f", convertPlanarQuat2Phi(pose_result.orientation));
}

void feedbackCb(const assignment_4::moveFeedbackConstPtr& feedback){
	ROS_INFO("inside feedbackCB");
	
	geometry_msgs::Pose current_pose;
	current_pose = feedback->pose_fdbk;

	/*current_pose.position.x = feedback->position_fdbk[0];
	current_pose.position.y = feedback->position_fdbk[1];
	current_pose.position.z = feedback->position_fdbk[2];

   	current_pose.orientation.x = feedback->orientation_fdbk[0];
   	current_pose.orientation.y = feedback->orientation_fdbk[1];
   	current_pose.orientation.z = feedback->orientation_fdbk[2];
   	current_pose.orientation.w = feedback->orientation_fdbk[3];
*/
    ROS_INFO("RESULTS: ");
	
	ROS_INFO("current position = (%f, %f, %f)", current_pose.position.x, current_pose.position.y, current_pose.position.z);
	ROS_INFO("current heading = %f", convertPlanarQuat2Phi(current_pose.orientation));
}

void activeCb() {
	ROS_INFO("Goal just went active");
	g_goal_active = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "action_client_node");
    int g_count = 0;
	
    ros::NodeHandle n;

    assignment_4::moveGoal goal;
    actionlib::SimpleActionClient<assignment_4::moveAction> action_client("move_action", true);

    ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCallback); 
    ros::Subscriber lidar_distance = n.subscribe("lidar_alarm_index",1,distanceCallback);
    geometry_msgs::Quaternion quat;

    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds

	if(!server_exists) {
		ROS_WARN("could not connect to server; halting");
		return 0; // bail out; optionally, could print a warning message and retry
	}

    ROS_INFO("connected to action server");

	while (true) {
        // stuff a goal message:
        g_count++;
        goal.num_goals = 1; // this merely sequentially numbers the goals sent
	geometry_msgs::PoseStamped pose_stamped;
    	geometry_msgs::Pose pose;
    	pose.position.x = 0.0; // say desired x-coord is 3
    	pose.position.y = 0.0;
    	pose.position.z = 12.0; // let's hope so!
    	pose.orientation.x = 0.0; //always, for motion in horizontal plane
    	pose.orientation.y = 0.0; // ditto
    	pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    	pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    	//go right 3 
    	pose_stamped.pose = pose;
    	goal.input_path.poses.push_back(pose_stamped);
        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        //action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

        bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d", g_count);
            return 0;
        } else {
            //if here, then server returned a result to us
        }

    }
    while (g_lidar_alarm && ros::ok()) 
    {
    	ROS_WARN("ALARM HAS GONE OFF!");
	}


    return 0;
}
