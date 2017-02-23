//path_service:
// example showing how to receive a nav_msgs/Path request
// run with complementary path_client
// responds immediately to ack new path...but execution takes longer

// this is a crude service; just assumes robot initial pose is 0,
// and all subgoals are expressed with respect to this initial frame.
// i.e., equivalent to expressing subgoals in odom frame

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment_4/moveAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>

#define PI 3.141592653589

using namespace std;

//some tunable constants, global
const double g_move_speed = 0.5; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 0.3; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.005;
const double g_dist_tol = 0.01; // 1cm
//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 

geometry_msgs::Pose goal_pose;
geometry_msgs::Pose moving_pose;

//here are a few useful utility functions:
double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);
void g_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading);

geometry_msgs::Pose convertArrays2Pose(const std::vector<double> position, const std::vector<double> orientation);

double * getPositionArrayFromPose(geometry_msgs::Pose given_pose);
double * getOrientationArrayFromPose(geometry_msgs::Pose given_pose);

class MoveActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in assignment_4/action/move.action
    // the type "moveAction" is auto-generated from our name "move" and generic name "Action"
   	actionlib::SimpleActionServer<assignment_4::moveAction> as_;

    // here are some message types to communicate with our client(s)
    assignment_4::moveGoal goal_; // goal message, received from client
    assignment_4::moveResult result_; // put results here, to be sent back to the client when done w/ goal
    assignment_4::moveFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
public:
    MoveActionServer(); //define the body of the constructor outside of class definition

    ~MoveActionServer(void) {
    }
    // Action Interface
	void executeCB(const actionlib::SimpleActionServer<assignment_4::moveAction>::GoalConstPtr& goal);
};



MoveActionServer::MoveActionServer() :
   as_(nh_, "move_action", boost::bind(&MoveActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of moveActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// MoveActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <assignment_4::moveAction> customizes the simple action server to use our own "action" message 
// defined in our package, "assignment_4", in the subdirectory "action", called "move.action"
// The name "move" is prepended to other message types created automatically during compilation.
// e.g.,  "moveAction" is auto-generated from (our) base name "move" and generic name "Action"
void MoveActionServer::executeCB(const actionlib::SimpleActionServer<assignment_4::moveAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
	
	goal_pose = convertArrays2Pose(goal->position_input, goal->orientation_input);
	moving_pose = convertArrays2Pose(goal->position_input, goal->orientation_input);

	ROS_INFO("goal position is: (%f, %f, %f)", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
	ROS_INFO("goal orientation is: %f", convertPlanarQuat2Phi(goal_pose.orientation));
    //do work here: this is where your interesting code goes
	
    while (goal->num_goals > 0) {
       ROS_INFO("number of goals = %d", goal->num_goals);

       // each iteration, check if cancellation has been ordered
       if (as_.isPreemptRequested()){
          ROS_WARN("goal cancelled!");

		  result_.position_output = getPositionArrayFromPose(moving_pose);
		  result_.orientation_output = getOrientationArrayFromPose(moving_pose);

          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
        }

       //if here, then goal is still valid; provide some feedback
	   
	   	feedback_.position_output = getPositionArrayFromPose(moving_pose);
	   	feedback_.orientation_output = getOrientationArrayFromPose(moving_pose);
		
		as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
		
		double yaw_desired, yaw_current, distance, spin_angle;

		g_and_dist(moving_pose, goal_pose, distance, yaw_desired);

		ROS_INFO("pose %d: desired yaw = %f", goal->num_goals, yaw_desired);        
        yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation); //our current yaw--should use a sensor
        spin_angle = yaw_desired - yaw_current; // spin this much
        spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
        do_spin(spin_angle); // carry out this incremental action
        // we will just assume that this action was successful--really should have sensor feedback here
	    pose_desired.orientation = convertPlanarPhi2Quaternion(yaw_desired);
    	ROS_INFO("pose_desired.orientation = %f", yaw_desired);  
        moving_pose.orientation = pose_desired.orientation; // assumes got to desired orientation precisely
        moving_pose.position = pose_desired.position;

		//timer.sleep(); //wait 1 sec between loop iterations of this timer
    }
    //if we survive to here, then the goal was successfully accomplished; inform the client
    result_.position_output[0] = moving_pose.position.x; //value should be zero, if completed countdown
    result_.position_output[1] = moving_pose.position.y; //value should be zero, if completed countdown
    result_.position_output[2] = moving_pose.position.z; //value should be zero, if completed countdown

    result_.orientation_output[0] = moving_pose.orientation.x; //value should be zero, if completed countdown
    result_.orientation_output[1] = moving_pose.orientation.y; //value should be zero, if completed countdown
    result_.orientation_output[2] = moving_pose.orientation.z; //value should be zero, if completed countdown
    result_.orientation_output[3] = moving_pose.orientation.w; //value should be zero, if completed countdown
    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}


//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
    ROS_INFO("sping_angle = %f", spin_angle);
    if (spin_angle>PI) {
	spin_angle -= PI*2;}
    if (spin_angle<-PI) {
	spin_angle += PI*2;}
     return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<5;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}
//converts two given arrays to a Pose
geometry_msgs::Pose convertArrays2Pose(const std::vector<double> position, const std::vector<double> orientation) {
	geometry_msgs::Pose new_pose;

	new_pose.position.x = position[0];
	new_pose.position.y = position[1];
	new_pose.position.z = position[2];

	new_pose.orientation.x = orientation[0];
	new_pose.orientation.y = orientation[1];
	new_pose.orientation.z = orientation[2];
	new_pose.orientation.w = orientation[3];

	return new_pose;
}

//gets an array of doubles equivilent to a given Pose's position
double * getPositionArrayFromPose(geometry_msgs::Pose given_pose) {
	double * position_array;

	position_array[0] = given_pose.position.x;
	position_array[1] = given_pose.position.y;
	position_array[2] = given_pose.position.z;

	return position_array;
}

//gets an array of doubles equivilent to a given Pose's orientation
double * getOrientationArrayFromPose(geometry_msgs::Pose given_pose) {
	double* orientation_array;

	orientation_array[0] = given_pose.orientation.x;
	orientation_array[1] = given_pose.orientation.y;
	orientation_array[2] = given_pose.orientation.z;
	orientation_array[3] = given_pose.orientation.w;

	return orientation_array;
}
//THIS FUNCTION IS NOT FILLED IN: NEED TO COMPUTE HEADING AND TRAVEL DISTANCE TO MOVE
//FROM START TO GOAL
void g_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
    
    dist = 0.0; //FALSE!!

    //current_position -> cp
    //cp_x -> current position x
    //goal position -> gp
    //gp_x -> gp_x

    double cp_x = current_pose.position.x; 
    double cp_y = current_pose.position.y; 

    double gp_x = goal_pose.position.x;
    double gp_y = goal_pose.position.y;
    
    double dist_x = gp_x - cp_x;
    double dist_y = gp_y - cp_y;
    ROS_INFO("dist_x = %f, dist_y = %f", dist_x, dist_y);
    dist = (dist_x*2 + dist_y*2) / 2;
    heading = atan2(dist_y, dist_x);
    ROS_INFO("heading = %f", heading);
    if(dist < 0) {
		dist *= -1;
    }
}

/*
bool callback(assignment_4::PathSrvRequest& request, assignment_4::PathSrvResponse& response)
{
    ROS_INFO("callback activated");
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    geometry_msgs::Pose pose_desired;
    int npts = request.nav_path.poses.size();
    ROS_INFO("received path request with %d poses",npts);    
    
    for (int i=0;i<npts;i++) { //visit each subgoal
        // odd notation: drill down, access vector element, drill some more to get pose
        pose_desired = request.nav_path.poses[i].pose; //get next pose from vector of poses
        
        //WRITE THIS FNC: compute desired heading and travel distance based on current and desired poses
	g_and_dist(g_current_pose, pose_desired,travel_distance, yaw_desired);
        ROS_INFO("pose %d: desired yaw = %f; desired (x,y) = (%f,%f)",i,yaw_desired, pose_desired.position.x,pose_desired.position.y); 
        ROS_INFO("current (x,y) = (%f, %f)",g_current_pose.position.x,g_current_pose.position.y);
        ROS_INFO("travel distance = %f",travel_distance);         
         
        // a quaternion is overkill for navigation in a plane; really only need a heading angle
        // this yaw is measured CCW from x-axis
        // GET RID OF NEXT LINE AFTER FIXING g_and_dist()
        
        ROS_INFO("pose %d: desired yaw = %f",i,yaw_desired);        
        yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation); //our current yaw--should use a sensor
        spin_angle = yaw_desired - yaw_current; // spin this much
        spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
        do_spin(spin_angle); // carry out this incremental action
        // we will just assume that this action was successful--really should have sensor feedback here
	pose_desired.orientation = convertPlanarPhi2Quaternion(yaw_desired);
	ROS_INFO("pose_desired.orientation = %f", yaw_desired);  
        g_current_pose.orientation = pose_desired.orientation; // assumes got to desired orientation precisely
       	g_current_pose.position = pose_desired.position;
        //FIX THE NEXT LINE, BASED ON g_and_dist()
        do_move(travel_distance);  // move forward 1m...just for illustration; SHOULD compute this from subgoal pose
    }

  return true;
}

void do_inits(ros::NodeHandle &n) {
  //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
    // we declared g_twist_commander as global, but never set it up; do that now that we have a node handle
    g_twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);    
}
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "timer_action_server_node"); // name this node 

    ROS_INFO("instantiating the timer_action_server: ");

    MoveActionServer as_object; // create an instance of the class "ExampleActionServer"

    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}
