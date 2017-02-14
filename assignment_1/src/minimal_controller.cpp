//Author: Benjamin Poreh
//case ID: bnp26
//email: bnp26@case.edu

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//node to send Twist commands to the Simple 2-Dimensional Robot Simulator via cmd_vel
int main(int argc, char **argv) {
    ros::init(argc, argv, "stdr_commander");
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    //some "magic numbers"
    //extra comment for compilation
	double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 1.0; // 1m/s speed command
    double turn_left = 1.57; //0.5 rad/sec yaw rate command
    double turn_right = -1.57;
	double move_one_box = 1.0; // should move 1 meter or 0.5 rad in 1 seconds
	double move_three_boxes = 3.0;// should move 3 meter or 1.5 rad in 1 seconds
	double move_four_boxes = 4.0;// should move 5 meter or 2.5 rad in 1 seconds
	double twist_90 = 1;

    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x=0.0;
    twist_cmd.linear.y=0.0;
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0;

    ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate     
    double timer=0.0;
    //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
    	twist_commander.publish(twist_cmd);
    	loop_timer.sleep();
    }
    twist_cmd.linear.x=speed; //command to move forward
    while(timer<move_three_boxes) {
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
        loop_timer.sleep();
    }
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=turn_left; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<twist_90) {
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
        loop_timer.sleep();
    } 
    twist_cmd.angular.z=0.0; //and stop spinning in place 
    twist_cmd.linear.x=speed; //and move forward again
    timer=0.0; //reset the timer
    while(timer<move_three_boxes) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
	}	
    	//halt the motion
    	twist_cmd.angular.z=0.0; 
    	twist_cmd.linear.x=0.0; 
    	timer=0.0; //reset the timer
    	//turn 90 deg. to the right
	twist_cmd.angular.z=turn_right;
	while(timer<twist_90) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
    } 
	//stop the turn
	twist_cmd.angular.z=0.0;
	twist_cmd.linear.x=speed;
    timer=0.0; //reset the timer	
	while(timer<move_four_boxes) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
    } 
	//turning left 90 deg.
	twist_cmd.linear.x=0;
	twist_cmd.angular.z=turn_left;
	timer=0.0;	
	while(timer<twist_90) {
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
    	loop_timer.sleep();
    }
	//moving foward 2 blocks
	twist_cmd.linear.x=speed;
	twist_cmd.angular.z=0.0;
	timer=0.0;	
	while(timer<move_one_box*2) {
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
    	loop_timer.sleep();
    }
	//turn left again 90 deg.
	twist_cmd.linear.x=0;
	twist_cmd.angular.z=turn_left;
	timer=0.0;	
	while(timer<twist_90) {
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
    	loop_timer.sleep();
    }
	//move forward 4 boxes
	twist_cmd.linear.x=speed;
	twist_cmd.angular.z=0.0;
	timer=0.0;
	while(timer<move_four_boxes) {
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
        loop_timer.sleep();
    }
	//turn right 90 deg.
	twist_cmd.linear.x=0.0;
	twist_cmd.angular.z=turn_right;
	timer=0.0;
	while(timer<twist_90) {
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
    	loop_timer.sleep();
    }
	// move forward 7 boxes
	twist_cmd.linear.x=speed;
	twist_cmd.angular.z=0.0;
	timer=0.0;
	while(timer<move_three_boxes + move_four_boxes) {
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
        loop_timer.sleep();
    }
   	//turn left 90 deg.	
	twist_cmd.angular.z=turn_left;
	twist_cmd.linear.x=0.0;	
	timer=0.0;
	while(timer<twist_90) {
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
        loop_timer.sleep();
    }
   	//move forward 4 boxes.	
	twist_cmd.angular.z=0.0;
	twist_cmd.linear.x=speed;	
	timer=0.0;
	while(timer<move_four_boxes) {
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
        loop_timer.sleep();
    } 
	//stop motion
	twist_cmd.angular.z=0.0;
	twist_cmd.linear.x=0.0;
	//finish up
	for (int i=0;i<10;i++) {
    	twist_commander.publish(twist_cmd);
    	loop_timer.sleep();
    }    
    //done commanding the robot; node runs to completion
}


