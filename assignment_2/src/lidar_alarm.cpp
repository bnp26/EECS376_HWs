// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 


const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0;
double angle_max_=0;
double angle_increment_=0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;


float closest_ping_dist = 0.0;
int closest_ping_index = 0;
int lidar_requested_total_range = 128;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        
    }
    
   closest_ping_dist = 4.0;
	
   int current_angle = -1*lidar_requested_total_range/2;
// implementing the algorthm
   for(int x = 0; x < lidar_requested_total_range; x++) {
// get the current value of the requested index.
		if(closest_ping_dist > laser_scan.ranges[ping_index_ + current_angle]) { 
			closest_ping_dist = laser_scan.ranges[ping_index_ + current_angle];
			closest_ping_index = current_angle;
		}	
		current_angle+=1;
   } 
   ROS_INFO("closest ping dist at index %i = %f", closest_ping_index, closest_ping_dist); 
   if (closest_ping_dist < MIN_SAFE_DISTANCE && closest_ping_dist > 0) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
   }
   else {
       laser_alarm_=false;
   }
   
   int ping_index = 0;
   
   if(closest_ping_index > 0){ ping_index = -1; }
   else if(closest_ping_index < 0){ ping_index = 1; }
   else { ping_index = 0; }
   
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Int8 lidar_dist_msg;
   lidar_dist_msg.data = ping_index;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Int8>("lidar_alarm_index", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

