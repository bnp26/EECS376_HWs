//odom_tf_fncs.cpp:
//wsn, March 2016
//implementation of member functions of OdomTf class

#include "assignment_8/odom_tf.h"
using namespace std;

//constructor: don't need nodehandle here, but could be handy if want to add a subscriber

OdomTf::OdomTf(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { // constructor
    ROS_INFO("in class constructor of DemoTfListener");
    tfListener_ = new tf::TransformListener; //create a transform listener
    odom_tf_ready_ = false;
    odom_ready_ = false;
    amcl_ready_ = false;
    // wait to start receiving valid tf transforms between odom and link2:
    bool tferr = true;
    pose_publisher_= nh_.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    
    ROS_INFO("waiting for tf between base_link and odom...");
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform from target frame "odom" to source frame "link2"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. 
            //See tf/CoordinateFrameConventions#Transform_Direction
            tfListener_->lookupTransform("odom", "base_link", ros::Time(0), stfBaseLinkWrtOdom_);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("odom to base_link tf is good");

    //now check for xform base_link w/rt map; would need amcl, or equivalent running
    tferr = true;
    ROS_INFO("waiting for tf between odom and map...");
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform from target frame "odom" to source frame "link2"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. 
            //See tf/CoordinateFrameConventions#Transform_Direction
            tfListener_->lookupTransform("map", "odom", ros::Time(0), stfOdomWrtMap_);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("map to odom tf is good");
    // from now on, tfListener will keep track of transforms
    //initialize an identity transform, declaring base-link and map frames are coincident:
    stfAmclBaseLinkWrtMap_.setIdentity();
    tf::Quaternion quat(0, 0, 0, 1);
    stfAmclBaseLinkWrtMap_.setRotation(quat);	//all rotations 0
    stfAmclBaseLinkWrtMap_.frame_id_ = "map";
    stfAmclBaseLinkWrtMap_.child_frame_id_ = "base_link";
    stfAmclBaseLinkWrtMap_.stamp_ = ros::Time::now();           

    cout<<endl<<"init stfAmclBaseLinkWrtMap_"<<endl;
    printStampedTf(stfAmclBaseLinkWrtMap_);    
    
    //initialize an odometry frame coincident with map and base-link
    stfDriftyOdomWrtMap_ = stfAmclBaseLinkWrtMap_;
    stfDriftyOdomWrtMap_.frame_id_ = "map"; // declare the respective frames
    stfDriftyOdomWrtMap_.child_frame_id_ = "drifty_odom"; 

    odom_subscriber_ = nh_.subscribe("/drifty_odom", 1, &OdomTf::odomCallback, this); //subscribe to odom messages

    //wait for drifty odom publication:
    ROS_WARN("waiting for drifty_odom publication");
    odom_count_=0;
    odom_phi_ = 1000.0; // put in impossible value for heading; test this value to make sure we have received a viable odom message
    ROS_INFO("waiting for valid odom message...");
    while (odom_phi_ > 500.0) {
        ros::Duration(0.5).sleep(); // sleep for half a second
        std::cout << ".";
        ros::spinOnce();
    }
    ROS_INFO("stfBaseLinkWrtDriftyOdom_");
    printStampedTf(stfBaseLinkWrtDriftyOdom_);
    amcl_subscriber_ = nh_.subscribe("/amcl_pose", 1, &OdomTf::amclCallback, this); //subscribe to odom messages
    ROS_WARN("waiting for amcl publication...");
    while (!amcl_ready_) {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }        
    ROS_INFO("got amcl callback; stfAmclBaseLinkWrtMap_:");
    printStampedTf(stfAmclBaseLinkWrtMap_);

    ROS_INFO("constructor: got an odom message; ready to roll");
    odom_tf_ready_=true;
}

void OdomTf::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    odom_subscriber_ = nh_.subscribe("/drifty_odom", 1, &OdomTf::odomCallback, this); //subscribe to odom messages
    amcl_subscriber_ = nh_.subscribe("/amcl_pose", 1, &OdomTf::amclCallback, this); //subscribe to odom messages

    // add more subscribers here, as needed
}

double OdomTf::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//borrowing from Andrew
void OdomTf::printTf(tf::Transform tf) {
    tf::Vector3 tfVec;
    tf::Matrix3x3 tfR;
    tf::Quaternion quat;
    tfVec = tf.getOrigin();
    cout << "vector from reference frame to child frame: " << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl;
    tfR = tf.getBasis();
    cout << "orientation of child frame w/rt reference frame: " << endl;
    tfVec = tfR.getRow(0);
    cout << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl;
    tfVec = tfR.getRow(1);
    cout << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl;
    tfVec = tfR.getRow(2);
    cout << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl;
    quat = tf.getRotation();
    cout << "quaternion: " << quat.x() << ", " << quat.y() << ", "
            << quat.z() << ", " << quat.w() << endl;
}

//borrowing from Andrew
void OdomTf::printStampedTf(tf::StampedTransform sTf) {
    tf::Transform tf;
    cout << "frame_id: " << sTf.frame_id_ << endl;
    cout << "child_frame_id: " << sTf.child_frame_id_ << endl;
    tf = get_tf_from_stamped_tf(sTf); //extract the tf from the stamped tf  
    printTf(tf); //and print its components      
}
//borrowing from Andrew
void OdomTf::printStampedPose(geometry_msgs::PoseStamped stPose) {
    cout << "frame id = " << stPose.header.frame_id << endl;
    cout << "origin: " << stPose.pose.position.x << ", " << stPose.pose.position.y << ", " << stPose.pose.position.z << endl;
    cout << "quaternion: " << stPose.pose.orientation.x << ", " << stPose.pose.orientation.y << ", "
            << stPose.pose.orientation.z << ", " << stPose.pose.orientation.w << endl;
}

bool OdomTf::multiply_stamped_tfs(tf::StampedTransform A_stf,
        tf::StampedTransform B_stf, tf::StampedTransform &C_stf) {

    //long-winded approach:
    //std::string str1(A_stf.child_frame_id_); //want to compare strings to check consistency
    //std::string str2(B_stf.frame_id_);
    //if (str1.compare(str2) != 0) { //SHOULD get that child frame of A is parent frame of B
    //more compact approach:
    if (A_stf.child_frame_id_.compare(B_stf.frame_id_) != 0) {    
        std::cout << "can't multiply transforms; mismatched frames" << endl;
	std::cout << A_stf.child_frame_id_ << " is not " << B_stf.frame_id_ << '\n';
        return false;
    }
    //if here, the named frames are logically consistent
        tf::Transform C = A_stf*B_stf; //multiplication is defined for transforms
	C_stf.setData(C);
	C_stf.frame_id_ = A_stf.frame_id_;
	C_stf.child_frame_id_ = B_stf.child_frame_id_;
	C_stf.stamp_ = ros::Time::now();
 
    //long-winded approach, equivalent to above:
    /*
    tf::Transform A, B; //simple transforms--not stamped
        
    A = get_tf_from_stamped_tf(A_stf); // get the transform from the stamped transform
    B = get_tf_from_stamped_tf(B_stf);
    C = A*B; //multiplication is defined for transforms 
    C_stf.frame_id_ = A_stf.frame_id_; //assign appropriate parent and child frames to result
    C_stf.child_frame_id_ = B_stf.child_frame_id_;
    C_stf.setOrigin(C.getOrigin()); //populate the origin and orientation of the result
    C_stf.setBasis(C.getBasis());
    C_stf.stamp_ = ros::Time::now(); //assign the time stamp to current time; 
     * */
    // alternatively, could assign this to the OLDER of A or B transforms
    return true; //if got here, the multiplication is valid
}

tf::StampedTransform OdomTf::stamped_transform_inverse(tf::StampedTransform stf) {
    // instantiate stamped transform with constructor args
    //note: child_frame and frame_id are reversed, to correspond to inverted transform
    tf::StampedTransform stf_inv(stf.inverse(), stf.stamp_, stf.child_frame_id_, stf.frame_id_);
    /* long-winded equivalent:
    tf::StampedTransform stf_inv;    
    tf::Transform tf = get_tf_from_stamped_tf(stf);
    tf::Transform tf_inv = tf.inverse();
    
    stf_inv.stamp_ = stf.stamp_;
    stf_inv.frame_id_ = stf.child_frame_id_;
    stf_inv.child_frame_id_ = stf.frame_id_;
    stf_inv.setOrigin(tf_inv.getOrigin());
    stf_inv.setBasis(tf_inv.getBasis());
     * */
    return stf_inv;
}

tf::Transform OdomTf::get_tf_from_stamped_tf(tf::StampedTransform sTf) {
    tf::Transform tf(sTf.getBasis(), sTf.getOrigin()); //construct a transform using elements of sTf
    return tf;
}

//callback fnc based on drifty_odom
void OdomTf::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    odom_count_++;
    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    current_odom_ = odom_rcvd; // save the entire message
    // but also pick apart pieces, for ease of use
    odom_pose_ = odom_rcvd.pose.pose;
    //SHOULD transform these velocity values to map frame as well
    //odom_vel_ = odom_rcvd.twist.twist.linear.x;
    //odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    odom_quat_ = odom_rcvd.pose.pose.orientation;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    odom_phi_ = convertPlanarQuat2Phi(odom_quat_); // cheap conversion from quaternion to heading for planar motion
    //get the position from odom, converted to a tf type
    tf::Vector3 pos;
    pos.setX(odom_pose_.position.x);
    pos.setY(odom_pose_.position.y);
    pos.setZ(odom_pose_.position.z);
    stfBaseLinkWrtDriftyOdom_.stamp_ = ros::Time::now();
    stfBaseLinkWrtDriftyOdom_.setOrigin(pos);
    // also need to get the orientation of odom_pose and use setRotation
    tf::Quaternion q;
    q.setX(odom_quat_.x);
    q.setY(odom_quat_.y);
    q.setZ(odom_quat_.z);
    q.setW(odom_quat_.w);
    stfBaseLinkWrtDriftyOdom_.setRotation(q);
    stfBaseLinkWrtDriftyOdom_.frame_id_ = "drifty_odom";
    stfBaseLinkWrtDriftyOdom_.child_frame_id_ = "base_link";
    //cout<<endl<<"odom_count: "<<odom_count_<<endl;
    //stfDriftyOdomWrtBase_.child_frame_id_ = "drifty_odom"; // make this legal for multiply
    
    // the odom message tells us the estimated pose of the robot's base_frame with respect to the "odom" frame
    // the odom frame is just (0,0,0) when and wherever the robot wakes up;
    // Further, the odom frame is defined to absorb the cumulative odometry drift errors.
    // i.e., it is assumed that the base_frame w/rt odom frame is perfect--but knowledge of the odom frame
    // with respect to the map frame is imperfect, and further, odom w/rt map changes (slowly) in time, according to 
    // odometry drift error accumulation
    
    //We can't publish base_frame w/rt drifty_odom frame, because base_frame is already a child of the "odom" frame
    // published by our simulation  (this will not be a problem with a real robot)
    //here's a trick: publish the drifty_odom frame as a child of the robot's base_frame to avoid a kinematic loop
    //rviz will then be able to transform the drifty_odom frame appropriately in whatever fixed frame is chosen for display
    // use stamped_transform_inverse() function to invert base w/rt odom into odom w/rt base
    stfDriftyOdomWrtBase_ = stamped_transform_inverse(stfBaseLinkWrtDriftyOdom_); 
    //use the TRANSFORM PUBLISHER to send out this transform on the tf topic
    br_.sendTransform(stfDriftyOdomWrtBase_);
    
    //DriftyOdomWrtMap is transform of our drifty_odom frame with respect to the map frame.
    // this is updated by callbacks from AMCL publications of robot w/rt map
    // this transform will update infrequently (e.g. 1Hz), but it also should only change slowly (at the rate of odom drift)

    //cout<<endl<<"stfDriftyOdomWrtMap_"<<endl;
    //xform_utils.printStampedTf(stfDriftyOdomWrtMap_);     
    // cout<<endl<<"stfBaseLinkWrtDriftyOdom_"<<endl;
    //xform_utils.printStampedTf(stfBaseLinkWrtDriftyOdom_);       
    
    //cascde the frames: for frames b==base_link, m==map, do==drifty_odom,
    // 
    // T_b/m = m^T_b = m^T_do * do^T_b = T_do/m * T_b/do
    // put the answer in stfEstBaseWrtMap_
    multiply_stamped_tfs(stfDriftyOdomWrtMap_,stfBaseLinkWrtDriftyOdom_,stfEstBaseWrtMap_);
    //publish this frame, for visualization; change "base_link" to "est_base" to avoid name conflict
    // and kinematic loop;  Visualized result of est_base is an estimate of the robot's pose in
    // the map frame, using amcl and using the imperfect (drifty) odom estimate
    // ideally, this frame is virtually perfect, matching the base_link frame known to rviz
    stfEstBaseWrtMap_.child_frame_id_ = "est_base";
    //cout<<endl<<"stfEstBaseWrtMap_"<<endl;
    // xform_utils.printStampedTf(stfEstBaseWrtMap_);   
    //publish this transform, making it available to rviz, known as "est_base"
    br_.sendTransform(stfEstBaseWrtMap_);

    tf::Vector3 pos1;
    pos1=stfEstBaseWrtMap_.getOrigin();
     odom_x_ = pos1[0];
    odom_y_ = pos1[1];
    odom_quat_.x = stfEstBaseWrtMap_.getRotation()[0];
    odom_quat_.y = stfEstBaseWrtMap_.getRotation()[1];
    odom_quat_.z = stfEstBaseWrtMap_.getRotation()[2];
    odom_quat_.w = stfEstBaseWrtMap_.getRotation()[3];
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    odom_phi_ = convertPlanarQuat2Phi(odom_quat_); 
    current_odom_global.pose.pose.position.x = odom_x_;
    current_odom_global.pose.pose.position.y = odom_y_;
    current_odom_global.pose.pose.orientation = odom_quat_;
      
}

// this callback will get triggered infrequently--only when amcl has updates available
//amcl will publish best-estimate poses of the base-link with respect to the map frame
// these are imprecise, and they are updated slowly (e.g. 1Hz)
// when we get an update, we'll know base-frame w/rt map;
//  at each such update, get the most current base-frame w/rt drifty odom
//  figure out from this where is the drifty odom frame w/rt the map
// we can assume that the drifty-odom frame moves only slowly with respect to the map, so infrequent updates should be OK
// We can re-use the transform: drifty_odom w/rt map repeatedly (e.g. at 50Hz, coincident with drifty-odom publication updates)
//  continue to use the same drifty_odom w/rt map transform until we get a new one  (which should be pretty close to the previous one)
void OdomTf::amclCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_rcvd) {
    amcl_pose_ = amcl_rcvd.pose.pose;
    ROS_WARN("amcl pose: x, y, yaw: %f, %f, %f", amcl_pose_.position.x, amcl_pose_.position.y, convertPlanarQuat2Phi(amcl_pose_.orientation));
    //stfAmclBaseLinkWrtMap_
    amcl_quat_ = amcl_pose_.orientation;
    tf::Vector3 pos;
    pos.setX(amcl_pose_.position.x);
    pos.setY(amcl_pose_.position.y);
    pos.setZ(amcl_pose_.position.z);
    stfAmclBaseLinkWrtMap_.stamp_ = ros::Time::now();
    stfAmclBaseLinkWrtMap_.setOrigin(pos);
    // also need to get the orientation of odom_pose and use setRotation
    tf::Quaternion q;
    q.setX(amcl_quat_.x);
    q.setY(amcl_quat_.y);
    q.setZ(amcl_quat_.z);
    q.setW(amcl_quat_.w);
    stfAmclBaseLinkWrtMap_.setRotation(q);
    stfAmclBaseLinkWrtMap_.frame_id_ = "map";
    stfAmclBaseLinkWrtMap_.child_frame_id_ = "base_link";
    //cout<<endl<<"stfAmclBaseLinkWrtMap_"<<endl;
    //xform_utils.printStampedTf(stfAmclBaseLinkWrtMap_);      
    
    stfDriftyOdomWrtBase_ = stamped_transform_inverse(stfBaseLinkWrtDriftyOdom_); 
    //cout<<endl<<"stfDriftyOdomWrtBase_"<<endl;
    //xform_utils.printStampedTf(stfDriftyOdomWrtBase_);    
    
    multiply_stamped_tfs(stfAmclBaseLinkWrtMap_,stfDriftyOdomWrtBase_,stfDriftyOdomWrtMap_);
    
   //publish the estimated odom frame w/rt the map.  This is useful for visualization, e.g. to see
    //how rapidly the odometry estimate is drifting
    // slight of hand...rename the child frame before publishing, so it does not conflict with
    // existing name "drifty_odom", to avoid a kinematic-loop error in tf
    // result will be corrected_odom w/rt map
    // THIS TRANSFORM SHOULD BE SLOWLY CHANGING if the odom estimates have low drift rate
    // THIS TRANSFORM will only get updated when new amcl estimates are available...e.g. 1Hz
    stfDriftyOdomWrtMap_.child_frame_id_ = "corrected_odom";
    //cout<<endl<<"stfDriftyOdomWrtMap_"<<endl;
    //xform_utils.printStampedTf(stfDriftyOdomWrtMap_);      
    br_.sendTransform(stfDriftyOdomWrtMap_);   
    //done publishing, so restore correct frame name
    stfDriftyOdomWrtMap_.child_frame_id_ = "drifty_odom"; //change this frame name back, for use
    // in odom callback
    
    //amcl makes its own estimate of the pose of the robot's base frame w/rt map
    // publish these updates as they are available.  Use a new name (amcl_base_link) to avoid
    // conflict with "base_link" already used in rviz (which is unrealistically smooth and accurate)
    stfAmclBaseLinkWrtMap_.child_frame_id_ = "amcl_base_link";
    br_.sendTransform(stfAmclBaseLinkWrtMap_);
    amcl_ready_=true;
}
