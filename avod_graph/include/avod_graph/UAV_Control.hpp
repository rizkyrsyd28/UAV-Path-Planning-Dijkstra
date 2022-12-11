#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>

// Drone Control copied and modified from https://github.com/Intelligent-Quads/iq_gnc

/**
\defgroup control_functions
This module is designed to make high level control programming more simple. 
*/


mavros_msgs::State current_state_g;
nav_msgs::Odometry current_pose_g;
geometry_msgs::Pose correction_vector_g;
geometry_msgs::Point local_offset_pose_g;
geometry_msgs::PoseStamped waypoint_g;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;
float local_desired_heading_g; 



ros::Publisher local_pos_pub;
ros::Publisher global_lla_pos_pub;
ros::Publisher global_lla_pos_pub_raw;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient command_client;
ros::ServiceClient auto_waypoint_pull_client;
ros::ServiceClient auto_waypoint_push_client;
ros::ServiceClient auto_waypoint_set_current_client;
/**
\ingroup control_functions
This structure is a convenient way to format waypoints
*/
struct gnc_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	float psi; ///< rotation about the third axis of your reference frame
};

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg);
//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);

geometry_msgs::Point get_current_location();

float get_current_heading();

//set orientation of the drone (drone should always be level) 
// Heading input should match the ENU coordinate system
/**
\ingroup control_functions
This function is used to specify the drone’s heading in the local reference frame. Psi is a counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
void set_heading(float heading);
// set position to fly to in the local frame
/**
\ingroup control_functions
This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
void set_destination(float x, float y, float z, float psi);

int wait4connect();

int wait4start();

int initialize_local_frame();

int arm();

int takeoff(float takeoff_alt);

/**
\ingroup control_functions
This function returns an int of 1 or 0. THis function can be used to check when to request the next waypoint in the mission. 
@return 1 - waypoint reached 
@return 0 - waypoint not reached
*/
// int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01);

/**
\ingroup control_functions
this function changes the mode of the drone to a user specified mode. This takes the mode as a string. ex. set_mode("GUIDED")
@returns 1 - mode change successful
@returns 0 - mode change not successful
*/
int set_mode(std::string mode);

/**
\ingroup control_functions
this function changes the mode of the drone to land
@returns 1 - mode change successful
@returns 0 - mode change not successful
*/
int land();

/**
\ingroup control_functions
This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input 
@returns n/a
*/
int init_publisher_subscriber(ros::NodeHandle controlnode);