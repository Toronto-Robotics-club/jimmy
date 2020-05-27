/*
	NOTE: please consult gazebo_lidar_motion.cpp for information regarding
          purpose of code, author, and dates.
*/



//all ros libraries
#include <ros/ros.h>
//messages regarding the current state of the lidar joint. Got info by running
//$rostopic info /jimmy_ns/joint_states 
#include <sensor_msgs/JointState.h>
//give command to gazebo. Got info by running: 
//$rostopic info /jimmy_ns/lidar_joint_position_controller/command
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

//=============================ROS handles======================================
//ros publisher object handle
ros::Publisher gazeboLidarMotionCommandPublisher;

//the mechanics of how to use the std_msg data types are shown here:
//https://stackoverflow.com/questions/23497130/need-information-regarding-publishing-a-double-variable-on-to-a-ros-topic
//is must be noted that it is recommended not to use these data types directly
//(https://wiki.ros.org/std_msgs). It is suggested to use them as building blocks 
//for topic messages as that makes the use easier to understand. However, there 
//is nothing more permenant than temporary solutions. Look here for advice: 
std_msgs::Float64 gazeboLidarMotionCommand;

//===========================Variables==========================================
//store the data that the diff_drive is subscribing to:
double linearVelocity;
double angularVelocity; 

bool robotMoving;

//this is the current lidar position in Gazebo. It is in radians, and is additive,
//meaning every revolution adds or subrtracts 2PI to the current value. Essentially
//it gives position and the historical total motion. 
double rawLidarPositionInGazebo;

//store the value of Pi, 2*PI, etc..
const double PI =     3.141592653;
const double TWO_PI = 6.283185307;
//const double stopLidarMotion = 0.0;
//const double midSpeedLidarMotion_stepperFaceCW = 0.2;
//double superSlowLidarMotion_stepperFaceCW = 0.00005;
//double superSlowLidarMotion_stepperFaceCCW = -0.00005;

//NOTE: this value is from the URDF. It is the 'origin' "p" value (from rpy) for 
//the continious joint for nodding the lidar. If it changes there, this will 
//have to change..... TODO
const double homePosition = -1.25;

//=============================Functions========================================

//put the lidar position value between 0.0 and TWO_PI. We don't care for the 
//position as position + cumulative historical data, we only care about position. 
double lidarPositionInGazeboToSingleRevolution(double);

//on startup go to horizontal position. This gets us to ball park horizontal.
void goToHorizontalPosition();






