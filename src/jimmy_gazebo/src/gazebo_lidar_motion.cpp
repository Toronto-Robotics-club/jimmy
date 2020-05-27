/*
* Purpose: At present this code is responsible for putting the lidar at a
*          horizontal position where the light emitted from teh lidar is 
*          parallel to ground. This is needed to make gmapping work. 
*
* Author: MO from https://torontoroboticsclub.com/
*
* Date: early 2020
*
* NOTE: this code is in progress as the lidar needs to "nod" up and down to get 
*       3D data (for the purposes of navigation). 
*/

#include "jimmy_gazebo/gazebo_lidar_motion.h"


// This method is called a "callback function". It is executed every time
// a new message on the topic we are subscribed to arrives. It is called later on
// in the program and is defined first as a C++ coding practice to make functions
// visible before they are used (otherwise need to use a function declaration). 
// NOTE: by putting a const in the arguments, we are making sure that the method
//       does not alter the variable we are passing in.
//arg 0 - (turtlesim) - the package name for the node we want to subscribe to
//        (Pose) - the TYPE of the message. Together this is the type of variable
//                 that is being passed into the method.
//       (&msg) - the memory address of the message we recieved on the topic.
//NOTE: the 'adress of' operator (&) provides the memory address of the variable.
// see http://www.learncpp.com/cpp-tutorial/67-introduction-to-pointers/
void jointStateMessageReceived(const sensor_msgs::JointState &msg) 
{
	//store current position of lidar
    rawLidarPositionInGazebo = msg.position[0];

	//convert the radian value to a range between 0.0 and 2 Pi
	//rawLidarPositionInGazebo = lidarPositionInGazeboToSingleRevolution(rawLidarPositionInGazebo);
}


//this callback function is called by the this node (the subscriber) any time a 
//message arrives on the "/mobile_base_controller/cmd_vel" topic from the 
//diff_drive controller.
void twistDataTypeCallbackFunction(const geometry_msgs::Twist &msg)
{
    //store the Twist data that just arrived via the topic.
    linearVelocity = msg.linear.x;
    angularVelocity = msg.angular.z;
}	


int main(int argc, char **argv) 
{
    //create a node and give it a defalut node name 
    ros::init(argc, argv, "gazebo_lidar_motion_control");

	//create a means to interract with the node.
    ros::NodeHandle nodeHandle;

	//setup the publisher object to give velocity commands to the joint in Gazebo
    gazeboLidarMotionCommandPublisher = nodeHandle.advertise<std_msgs::Float64>("/jimmy_ns/lidar_joint_position_controller/command", 1);

    // This subscriber is getting the position of the lidar joint in Gazebo
    // arg 0 - "/jimmy_ns/joint_states" is the topic name.
    // arg 1 - 1 is the size of the message queue; essentially it is a buffer.
    // arg 2 - pointer to callback function (in this class). Do not include any 
    //         arguments here, all ros really needs is the pointer to the function 
    //         and it (ros) takes care of everything itself. The <&> is optional, 
    //         but it's best to include it to alert humans that we are dealing with 
    //         a pointer and not a method call. To read up on function pointers
    //         see: http://www.learncpp.com/cpp-tutorial/78-function-pointers/
    ros::Subscriber lidarPositionSub = nodeHandle.subscribe("/jimmy_ns/joint_states", 1, &jointStateMessageReceived);

	ros::Subscriber twistDataSub = nodeHandle.subscribe("/cmd_vel", 1, &twistDataTypeCallbackFunction);

    // controls the rate of a loop.
    // NOTE: ros::Rate takes into accont the computation time! see end of pg 53.
    ros::Rate rate(10);

    while(ros::ok())  //ros::ok() means the node is still good (see pg 52 for details)
    {	
		//get Twist data and current position Data
		ros::spinOnce();//call all the callback functions to get latest values.

		//get more horizontal
        goToHorizontalPosition();
		

		//=============================FUTURE TODO=====================================
		//ROS_FATAL_STREAM("");
		//(1) if moving  we want nod flag.
		//(2) if stopped we send home.
		//(3) if nod flag high & position overshot "high limit" pitch down.
		//(4) if nod flag high & position overshot "low limit" pitch up.


        // Wait until it's time for another iteration.
        rate.sleep();		
	}//ros while loop
}//main method

////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////functions//////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
//put the lidar position value between 0.0 and TWO_PI. We don't care for the 
//position as position + cumulative historical data, we only care about position. 
double lidarPositionInGazeboToSingleRevolution(double lidarPosition)
{
	//if the lidar position is equal or larger than 2*Pi, keep reducing it
    //until it is between 0.0 and just shy of 2*Pi (as the two values are the 
    //same position)
	if((0.0 <= lidarPosition) && (lidarPosition > TWO_PI))
	{
		//eliminated having to be larger than 0.0 in case the result ends up at -0.000001
		while(lidarPosition > TWO_PI)
		{
			lidarPosition = lidarPosition - TWO_PI;
		}
	}

    //if the liadar position is negetive, keep increasing it until it is between 
    //0.0 and just shy of 2*Pi (as the two values are the same position)
	if(0.0 > lidarPosition)
	{
		//keep adding 2*pi until we are no longer negetive
		while(0.0 > lidarPosition)
		{
			lidarPosition = lidarPosition + TWO_PI;
		}
	}
	
	return lidarPosition;
}

//when we start the lidar is not home. This is due to an offset I put in the
//URDF to allow the lidar position of the physical robot to match the virtual 
//robot in RVIz. This function erases this.
void goToHorizontalPosition()
{
	//home being horizontal looking forward
	bool lidarAtHome = false;

	//margin for what passes for home position
	double margin = 0.0005;

	//get the latest values
	ros::spinOnce();

	//check if we are home (within a tolerance)
	if(rawLidarPositionInGazebo >= homePosition-margin 
       && rawLidarPositionInGazebo <= homePosition+margin)
	{
		lidarAtHome = true;
	}

	//if we are not horizontal send the radian positin to go to
	if(lidarAtHome == false)
	{
		//go to home position. This sends a position to a position controller
		//that controlls the speed to get to the position.
		gazeboLidarMotionCommand.data = homePosition;
		gazeboLidarMotionCommandPublisher.publish(gazeboLidarMotionCommand);
	}
}
