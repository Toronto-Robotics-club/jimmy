/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 * Authors:
 * Dave Coleman (who is awesome) put together a package to help others understand
 * ros control (see: https://github.com/PickNikRobotics/ros_control_boilerplate). 
 * I (MO) used one of the files in that package as a guide to utilize ros_control:
 * https://github.com/PickNikRobotics/ros_control_boilerplate/blob/melodic-devel/rrbot_control/src/rrbot_hw_interface.cpp
 * All this code was written by me, but I used Dave Coleman's code as a template;
 * As well, I relied on https://github.com/eborghi10/my_ROS_mobile_robot to further
 * understand how to utilize ros_control. I am more of an integrator of the code 
 * onto my hardware, than an author. 
 * 
 * Date: Dec 29, 2019
 *
 * Purpose: This code abstracts two joints (drive wheels to base link) as 
 * arrays, and fills the arrays with the state and command of/for each joint.
 * This format is required to use the ros_control library, with it's precanned
 * controllers. See: https://wiki.ros.org/ros_control
 */


#include <jimmy_hw_interface.h>

///implementation of JimmyHWInterface constructor. The 4 items after the colon 
/// are a means to initialize member variables of the class the constructor is
/// a part of. This is called "dirct initialization of member variables". See:
///https://www.learncpp.com/cpp-tutorial/8-5a-constructor-member-initializer-lists/
JimmyHWInterface::JimmyHWInterface() 
{
    // Intialize raw data
    ///this std lib function fills arrays (arg 1), for a number of elements (arg 2)
    /// a specific value (arg 3). Basically here we are assigning all our 
    ///JointState statuses with 0.0.
    std::fill_n(pos, NUM_JOINTS, 0.0);
    std::fill_n(vel, NUM_JOINTS, 0.0);
    std::fill_n(eff, NUM_JOINTS, 0.0);
    std::fill_n(cmd, NUM_JOINTS, 0.0);

    // connect and register the joint state and velocity interfaces. for more info see:
    ///https://github.com/ros-controls/ros_control/wiki/hardware_interface. Basically
    /// connect the interface to the resouces and the URDF; and give access to control manager
    
    ///*******************************LEFT WHEEL "resource"****************************
    ///the resource name for the left wheel as in the URDF
    std::string left_wheel = "base_to_left_drive_wheel_joint";
    ///This is a joint state interface; which means that this gives the 
    ///controller manager node access to the joint state of the robot.
    hardware_interface::JointStateHandle state_handle_left(left_wheel, &pos[LEFT], &vel[LEFT], &eff[LEFT]);
    jnt_state_interface.registerHandle(state_handle_left);
    ///My understanding is that here, we are giving the controller manager 
    ///node access to the command (the memory which tells the joint what to
    ///do) part of the array.
    hardware_interface::JointHandle vel_handle_left_wheel(jnt_state_interface.getHandle(left_wheel), &cmd[LEFT]);
    jnt_vel_interface.registerHandle(vel_handle_left_wheel);

    ///*******************************RIGHT WHEEL "resource"****************************
    //the resource name for the right wheel as in the URDF
    std::string right_wheel = "base_to_right_drive_wheel_joint";
    ///This is a joint state interface; which means that this gives the 
    ///controller manager node access to the joint state of the robot.
    hardware_interface::JointStateHandle state_handle_right(right_wheel, &pos[RIGHT], &vel[RIGHT], &eff[RIGHT]);
    jnt_state_interface.registerHandle(state_handle_right);
    ///My understanding is that here, we are giving the controller manager 
    ///node access to the command (the memory which tells the joint what to
    ///do) part of the array.
    hardware_interface::JointHandle vel_handle_right_wheel(jnt_state_interface.getHandle(right_wheel), &cmd[RIGHT]);
    jnt_vel_interface.registerHandle(vel_handle_right_wheel);
       
    registerInterface(&jnt_state_interface); //access to state of joint.(pos[], vel[], eff[])
    registerInterface(&jnt_vel_interface); //access to giving joint commands (cmd[])
   
}

//Reading encoder values and setting position and velocity of enconders 
///The read method is going to get data from the robot hardware so that the 
///controller can correct the commands to better reach the desired objective.
void JimmyHWInterface::read(const ros::Duration &period) 
{
    
    //send a status update request to the arduino, and get all the data from the arduino.
    jimmycppArduinoObj.getDataFromArduino(AA_request);
    
    //Reset the encoder data that we were storing:
	rightWheelEncReading_deg = 0.0;
    leftWheelEncReading_deg = 0.0;

    //get the current degrees from arduino:
    rightWheelEncReading_deg = jimmycppArduinoObj.getRightWheelPositionInDegrees();
	leftWheelEncReading_deg = jimmycppArduinoObj.getLeftWheelPositionInDegrees();

    //reset old data that was stored in radians
	rightWheelEncReading_rad = 0.0;
    leftWheelEncReading_rad = 0.0;

    //convert the degree values we got from arduino into radians:
    //store the position of the wheels as gotten from arduino in radians.
    rightWheelEncReading_rad = rightWheelEncReading_deg * degToRadConst;
	leftWheelEncReading_rad = leftWheelEncReading_deg * degToRadConst;


    //write robot data to the right structure.
    //0=left, 1=right
    pos[LEFT] = leftWheelEncReading_rad;
    vel[LEFT] = leftWheelEncReading_rad / period.toSec();
    pos[RIGHT] = rightWheelEncReading_rad;
    vel[RIGHT] = rightWheelEncReading_rad / period.toSec();
}//end of read

/// I believe this method is how we send commands to the hardware. 
void JimmyHWInterface::write() 
{
    /// cmd[int] - this is the command that is returned to us by the 
    /// diff_drive controller. Lets store the values to make sure they are safe. 
    double diff_speed_left = cmd[LEFT];
    double diff_speed_right = cmd[RIGHT];

    //did right or left command exceed robot limits?
    bool leftMaxCommandIssued = false;
    bool rightMaxCommandIssued = false;
    double speedReductionRatio = 0.0;

    //1.2936 to -1.2936
    if((diff_speed_left >= 1.2935) || (diff_speed_left <= -1.2935))
    {
        leftMaxCommandIssued = true;
    }

    //1.2936 to -1.2936
    if((diff_speed_right >= 1.2935) || (diff_speed_right <= -1.2935))
    {
        rightMaxCommandIssued = true;
    }

    //if we hit the max value, then we need to fix up our YAML file, but for now
    //the plan is to simply maintain the ratio of the commands to achieve the 
    //correct motion at a slower speed. Here we figure out the ratio.
    if((leftMaxCommandIssued==true) || (rightMaxCommandIssued==true))
    {
        //find which value is the largest magnitude via absolute values
        double absLeft = std::abs(diff_speed_left);
        double absRight = std::abs(diff_speed_right);
        
        //set teh speed reduction ration based on the larger max value.
        if(absLeft >= absRight)
        {
            speedReductionRatio = (1.29/absLeft);
        }

        if(absLeft < absRight)
        {
            speedReductionRatio = (1.29/absRight);
        }
        
        //let the user know we have a problem
         ROS_FATAL_STREAM("drive wheel command by diff_drive controller is excessive!! fix yaml file");
    }

    //if we didn't have an excessive speed command, then send the command to the drives
    // via jimmy cpp:
    if(speedReductionRatio == 0.0)
    {
        //Send command to left stepper drive
        jimmycppAmpSteppersObj.processControllerCommandAndSendToLeftDrive(cmd[LEFT]);
        //send command to right stepper drive
        jimmycppAmpSteppersObj.processControllerCommandAndSendToRightDrive(cmd[RIGHT]);
    }

    //we need to done down te speeds, but still keep the ratio the same so that
    // the motion trajectory is the same, but the velocity is acceptable for the
    // hardware. 
    if(speedReductionRatio != 0.0)
    {
        //step down the speeds for both wheels proportionally, so that we maintain
        //the correct ratio, but are below the max physical speed of the robot.
        diff_speed_left = diff_speed_left * speedReductionRatio;
        diff_speed_right = diff_speed_right * speedReductionRatio;

        //Send command to left stepper drive
        jimmycppAmpSteppersObj.processControllerCommandAndSendToLeftDrive(diff_speed_left);
        //send command to right stepper drive
        jimmycppAmpSteppersObj.processControllerCommandAndSendToRightDrive(diff_speed_right);
    }
} //end of write()



///This method stored ros::Time type data into two variables. One variable is
/// holding time of last update, and other variable is holding the time right
/// now. This method returns the time right now. 
ros::Time JimmyHWInterface::get_time() 
{
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
}
  
///compute the duration between the previous update, and the current time; 
/// where it is likely we are also doing an update at the current time.  
ros::Duration JimmyHWInterface::get_period() 
{
    return curr_update_time - prev_update_time;
}
