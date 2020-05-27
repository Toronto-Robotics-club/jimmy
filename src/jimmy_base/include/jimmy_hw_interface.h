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
 *********************************************************************/

/* Author: Dave Coleman > modified by Emiliano Borgi > modified by MO
   Desc:   See the jimmy_hw_interface.cpp file for details.
*/

#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
// ostringstream
#include <sstream>
/// my two low level libraries
#include <jimmycpp/ampsteppers.h>
#include <jimmycpp/arduinomega.h>

/// yep... on the mobile base, two joints.
const unsigned int NUM_JOINTS = 2;
const unsigned int LEFT=0;
const unsigned int RIGHT=1;

/// Class representing Jimmy hardware, allows for ros_control to modify internal 
/// state via joint interfaces (where an interface is a collection of resources,
/// in this case 2 resources that are the motors turning the wheels).
/// NOTE: the class "JimmyHWInterface" is iheriting from RobotHW; where JimmyHWInterface
/// is the child class, and RobotHW is the parent class. The type of inheritance
/// used is public inheritance.
/// Reference: https://www.learncpp.com/cpp-tutorial/112-basic-inheritance-in-c/
class JimmyHWInterface : public hardware_interface::RobotHW
{

public:///set all functions below as public

    JimmyHWInterface(); ///constructor declaration
    
    void write(); /// I believe this method is how we send commands to the hardware.
     
    ///The read method is going to get data from the robot hardware so that the 
    ///controller can correct the commands to better reach the desired objective.
    ///After reading values, we set pos[] and vel[] for each joint.
    void read(const ros::Duration &period); //Reading encoder values and setting position and velocity of enconders

    ///This method stored ros::Time type data into two variables. One variable is
    /// holding time of last update, and other variable is holding the time right
    /// now. This method returns the time right now. 
    ros::Time get_time();

    ///compute the duration between the previous update, and the current time; 
    /// where it is likely we are also doing an update at the current time.  
    ros::Duration get_period();

    ///NodeHandle objects is how we create publisher & subscriber objects.
    ros::NodeHandle nh;

private: ///all these items are likely inherited from hardware interface.

    ///This interface (which represents several resources (2 motors in my case))
    /// is responsible for collecting the state of the wheels. I am guessing in 
    /// radians. 
    hardware_interface::JointStateInterface jnt_state_interface;
    
    ///This interface (which represents the same 2 motors is responsible for )
    /// sending velocity commands recived from the controller to the underlying
    /// hardware. 
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    
    ///These arrays store all known info about our two resources (the diff_drive
    ///motors).
    double cmd[NUM_JOINTS];
    double pos[NUM_JOINTS];
    double vel[NUM_JOINTS];
    double eff[NUM_JOINTS];

    ///The top speed we are allowed to issue to the wheels. units are rad/sec, 
    ///and direction is relative to the robot. Jimmy values. 
    const double _max_speed_forward = 1.2936;
    double _max_speed_backward = -1.2936; 

    //keep state of the number of encoder pulses (in degrees) from a referance
    //point for both wheels. forward increments the count, backwards decrements
    //it. The values are fetched by the read() method.  
    double _wheel_angle[NUM_JOINTS];

    ///define some variables that will hold time stamps
    ros::Time curr_update_time;
    ros::Time prev_update_time;
    
    ///jimmycpp object handle for communication with arduino which is responsible
    /// for communicating with the drive wheel encoders. :
    jimmycpp::ArduinoMega jimmycppArduinoObj;

    //a contant to convert degrees to radians
    const double degToRadConst = 3.141592653/180.0;

    //This is the AA~ message (where AB~ is a test message)
    const int AA_request = 1;

    //the variables that hold the wheel status in degrees since we started moving.
    double rightWheelEncReading_deg;
    double leftWheelEncReading_deg;

    double rightWheelEncReading_rad;
    double leftWheelEncReading_rad;

    //jimmycpp object handle for sending commands to stepper drives.
    jimmycpp::AmpSteppers jimmycppAmpSteppersObj;

};  // end of JimmyHWInterface class "definition"
