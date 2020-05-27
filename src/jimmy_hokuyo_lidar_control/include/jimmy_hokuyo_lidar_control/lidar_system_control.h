/*
	NOTE: for purpose, author, date and other info, please consult lidar_system_control.cpp
*/

#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

//#include <chrono>
//#include <functional>
//the type of message that contains data representing the move desired by the
//node sending this message type on the topic.
#include <jimmy_hokuyo_lidar_control/lidar_message.h>

/// my low level library
#include <jimmycpp/lidar_magnetic_encoder.h>
#include <jimmycpp/lidar_stepper_motion.h>

//ROS_FATAL_STREAM("Up to here OK......");
//This is my handle for my library.


//===================define o/s serial ports====================================
//NOTE: you must configure a symbolic link between each arduino and the port the 
//O/S assigns based on the serial port. 
//see: https://askubuntu.com/questions/1230334/how-to-assign-serial-port-name-using-rules-d/1230575#1230575
//define path to ENCODER Arduino USB serial port in the O/S
const std::string encoderArduinoPath = "/dev/ttyACM10";
//define path to STEPPER Arduino USB serial port in the O/S
const std::string stepperArduinoPath = "/dev/ttyACM11";



//===================define arduino messages====================================

//define the message types the ENCODER Arduino can handle
const std::string encoderSerialCommTestCommand = "AA~";//fd
const std::string setEncoderHomePositionCommand = "AB~";
const std::string getEncoderDegreeAndUncompPosition = "AC~";

//define the message types the ENCODER Arduino can handle
const std::string stepperSerialCommTestCommand = "AA~";
const std::string isLidarAtLimitSwitch = "AB~";
const std::string isLidarLimitSwitchOk = "AC~";
const std::string commandToMoveLidarToLimitSwitch = "AD~";
const std::string commandToSweepFrontOfRobot = "AE~";
const std::string commandToSweepBackOfRobot = "AF~";
const std::string commandToPutLidarHorizontal = "AG~";


//=======================Object Creation=======================================
//create an ENCODER lib object, the arg is the path to the serial port.
LidarMagneticEncoder magneticEncoderObj(encoderArduinoPath);

//create a STEPPER motor object, the arg is the path to the serial port.
LidarStepperPulses stepperMotorObj(stepperArduinoPath);


//============================VARIABLES=========================================
const int fullEncoderRev = 16383; //counting from zero
const double pi = 3.14159265358979323;

//the names of the joints in the urdf that we will publish positino for.
double lidarAngle;
double left_castor_mount_to_left_castor_dummy_joint;
double left_castor_dummy_to_left_castor_wheel_joint;
double right_castor_mount_to_right_castor_dummy_joint;
double right_castor_dummy_to_right_castor_wheel_joint;

const int initValue = -2;//a value used for initialization of ints
const int errorFlag = -1;//there must be some issue...

//variables used throughout the program to get position data from encoder controller
int degree;
int position;
std::pair<int, int> arduinoDegreeAndPositionData;

//comm status to arduinos
int encoderCommStatus;
int stepperCommStatus;

//recovered Comm to stepper controller via homing...
bool recoveredCommViaHoming;
bool homingSuccessful;


//==========================FUNCTIONS=========================================
void initializeVariables();
bool isLidarMotorInMotion();
int recoverStepperControllerCommunications();
void homeLidarSystem();
void setSoftHomePositionOnEncoderSystem();
void sweepFrontAndStopDemo();
bool encoderHasError();
double returnPositionAsRadians(int);


