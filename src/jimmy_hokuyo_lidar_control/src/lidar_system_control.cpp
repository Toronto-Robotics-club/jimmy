/*******************************************************************************
*  Purpose: This code sends commands to the lidar system stepper motor (via    *
*           jimmycpp) and recieves the status of the magenetic encoder on the  *
*           lidar system (also via jimmycpp). A summary of the commands we can *
*           send to the Arduinos is:                                           *
*           (1) Stepper Arduino:                                               *
*               AA~ test if the communication is working                       *
*               AB~ is the lidar at the limit switch?                          *
*               AC~ is the limit switch ok?                                    * 
*               AD~ command to move to limit switch                            *
*               AE~ command to sweep the front of the robot.                   *
*               AF~ command to sweep the rear of the robot.                    *
*               AG~ command to go to a horizontal position.                    *
*           (2) Enocder Arduino:                                               *
*               AA~ test if the communication is working                       *
*               AB~ set encoder home position. this is the new refrence point. *
*               AC~ get encoder values (14 bit position, degree, and error     * 
*                   flag.                                                      *
*                                                                              *
*  Author: MO at https://torontoroboticsclub.com/                              *
*                                                                              *
*  Date: March 2020                                                            *
*******************************************************************************/


#include <jimmy_hokuyo_lidar_control/lidar_system_control.h> //a header for this file.

//main method declaration
int main(int argc, char **argv) 
{
    // Initialize the ROS system and become a node. Last string is
 	// the default node name.
  	ros::init(argc, argv, "lidar_hokuyo_control_node");

  	//this is the main mechanism for my code to interact with ROS...
  	// instantiating this object registers my code with the master.
  	ros::NodeHandle nodeHandle;

    // Create a publisher object; where a Publisher object will need to be created for
    // every topic that a node want to send messages on. to publish on multiple 
    // topics create multiple Publisher objects. Individual Items are as follows:
    // nodeHandle.advertise - is a factory for making publisher objects.
    // <std_msgs::Float32> - this is the message TYPE; the data type for the 
    //                          messages we will publish
    // "/lidar_enc_pos_enc" - this is the topic name on which we want to publish. this should be
    //                    the same as the string from <topic_list or rqt_graph> 
    //                    commands (but without the / character to make the name
    //                    relative (see ch 5 in agitr or pg 50 at the top)
    // ATTENTION - do not confuse the "Topic-name" and "Message-Type"
    // 1 - the message queue size. Oldest message will be discarded if code publishes messages
    //        than the queue can hold. Messages are put into a queue because messages are generated
    //        faster then they are sent on the network (as generation of messages is faster than 
    //        almost any network)
    //ros::Publisher pub = nodeHandle.advertise<std_msgs::Float32>("/joint_state", 1);

    //the joint state message type describes a torque controlled joint (between
    // two links). This line creates a publisher object. Arg #1 is topic name; 
    // arg 2 is number of messages to keep in the buffer.
    ros::Publisher joint_pub = nodeHandle.advertise<sensor_msgs::JointState>("/joint_states", 1);

    //create a handle for this data type. I will populate this data type with 
    // data and send via the publisher.
    sensor_msgs::JointState joint_state;

	//=============================Setup hokuyo hardware====================================
	//this basically does the homing, and sets a soft home position.....
	initializeVariables();

	//check communication with both arduinos
	encoderCommStatus = magneticEncoderObj.getDataForCommand_AA(encoderSerialCommTestCommand);
	//std::cout << "Encoder: " << encoderCommStatus << std::endl; //DELETE
	stepperCommStatus = stepperMotorObj.getDataForCommand_AA(stepperSerialCommTestCommand);
	//std::cout <<  "Stepper: " << stepperCommStatus << std::endl; //DELETE

	//since there are no blocking functions in the encoder controller, if we cannot communicate,
    //there trully is an issue. need to exit and figure it out.
    if(encoderCommStatus != 1)
    {
        std::cout << "we cannot recover if encoder controller has no comm. Exiting.... " << std::endl;
        return errorFlag;
    }

	//if we have no communication to the stepper controller, that is not a dead
	//end. Since it has blocking functions, it is possible that it is simply busy;
	//for example, this program terminated & the arduino is still moving... and 
	//now we just launched again. In this case try to recover the communication
	if(stepperCommStatus != 1)
	{
		int recoveryFlag = recoverStepperControllerCommunications();
		
		//if value is negetive, we could not recover communication, and we need
		//to exit the node.
		if(recoveryFlag < 0)
		{
			std::cout << "could not recover comm with stepper controller" << std::endl;
			return errorFlag;
		}
	}

	//final serial comm check 
    if(magneticEncoderObj.getDataForCommand_AA(encoderSerialCommTestCommand) != 1 ||
       stepperMotorObj.getDataForCommand_AA(stepperSerialCommTestCommand) !=1)
    {
        std::cout << "FATAL ERROR: comm is flaky. here is current status of comm: " << std::endl;
        std::cout << "Encoder: " << magneticEncoderObj.getDataForCommand_AA(encoderSerialCommTestCommand) << std::endl;
        std::cout <<  "Stepper: " << stepperMotorObj.getDataForCommand_AA(stepperSerialCommTestCommand) << std::endl;
        return errorFlag;
    }

    std::cout << "Both Arduinos are communicating properly" << std::endl;

	//if we just homed via recovery no need to home, otherwise, send the system home.
    if(recoveredCommViaHoming == false)
    {
		homeLidarSystem();
	}


	//this function will set the home position on the Encoder controller. Basically
	//all the pulses are now referenced from the home position. 
	setSoftHomePositionOnEncoderSystem();

	//since homeLidarSystem() is blocking, and if we already recovered by homing
    //we are home now. Let's set this as 0.0

	//a silly demo...
	//sweepFrontAndStopDemo();

	//send to working position
	stepperMotorObj.getDataForCommand_AG(commandToPutLidarHorizontal);
	//>>>>>>>>>>>>>>>>>>>>END OF Hokuyo Setup<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	//publish zeroes for the links for which I have no encoders to keep everything happy
    left_castor_mount_to_left_castor_dummy_joint=0.0;
    left_castor_dummy_to_left_castor_wheel_joint=0.0;
    right_castor_mount_to_right_castor_dummy_joint=0.0;
    right_castor_dummy_to_right_castor_wheel_joint=0.0;	


    //initialize a variable to store the final data
    lidarAngle=0.0;//supposed to be radians....


  	// controls the rate of publishing - Loop at 2Hz until the node is shut down.
  	// NOTE: ros::Rate takes into accont the computation time! see end of pg 53.
  	ros::Rate pubRate(30);


  
	while(ros::ok())  //ros::ok() means the node is still good (see pg 52 for details)
  	{
		//get data from hardware:
		//get latest degree and position data from encoder controller.
    	arduinoDegreeAndPositionData = magneticEncoderObj.getDataForCommand_AC(getEncoderDegreeAndUncompPosition);
    	degree = arduinoDegreeAndPositionData.first;
    	position = arduinoDegreeAndPositionData.second;

		//error check
		if(degree == -1)
		{
			//fun fact: the -1 comes from the arduino lib that communicates with the
			//hardware.
			ROS_FATAL_STREAM("The encoder has an error. could be the power, the spi comm,");
			ROS_FATAL_STREAM("or an actual hardware failure.");
			return -1;
		}

		lidarAngle = returnPositionAsRadians(position);


		//update joint_state. Make sure the names match the URDF file!
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(5); //resize array
        joint_state.position.resize(5); //resize array
        joint_state.name[0] ="hokuyo_stepper_shaft_to_hokuyo_base_plate_joint"; //name of joint
        joint_state.position[0] = lidarAngle; //value
        joint_state.name[1] ="left_castor_mount_to_left_castor_dummy_joint";
        joint_state.position[1] = left_castor_mount_to_left_castor_dummy_joint;
        joint_state.name[2] ="left_castor_dummy_to_left_castor_wheel_joint";
        joint_state.position[2] = left_castor_dummy_to_left_castor_wheel_joint;
        joint_state.name[3] ="right_castor_mount_to_right_castor_dummy_joint";
        joint_state.position[3] = right_castor_mount_to_right_castor_dummy_joint;
        joint_state.name[4] ="right_castor_dummy_to_right_castor_wheel_joint";
        joint_state.position[4] = right_castor_dummy_to_right_castor_wheel_joint;

		

        // Publish the message. The publisher object has a method to publish the 
        // message we built.
        //pub.publish(enc_pos);
        //send the joint state and transform
        joint_pub.publish(joint_state);

		// Wait until it's time for another iteration.
		pubRate.sleep();
  	}
}

////////////////////////////////////////FUNCTIONS/////////////////////////////

//convert the position of the encoder into a radian position TODO
double returnPositionAsRadians(int currentPosition)
{ 
    //this eliminates the whole "laser needs to be mounted planar" error.
	const double eliminateGmappingPlanarError = 0.008;

	//get the current 14 bit encoder position as a decimal
	double positionAsFraction = double(currentPosition)/double(fullEncoderRev);
	//std::cout << "Show fraction: " << positionAsFraction << std::endl;

   	return ((positionAsFraction * 2 * pi) + eliminateGmappingPlanarError);
}


//this function resets the values which represent the position of the lidar.
void initializeVariables()
{
	degree = initValue;
	position = initValue;

	encoderCommStatus = initValue;
	stepperCommStatus = initValue;

	recoveredCommViaHoming = false;
	homingSuccessful = false;
}

//using the encoder arduino figure out if the lidar is moving
//function to check if lidar is in motion.
bool isLidarMotorInMotion()
{
    //initialize our return value
    bool result = true;

    //initialze variables
    int _degree = -1;

    std::pair<int, int> _arduinoDegreeAndPositionData;

    //run 2 iterations
    for(int i=0; i !=2; i++)
    {
        _arduinoDegreeAndPositionData = magneticEncoderObj.getDataForCommand_AC(getEncoderDegreeAndUncompPosition);
        _degree = _arduinoDegreeAndPositionData.first;
        std::cout << "degreeOld: " << _degree << std::endl;

        sleep(1);

        _arduinoDegreeAndPositionData = magneticEncoderObj.getDataForCommand_AC(getEncoderDegreeAndUncompPosition);
        int degreeNow = _arduinoDegreeAndPositionData.first;
        std::cout << "degreeNow: " << degreeNow << std::endl;
        std::cout << "----------------------------------------------------" << std::endl;

        //if the two values are equal just exit...
        if(_degree == degreeNow)
        {
            std::cout << "No motion as degrees are equal." << std::endl;
            result = false;
        }

        //accomodate the dancing of degrees (noise from sensor).
        if(_degree+1 == degreeNow || _degree-1 == degreeNow)
        {
            std::cout << "Assuming no motion  as degrees close enough." << std::endl;
            result = false;
        }

        i++;
    }

    return result;
}


//recover stepper controller communication (perhaps it was busy when we tried
//to establish comm (it has blocking functions).
int recoverStepperControllerCommunications()
{
	std::cout << "Issue with stepper controller comm. let's check if system is moving using encoder data." << std::endl;

	//check for motion using encoder controller.
    bool systemMoving = isLidarMotorInMotion();

    if(systemMoving == true)
    {
        std::cout << "System moving...Trying to recover stepper comm...." << std::endl;
        stepperMotorObj.getDataForCommand_AD(commandToMoveLidarToLimitSwitch);
        int i = 0;

        while(i < 30)
        {
            if(isLidarMotorInMotion() == false)
            {
                std::cout << "recovery seems successful. Lidar system stopped." << std::endl;
                recoveredCommViaHoming = true;
                break;
            }
            sleep (10);
            i++;
        }

        if(isLidarMotorInMotion() == true)
        {
            //it will not take more than 30 seconds to try to recover... so exit.
            if(i >=4)
            {
                std::cout << "could not stop the system. Recovery failed." << std::endl;
				return errorFlag;
            }
        }
    }

    //try again
    if(systemMoving == false)
    {
        stepperCommStatus = stepperMotorObj.getDataForCommand_AA(stepperSerialCommTestCommand);
        std::cout << "with system stopped, are we communicating? " << stepperCommStatus << std::endl;

        //if we are not moving and still have an issue, there must be an issue, exit.
        if(stepperCommStatus != 1)
        {
            std::cout << "we cannot recover comm with stepper controller. Exiting.... " << std::endl;
            return errorFlag;
        }
    }
	
	//if we didn't exit with an error flag of -1, all must be OK....
	return 0;
}

void homeLidarSystem()
{
	std::cout << "homing as we did not recover comm via homing" << std::endl;
    //command to home
    int commandSent = stepperMotorObj.getDataForCommand_AD(commandToMoveLidarToLimitSwitch);

	std::cout << "sent the homing command? " << commandSent << std::endl;

    //Are we home yet?
    //since we are homing (and the step function is a blocking function) , we cannot run
    //the stepper AB~ command (are you home?). So, let's get encoder readings, and if they are
    //steady, we can assume we are home, and we can then run AB~. This makes more sense than
    //sleeping.
	bool motionHasStopped = false;

    //19 seconds is the longest possible homing. when we get out of this loop lidar is home. I made the
	//value 25, just to accomodate some extra space.
	while(motionHasStopped == false)
    {
		motionHasStopped = isLidarMotorInMotion();

		sleep(1);

    }

    //confirm we are home using limit switch
    int homeConfirmation = stepperMotorObj.getDataForCommand_AB(isLidarAtLimitSwitch);
    std::cout << "home confirmation: " << homeConfirmation << std::endl;

    if(homeConfirmation ==1)
    {
    	homingSuccessful = true;
	}
}

void setSoftHomePositionOnEncoderSystem()
{
	//==============show pre soft home value encoder data=============================
    //get latest degree and position data from encoder controller.
    arduinoDegreeAndPositionData = magneticEncoderObj.getDataForCommand_AC(getEncoderDegreeAndUncompPosition);

    degree = arduinoDegreeAndPositionData.first;
    position = arduinoDegreeAndPositionData.second;

    std::cout << "(AC~): degree: " << degree << " | position: " << position << std::endl;

	//===================set soft home position values=====================
    //confirm we are still home
    int stillHome = stepperMotorObj.getDataForCommand_AB(isLidarAtLimitSwitch);

    //we are home for sure. Meaning we are still home, and we called the stepper system
    //homing command (AD~) in comm recovery or during regular homing.
    if(stillHome == 1 && (homingSuccessful == 1 || recoveredCommViaHoming == 1))
    {
        //sleep for 2 seconds to give a chance for the unit to come to a rest in case
        //we triggered the limit switch but are still moving
        sleep(2);

        //since we are home, set the encoder zero position
        int resultOfSoftHomeSetting = magneticEncoderObj.getDataForCommand_AB(setEncoderHomePositionCommand);
        std::cout << "Were we able to set the encoder home position? " << resultOfSoftHomeSetting << std::endl;
    }

    //show the new soft values:
    std::cout << "showing that the soft home worked:" << std::endl;
    //get latest degree and position data from encoder controller.
    arduinoDegreeAndPositionData = magneticEncoderObj.getDataForCommand_AC(getEncoderDegreeAndUncompPosition);

    degree = arduinoDegreeAndPositionData.first;
    position = arduinoDegreeAndPositionData.second;

    std::cout << "(AC~): degree: " << degree << " | position: " << position << std::endl;

}

void sweepFrontAndStopDemo()
{
    std::cout << "=================DEMO=======================" << std::endl;
    int j = 0;

    while(j <= 1000)
    {
        //give a pitching command to stepper subsystem
        if(j == 0)
        {
            stepperMotorObj.getDataForCommand_AE(commandToSweepFrontOfRobot);
        }

        //get encoder positional data:
        arduinoDegreeAndPositionData = magneticEncoderObj.getDataForCommand_AC(getEncoderDegreeAndUncompPosition);

        degree = arduinoDegreeAndPositionData.first;
        position = arduinoDegreeAndPositionData.second;

        std::cout << "(AC~): degree: " << degree << " | position: " << position << std::endl;

        j++;
    }

    //send a homing command.
    std::cout << "Demo done. go home." << std::endl;
    //command to home
    stepperMotorObj.getDataForCommand_AD(commandToMoveLidarToLimitSwitch);
}


