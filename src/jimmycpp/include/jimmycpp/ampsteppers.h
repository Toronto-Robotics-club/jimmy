/*
    For Purpose (and comm protocol), Author, and Date, have a look at "ampsteppers.cpp"

    version: 1.00 (conversion to OOP)
*/


#ifndef AMPSTEPPERS_H
#define AMPSTEPPERS_H
#include <iostream> //for std::string
#include <arpa/inet.h> //inet_addr (for sockaddr_in)
#include <vector>
#include <chrono>
#include "trc_logging.h" // my logging library
#include <netinet/tcp.h>



//the namespace
namespace jimmycpp
{

   //to access the class use this syntax: jimmycpp::AmpSteppers
	class AmpSteppers
	{
	    private:

	        //This struct is passed to the recv() method to achieve a user defined timeout. See:
	        //https://stackoverflow.com/questions/2876024/linux-is-there-a-read-or-recv-from-socket-with-timeout/2939145#2939145
            struct timeval structToSetRecvTimeOut;


            //these bools are used to make sure the logs make sense (prevent an entry for failure followed by
            //an entry for success for the same item.
            bool m_leftSocketConnectionFailure;
            bool m_rightSocketConnectionFailure;

	        //constants for converting rad/sec or m/sec that I anticipate
            //to get from the diff_drive_controller into RPS (rotations per second)
            //from the motor.
            const double m_oneRevInRad = 6.2831853072;
            const int m_oneRevInDeg = 360;
            const int m_oneRevInStepperPulses = 20000;
            const double m_driveWheelCircumfInCm = 112.8;
            const double m_driveWheelCircumfInM = 1.128;
            const int m_teethOnMotorGear = 14;
            const int m_teethOnWheelGear = 34;
            const double m_gearingRatio13over34 = 0.411765;
            const double m_oneMotorStepAsRad = 20000/m_oneRevInRad;
            const double m_topSpeedOfStepperMotorInRPS = 0.5;


            //***************** HEX Values to build commands****************
            //numbers
            const char m_ZERO_HEX = 0x30;
            const char m_ONE_HEX = 0x31;
            const char m_TWO_HEX = 0x32;
            const char m_THREE_HEX = 0x33;
            const char m_FOUR_HEX = 0x34;
            const char m_FIVE_HEX = 0x35;
            const char m_SIX_HEX = 0x36;
            const char m_SEVEN_HEX = 0x37;
            const char m_EIGHT_HEX = 0x38;
            const char m_NINE_HEX = 0x39;
            const char m_MINUS_HEX = 0x2D; //negative sign.
            const char m_DOT = 0x2E;

            //letters
            const char m_A_HEX = 0x41;
            const char m_C_HEX = 0x43;
            const char m_D_HEX = 0x44;
            const char m_E_HEX = 0x45;
            const char m_F_HEX = 0x46;
            const char m_I_HEX = 0x49;
            const char m_J_HEX = 0x4A;
            const char m_K_HEX = 0x4B;
            const char m_L_HEX = 0x4C;
            const char m_M_HEX = 0x4D;
            const char m_P_HEX = 0x50;
            const char m_R_HEX = 0x52;
            const char m_S_HEX = 0x53;



            //message header and footer
            const char m_HEADER_ONE = 0x00;
            const char m_HEADER_TWO = 0x07;//message must begin with this header
            const char m_FOOTER_HEX = 0x0D;//message must end with this foother (Carrige Return)

            //**************** Already built commands*********************
            //NOTE: the message must have a header of 07 (two bytes), and a footer of D (one byte).
            // Drive Command ME (motor enable)
            int m_ME_size = 5;//size of c style array
            const char m_ME[5] = {m_HEADER_ONE, m_HEADER_TWO, m_M_HEX, m_E_HEX, m_FOOTER_HEX};


            // Drive Command JA5 (Jog Accel). This allows the acceleration to be 5RPS per second.
            int m_JA5_size = 6;//size of c style array
            const char m_JA5[6] = {m_HEADER_ONE, m_HEADER_TWO, m_J_HEX, m_A_HEX, m_FIVE_HEX, m_FOOTER_HEX};


            // Drive Command JL5 (Jog Decel). This allows the deceleration to be 5RPS per second.
            int m_JL5_size = 6;//size of c style array
            const char m_JL5[6] = {m_HEADER_ONE, m_HEADER_TWO, m_J_HEX, m_L_HEX, m_FIVE_HEX, m_FOOTER_HEX};


            // Drive Command JS (Jog Speed). This is the speed (0.5 revolutions per second) that the
            // the drive will rotate the motor at when it recieves a JC instruction.
            int m_JS0dot5_size = 8;//size of c style array
            const char m_JS0dot5[8] = {m_HEADER_ONE, m_HEADER_TWO, m_J_HEX, m_S_HEX, m_ZERO_HEX, m_DOT, m_FIVE_HEX, m_FOOTER_HEX};


            //Part 1 of resetting the stepping registers (register increments and decrements
            //as motor moves).
            int m_EP0_size = 6;//size of c style array
            const char m_EP0[6] = {m_HEADER_ONE, m_HEADER_TWO, m_E_HEX, m_P_HEX, m_ZERO_HEX, m_FOOTER_HEX};


            //Part 2 of resetting the stepping registers (register increments and decrements
            //as motor moves).
            int m_SP0_size = 6;//size of c style array
            const char m_SP0[6] = {m_HEADER_ONE, m_HEADER_TWO, m_S_HEX, m_P_HEX, m_ZERO_HEX, m_FOOTER_HEX};


            //Request current value of registers that keep track of motor motion. These values
            //increment and decrement based on motor direction and a set point. IP is Immediate Position
            int m_IP_size = 5;//size of c style array
            const char m_IP[5] = {m_HEADER_ONE, m_HEADER_TWO, m_I_HEX, m_P_HEX, m_FOOTER_HEX};


            //Request Status. The driver responds with all the letters that are applicable to
            //the current status:
            /*
                Status character codes:
                A = An Alarm code is present (use AL command to see code, AR command to clear code)
                D = Disabled (the drive is disabled)
                E = Drive Fault (drive must be reset by AR command to clear this fault)
                F = Motor moving (THis command is not used anymore > by Jim from AMP).
                H = Homing (SH in progress)
                J = Jogging (CJ in progress)
                M = Motion in progress (Feed & Jog Commands)
                P = In position
                R = Ready (Drive is enabled and ready)
                S = Stopping a motion (ST or SK command executing)
                T = Wait Time (WT command executing)
                W = Wait Input (WI command executing
            */
            int m_RS_size = 5;//size of c style array
            const char m_RS[5] = {m_HEADER_ONE, m_HEADER_TWO, m_R_HEX, m_S_HEX, m_FOOTER_HEX};

            int m_AL_size = 5;//size of c style array
            const char m_AL[5] = {m_HEADER_ONE, m_HEADER_TWO, m_A_HEX, m_L_HEX, m_FOOTER_HEX};

            //stop & kill buffer
            int m_SK_size = 5;
            const char m_SK[5] = {m_HEADER_ONE, m_HEADER_TWO, m_S_HEX, m_K_HEX, m_FOOTER_HEX};

            // turn power to motor off
            int m_MD_size = 5;//size of c style array
            const char m_MD[5] = {m_HEADER_ONE, m_HEADER_TWO, m_M_HEX, m_D_HEX, m_FOOTER_HEX};

            //Commence Jog - start jogging
            int m_CJ_size = 5;
            const char m_CJ[5] = {m_HEADER_ONE, m_HEADER_TWO, m_C_HEX, m_J_HEX, m_FOOTER_HEX};

            //stop & kill buffer
            int m_SJ_size = 5;
            const char m_SJ[5] = {m_HEADER_ONE, m_HEADER_TWO, m_S_HEX, m_J_HEX, m_FOOTER_HEX};

            int m_DI_NEG_size = 7;
            const char m_DI_NEG[7] = {m_HEADER_ONE, m_HEADER_TWO, m_D_HEX, m_I_HEX, m_MINUS_HEX, m_ONE_HEX, m_FOOTER_HEX};

            int m_DI_POS_size = 6;
            const char m_DI_POS[6] = {m_HEADER_ONE, m_HEADER_TWO, m_D_HEX, m_I_HEX, m_ONE_HEX, m_FOOTER_HEX};






            //the struct is defined in "/usr/include/netinet/in.h", and it uses a
            //data type called "in_addr_t" which is defined in "/usr/include/arpa/inet.h".
            //since arpa contains a "#include netinet/in.h" we can get away with using
            //only a "#include arpa/inet.h" header.
            //below I copied the implementation of the structure:

            //struct sockaddr_in
            //{
            //   short            sin_family;   // e.g. AF_INET (IPv4) or AF_INET6 (IPv6)
            //   unsigned short   sin_port;     // e.g. htons(3490). This is the port value via a method.
            //   struct in_addr   sin_addr;     // see struct in_addr, below. IP address
            //   char             sin_zero[8];  // zero this if you want to
            //};

            //struct in_addr
            //{
            //   unsigned long s_addr;          // load with inet_pton(). this is IP address
            //};
            struct sockaddr_in m_connectionToLeftStepperDrive;
            struct sockaddr_in m_connectionToRightStepperDrive;

            //the socket is represented as an int (for some reason)
            int m_socketRight;
            int m_socketLeft;

            void createSocketRight(); //initialize some aspect of socket
            void createSocketLeft();

            //sending commands to ST10 controllers
            bool sendRequest(const char *, int, int, std::string);

            //recieving responses from ST10 controllers
            std::string collectResponse(int);


            //this method will get the command from the controller as an arg, and
            //make sure the command does not attempt to get the ST10 drives to
            //perform an impossible move.
            bool verifyCommandFromController(double);

            //once the command has been verified good by the method above, I am storing it
            //here so I can tap into it later if needed.
            double m_controllerCommandInRadVerifiedGood;

            //TO DELETE**************************************************************************************
            //This is the size of the diff_drive_controller command (rad/sec for drive wheels).
            //It figures out how many "characters" there are in the diff_drive_controller command.
            //ex: -1.0024543 >>> I will use 7 characters (-1.0024)
            //ex2: 1.0024543 >>> I will use 6 characters  (1.0024)
            //ex3: 0.1234567 >>> I will use 6 characters  (0.1234)
            //TO DELETE*****************************************************************************************

            //Since the diff_drive_controller is giving us a radians/second for the
            //drive wheels, before I can begin to convert the double into an Applied
            //Motion Products command, I have to convert the radians/second for the
            //drive wheel into RPS (rotations per second) of the motor shaft!
            double convertControllerCommandToMotorRPS(double);

            //THis value holds the results after "m_controllerCommandInRadVerifiedGood" has been
            //verified as "good" and has been converter into Rotatinos per sec from Radians per sec.
            double m_goodControllerCommandInRps;


            //what is the most digits in a double I will ever get from the controller? there are
            //several options. some options are:
            //(1) max size of a double is 8 bytes (64 bits), which can generate a huge value!
            //(2) stack overflow tells me its 15 digits for a double.
            //I decided to go with 65. just cause I liked this value, and it is the size I made
            //for the character array which holds all the double digits (including the 'minus sign',
            //and the 'dot' for the digits less than 1) from the controller (the command).
            const static int m_sizeOfBuffer = 65;
            char m_bufferToHoldControllerCommandRightWheel [m_sizeOfBuffer];//an array
            char m_bufferToHoldControllerCommandLeftWheel [m_sizeOfBuffer];//an array

            //this method will convert the controller command into individual
            //numbers stored in an array. Ex: -1.987... will become:
            //array[0]='-';
            //array[1]='1';
            //array[2]='.';
            //array[3]='9';
            //array[4]='8';
            //array[5]='7';
            //...and so on
            void storeFinalCommandFromControllerAsMotorRpsRightWheel(double);
            void storeFinalCommandFromControllerAsMotorRpsLeftWheel(double);

            //When the controller gives us a positive or negative value it determins the direction
            //which the wheel needs to rotate in. However, if the robot is starting from stopped
            //the JS command does not take a negetive sign (CS does, but JS must be positive). So
            //for this reason if a negetive is present I will simply drop it and set one of these
            //booleans to accomodate the situation.
            bool m_controllerRequestsRightWheelBackward;
            bool m_controllerRequestsLeftWheelBackward;


            //before running any commands, we may need to configure the stepper
            //drives. Ex: JA-jog accel, JL-jog decel, JS-jog speed,
            //NOTE: all these commands should be prebuilt in this file, and const.
            void configureStepperDrives();

            //make sure wheels are configured successfully.
            bool m_bothDrivesConfiguredSuccessfullyOnInit;

            //send ME commands to both stepper drives to enable the motors to run.
            void enableStepperDrives();

            //confirm both drives were enabled on start up
            bool m_bothDrivesEnabledProperlyOnInit;

            //reset the step counting register. These registers keep track of the
            //number of steps from a specific set point. Meaning the value grows
            //and shrinks based on the direction the motor is rotating.
            void resetStepRegisters();

            //read the values of the step registers. These registers keep track of the
            //number of steps from a specific set point. Meaning the value grows
            //and shrinks based on the direction the motor is rotating.
            //NOTE: the values will be provided in HEX, as that is much faster.
            //      I will have to convert the hex to decimal.
            void getStatusOfStepRegisters();

            //we need to know that we were able to reset the register count of both
            //stepper drives when we reset the registers.
            bool m_bothDrivesResetRegisterCountOnInit;

            //method to convert the response from the IP command into an int. The
            //IP command outputs a  string with an embedded hex value. Ex: "IP=000fe12".
            int stringFromIpCommandToInt(std::string);

            //variables to hold the latest status of the step registers (see the above
            // 3 methods).
            int m_stepCountForRightStepperDrive;
            int m_stepCountForLeftStepperDrive;

            //*************************TESTING***************************
            //struct to hold status of a ST10 drive. Status character codes:
            //  A = An Alarm code is present (use AL command to see code, AR command to clear code)
            //  D = Disabled (the drive is disabled)
            //  E = Drive Fault (drive must be reset by AR command to clear this fault)
            //  F = Motor moving (NOT USED - I spoke to Jim from AMP) CORRECTION IT IS USED!!!
            //  H = Homing (SH in progress)
            //  J = Jogging (CJ in progress)
            //  M = Motion in progress (Feed & Jog Commands)
            //  P = In position
            //  R = Ready (Drive is enabled and ready)
            //  S = Stopping a motion (ST or SK command executing)
            //  T = Wait Time (WT command executing)
            //  W = Wait Input (WI command executing
            struct m_DriveStatus
            {
                bool m_st10InAlarm_A; //alarm on drive (motion possible)
                bool m_st10IsDisabled_D; //drive disabled
                bool m_st10IsFaulted_E; //drive in fault (no motion)
                bool m_st10MotorIsMoving_F; //motor is moving
                bool m_st10IsJogging_J; //is drive executing a CJ command?
                bool m_st10MotionCommandExecuting_M; //motor is in motion (feed & Jog commands)
                bool m_st10IsReadyToWork_R; //drive enabled and ready
            };

            //structs to hold the status of the right and left stepper drives.
            m_DriveStatus rightSt10Drive;
            m_DriveStatus leftSt10Drive;

            //get the status for the desired drive
            //arg 1 - struct holding all the status data for right or left ST10
            //arg 2 - the socket
            //arg 3 - a string that lets us know if the command is used for left or right drive
            void getFullStatusOfStepperDrive(struct m_DriveStatus&, int, std::string);


            //a value that makes sure that all the initialization steps during power up
            //were completed successfully. if This is false after the constructor runs,
            //then there is no need to try to run the robot. There is some problem that
            //must be addressed.
            bool m_initializationCompletedSuccessfullySystemIsReady;


            //based on the status of the robot, and the command from the controller
            //build a command to send to the drives.
            void buildCommandToSendToRightStepperDrive(double);
            void buildCommandToSendToLeftStepperDrive(double);


            //a method to enforce the policy on when the status of both ST10 drives is updated.
            void decideIfStatusUpdateForBothDrivesIsRequired();



		public:
			AmpSteppers();//constructor declaration
            ~AmpSteppers();//destructor declaration

            //standerdized methods that use all the small methods above with a core purpose of
            //recieveing a controller command, processing it (make sure it is withing speed bounds,
            //convert radians/sec of wheel to rotations per sec of motor), building a command
            //which the ST10 drives understand, and sendng it to the ST10 drive.
            void processControllerCommandAndSendToRightDrive(double);
            void processControllerCommandAndSendToLeftDrive(double);

            //this method sends a velocity command to both motors, and after
            //some delay (where the entire program is "sleeping", it sends a
            //zero velocity command. This is used stricly for testing.
            //arg 1 - desired velocity for right wheel in radians per second.
            //        (range is 1.2936 to -1.2936 radians per second)
            //arg 2 - desired velocity for left wheel in radians per second.
            //        (range is 1.2936 to -1.2936 radians per second)
            //arg 3 - time before stop command is automatically sent. the values
            //        need to be above 0 and are in microseconds. So a value of
            //        1,000,000 is 1 second. BTW long long has 64 bits.
            //NOTE: the reason I had to make a single method is because it is not
            //      possible otherwise - as one wheel sleeps, the other cannot
            //      recieve a run command! The sleeping aspect makes this method
            //      a serial process.
            void timedRightAndLeftWheelTesting(double, double, long long);


	}; //end of class


}//end of namespace

#endif // AMPSTEPPERS_H

