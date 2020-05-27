/*
    Purpose: the "ampsteppers" (.cpp & .h) files are a means for the NUC (riding on the
             robot) to control the Applied Motion Product ST10 stepper drives which drive
             the two drive wheels (via Ethernet). This entire package is built to recieve commands from
             the diff_drive_controller (which provides commands in terms of
             radians/second. See: https://answers.ros.org/question/311686/which-units-does-a-diff_drive_controller-output/)
             This package has to be able to provide the following functionality:
             (1) configure the ST10 drives
             (2) Verify on an ongoing basis that the ST10 drives are not faulted
             (3) Confirm that the diff_drive is sending commands that are within
                 the ability of the ST10 drives to carry out (ex: speed not excessive)
             (4) convert the command from the diff_drive (in Radians per sec for the
                 actual wheel) to rotations per sec of the motor; and build a command
                 that can be sent to the ST10s.
             (5) provide public methods/functions that can be dropped into ros_controller's
                 read and write to keep that code as simple as possible.

    Author - admin@TorontoRoboticsClub.com (MO)

    Date - December 23, 2018 (to January 21, 2019)

    version - 1.00 (this version is first time I attempt to control the AMP drives via
              C++).

    Future Improvements - some issues I know need to be addressed are as follows:
             (1) If the code is killed while the motor is running, it will keep running!
                 I need to figure out how to kill the motors if the code dies.
             (2) for the CS and CJ commands, collect the responses and make sure the
                 drive has recieved commands and all is well.
             (3) get away from the very nice (and expensive) applied motion drives.
*/




#include "jimmycpp/ampsteppers.h"
//#include <iostream> //for std::cout, std::endl, and std::string
//#include <arpa/inet.h> //inet_addr
#include <cstring> // for strlen (never use string.h. See https://stackoverflow.com/questions/9257665/difference-between-string-and-string-h#9257719
#include <string> //for std::stoi
//#include <vector>
#include <unistd.h> //for usleep(microseconds)
//#include <chrono> //timing libarary in std
#include <netinet/tcp.h>

namespace jimmycpp
{
    //! TO DO: need to figure out how to put this text into the header file
    Logging m_loggingObj {"ampsteppers_log.txt", "SCREEN_AND_FILE", "DEBUG"};

	//this is a definition of the class constructor outside of the class. In
   //this case we have ClassName::Constructor() to be able to define the item.
	AmpSteppers::AmpSteppers()
	{
        //initialize some bools used for logging:
        m_leftSocketConnectionFailure = false;
        m_rightSocketConnectionFailure = false;

	    //arg 1 - name of file to log to (must be unique per logger object)
        //arg 2 - this dictates where the log entry is sent to. The 3 options are:
        //        SCREEN - log entry is sent only to the screen
        //        FILE - log entry is sent only to a file
        //        SCREEN_AND_FILE - send to screen and file. if the second and third arguments are
        //                          omitted this is automatically set as the default.
        //arg 3 - We are able to filter the severity of message that is shown on the screen. The
        //        severity level passed as an arg (an all other more severe messages) will get printed
        //        to the screen, but lower severity is not printed to the screen. NOTE: the file is
        //        logging every severity level. The levels from least sever to most sever are:
        //        DEBUG, INFO, WARN, ERROR, FATAL.
        //loggingObj = Logging {"ampsteppers_log.txt", "SCREEN_AND_FILE", "DEBUG"};
        //logging object handle
        //!TO DO - FIGURE OUT HOW TO PUT HANDLE IN HEADER FILE, and init object here.


        //generate a line in the log.
        //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
        //arg 2 - line number in source file
        //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
        //arg 4 - message for user
        m_loggingObj.buildLogMessage("INFO", 77, "INITIALIZATION", "Logging enabled");

        //set the time value which determins the time out for the recv() function
        structToSetRecvTimeOut.tv_sec = 0.75; //timeout for recv() in seconds.
        structToSetRecvTimeOut.tv_usec = 0; //not used but must be initialized.


        //create a socket (more like define that I am using IP, TCP, and IP version 4)
        createSocketRight();
        createSocketLeft();

        //this code leverages the sockaddr_in structure, where I assign
        //values to a few of the structure's elements. This allows us to communicate with the
        //ST10 drives.
        //7775 is the udp port; 7776 is the TCP port
        m_connectionToLeftStepperDrive.sin_addr.s_addr = inet_addr("192.168.0.70");//Left ST10 IP
        m_connectionToLeftStepperDrive.sin_family = AF_INET; //type of IP addres. where AF_INET is IPv4
        m_connectionToLeftStepperDrive.sin_port = htons(7776); //ST10 TCP port. See appendix G in command manual

        m_connectionToRightStepperDrive.sin_addr.s_addr = inet_addr("192.168.0.60");//right ST10 IP
        m_connectionToRightStepperDrive.sin_family = AF_INET; //type of IP addres. where AF_INET is IPv4
        m_connectionToRightStepperDrive.sin_port = htons(7776); //ST10 TCP port. See appendix G in command manual


        //create a TCP connection to left ST10
        if (connect(m_socketLeft, (struct sockaddr *)&m_connectionToLeftStepperDrive, sizeof(m_connectionToLeftStepperDrive)) < 0)
        {
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("ERROR", 104, "INIT-NETWORKING", "A TCP connection to the left ST10 has failed to establish.");

            m_leftSocketConnectionFailure = true;
        }

        if(m_leftSocketConnectionFailure == false)
        {
            //generate a line in the log
            m_loggingObj.buildLogMessage("DEBUG", 107, "INIT-NETWORKING", "A TCP connection to the left ST10 has established.");
        }


        //create a TCP connection to right ST10
        if (connect(m_socketRight , (struct sockaddr *)&m_connectionToRightStepperDrive , sizeof(m_connectionToRightStepperDrive)) < 0)
        {
            //generate an error in the log file
            m_loggingObj.buildLogMessage("ERROR", 114, "INIT-NETWORKING", "A TCP connection to the right ST10 has failed to establish.");

            m_rightSocketConnectionFailure = true;
        }

        if(m_rightSocketConnectionFailure == false)
        {
            //generate a line in the log
            m_loggingObj.buildLogMessage("DEBUG", 118, "INIT-NETWORKING", "A TCP connection to the right ST10 has established.");
        }

        //a value that checks that all the initialization steps completed correcly
        m_initializationCompletedSuccessfullySystemIsReady = false;

	    //configure both stepper drives (accel, decel, and running speed)
        configureStepperDrives();

        //Initialize class member variable that tracks if step registers were set to
        //0 on system start up
        m_bothDrivesResetRegisterCountOnInit = false;

        //the stepper drives have a register that keeps the status of how many
        //steps the motors have moved relative to a fixed point (number increments
        //as there is forward motion, and decrements on reverse motion). This
        //method resets this register for both drives
        resetStepRegisters();

        //update the current value of the step registers of both drives (at this point
        // the registers on both drives should be ZERO).
        getStatusOfStepRegisters();

        //check that zeroing the step registers on both ST drives was successful
        if(m_stepCountForLeftStepperDrive==0 && m_stepCountForRightStepperDrive==0)
        {
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("INFO", 148, "INIT-ST10 SETUP", "Both ST10 drives have had the step registers reset to 0");
            m_bothDrivesResetRegisterCountOnInit = true;
        }


        //Before we start to do any work, we want to enable the drives (ME command)
        enableStepperDrives();

        //Get the status of both stepper drivers. Before running I need to have
        //knowledge of what is going on.
        getFullStatusOfStepperDrive(rightSt10Drive, m_socketRight, "Right");
        getFullStatusOfStepperDrive(leftSt10Drive, m_socketLeft, "Left");

        //only start working if initialization during start up executed correctly. Meaning
        //accel & decel (JA which also sets JL), Jog speed (JS), setting of the step
        //registers to 0 (EP0 & SP0), enabling the motors (ME) for both drive wheels was a
        //success. all of this is also verified by the RS (request status command). Basically
        //I am making sure all the commands I sent were answered with an ACK (% char).
        if(m_bothDrivesConfiguredSuccessfullyOnInit==true && m_bothDrivesResetRegisterCountOnInit==true && m_bothDrivesEnabledProperlyOnInit==true)
        {
            //so far it looks like all is OK. Now I am checking if the status of the drive
            //is ready (status code 'R')
            if(rightSt10Drive.m_st10IsReadyToWork_R==true && leftSt10Drive.m_st10IsReadyToWork_R==true)
            {
                m_initializationCompletedSuccessfullySystemIsReady = true;
            }
        }


        //initialize some more values
        m_controllerRequestsRightWheelBackward = false;
        m_controllerRequestsLeftWheelBackward = false;


        //close the connection
        close(m_socketLeft);
        close(m_socketRight);

        //in the future thsi will be a boolean, so that I can keep creating
        //this object in a loop until this bool is reporting success
        //generate a line in the log.
        //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
        //arg 2 - line number in source file
        //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
        //arg 4 - message for user
        m_loggingObj.buildLogMessage("INFO", 198, "INITIALIZATION", "--------------------------------------------------------------------------");
        m_loggingObj.buildLogMessage("INFO", 198, "INITIALIZATION", "Initialization has completed successfully - constructor executed correctly");
        m_loggingObj.buildLogMessage("INFO", 198, "INITIALIZATION", "--------------------------------------------------------------------------");
	}//end of constructor

	//destructor
	AmpSteppers::~AmpSteppers()
	{
	    //If the code is killed while the motor is running, it will keep running!
	    //I need to figure out how to kill the motors if the code dies.
	}

	//this class method generates a socket (which for some reason is stored as an int), and
	//stores it in a member variable.
	void AmpSteppers::createSocketRight()
	{
	    //initializing the socket with the socket() function.
        //arg 1 - Address Family - AF_INET (this is IP version 4)
        //arg 2 - Type - SOCK_STREAM (this means connection oriented TCP protocol)
        //        SOCK_DGRAM indicates the UDP protocol.
        //arg 3 - Protocol - 0 [ or IPPROTO_IP This is IP protocol]
        m_socketRight = socket(AF_INET , SOCK_STREAM , 0);

        //make sure the socket we got is OK
        if (m_socketRight == -1)
        {
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("ERROR", 228, "NETWORKING", "Socket for the RIGHT ST10 drive has failed to be created.");

            //if the socket failed then setting a recv timeout and the TCP_NODELAY is useless.
            return;
        }

        //set up a timeout value for the recv() method. structToSetRecvTimeOut is defined in the header file, and the values
        //are initialized in the constructor. For more information about what is going here, have a look here:
        //https://stackoverflow.com/questions/2876024/linux-is-there-a-read-or-recv-from-socket-with-timeout/2939145#2939145
        setsockopt(m_socketRight, SOL_SOCKET, SO_RCVTIMEO, (const char*)&structToSetRecvTimeOut, sizeof structToSetRecvTimeOut);

        //eliminate the PSH flag that is turned on automatically
        //arg 1 - the socket
        //arg 2 - level. Basiclly at what location in the stack do I want to set the option. I opted for the TCP level. See:
        //        https://pubs.opengroup.org/onlinepubs/009695399/functions/setsockopt.html
        //arg 3 - the socket option to set.
        //arg 4 - pointer to the variable holding the setting. in this case 1 means we have NODEALY turned on, and 0 means it's off
        int noDelayState = 1; //0 means prefer bandwidth optimization over low latency
        //arg 4 - length of the setting
        int tcpPushOptionOff = setsockopt(m_socketRight, IPPROTO_TCP, TCP_NODELAY,(char *) &noDelayState, sizeof(int));

        //if we were not able to turn off the push flag, log it.
        if(tcpPushOptionOff < 0)
        {
            m_loggingObj.buildLogMessage("ERROR", 269, "NETWORKING", "Failed to turn off the TCP PSH flag in RIGHT socket.");

        }
	}

		//this class method generates a socket (which for some reason is stored as an int), and
	//stores it in a member variable.
	void AmpSteppers::createSocketLeft()
	{
	    //initializing the socket with the socket() function.
        //arg 1 - Address Family - AF_INET (this is IP version 4)
        //arg 2 - Type - SOCK_STREAM (this means connection oriented TCP protocol)
        //        SOCK_DGRAM indicates the UDP protocol.
        //arg 3 - Protocol - 0 [ or IPPROTO_IP This is IP protocol]
        m_socketLeft = socket(AF_INET , SOCK_STREAM , 0);

        //make sure the socket we got is OK
        if (m_socketLeft == -1)
        {
            m_loggingObj.buildLogMessage("ERROR", 246, "NETWORKING", "Socket for the LEFT ST10 drive has failed to be created.");
                    //if the socket failed then setting a recv timeout and the TCP_NODELAY is useless.
            return;
        }

        //set up a timeout value for the recv() method. structToSetRecvTimeOut is defined in the header file, and the values
        //are initialized in the constructor. For more information about what is going here, have a look here:
        //https://stackoverflow.com/questions/2876024/linux-is-there-a-read-or-recv-from-socket-with-timeout/2939145#2939145
        setsockopt(m_socketLeft, SOL_SOCKET, SO_RCVTIMEO, (const char*)&structToSetRecvTimeOut, sizeof structToSetRecvTimeOut);


        //eliminate the PSH flag that is turned on automatically
        //arg 1 - the socket
        //arg 2 - level. Basiclly at what location in the stack do I want to set the option. I opted for the TCP level. See:
        //        https://pubs.opengroup.org/onlinepubs/009695399/functions/setsockopt.html
        //arg 3 - the socket option to set.
        //arg 4 - pointer to the variable holding the setting. in this case 1 means we have NODEALY turned on, and 0 means it's off
        int noDelayState = 1; //0 means prefer bandwidth optimization over low latency
        //arg 4 - length of the setting
        int tcpPushOptionOff = setsockopt(m_socketLeft, IPPROTO_TCP, TCP_NODELAY,(char *) &noDelayState, sizeof(int));

        //if we were not able to turn off the push flag, log it.
        if(tcpPushOptionOff < 0)
        {
            m_loggingObj.buildLogMessage("ERROR", 316, "NETWORKING", "Failed to turn off the TCP PSH flag in LEFT socket.");

        }
	}


	// The send method sends data out via a char array. Basically the contents of a char array
    // is the data that is sent out by the send method. The sizeOfMessage is needed as the send
    //method requires this info, and there is no way to figure this out inside the method (as the
    // c style array decays into a pointer, and getting the size of a pointer does not in any way
    // tell us anything about the size of the array! See:
    //https://stackoverflow.com/questions/4108313/how-do-i-find-the-length-of-an-array
	bool AmpSteppers::sendRequest(const char *pointerToMessageAsCharArray, int sizeOfMessage, int socket, std::string driveSide)
	{
	    //a bool to keep track if there was a failure to send message
	    bool sendFailure = false;

	    //!DELETE FOR IMPROVED PERFORMANCE>>>TESTING ONLY<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	    //initialize a string
        std::string messageSentAsString = "";

        //examine each message we are sending out.
        for(int i = 0; i < sizeOfMessage; i++)
        {
            //grab the first header (non printable ascii)
            if(*&pointerToMessageAsCharArray[i]== 0x00)
            {
                //std::cout<< "H1" << std::endl;
                messageSentAsString.append("H1");
                continue;
            }

            //grab the second header (non printable ascii)
            if(*&pointerToMessageAsCharArray[i]== 0x07)
            {
                //std::cout<< "H2" << std::endl;
                messageSentAsString.append("H2_");
                continue;
            }

            //grab the footer (non printable ascii)
            if(*&pointerToMessageAsCharArray[i]== 0x0D)
            {
                //std::cout<< "EOL" << std::endl;
                messageSentAsString.append("_EOL");
                continue;
            }

            //std::cout << *&cpt[i] << std::endl;
            //grow the string
            messageSentAsString.push_back(*&pointerToMessageAsCharArray[i]);
        }

        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!DELETE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	    //send the message
	    int sendStatusCode = send(socket , *&pointerToMessageAsCharArray , sizeOfMessage , 0);

	    if( sendStatusCode < 0)
        {
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("ERROR", 330, "NETWORKING", "The send function has failed. No message was sent! The failed command was: " + messageSentAsString
                                         + "; to the " + driveSide + " drive. The error code is: " + std::to_string(sendStatusCode));
            sendFailure = true;
        }

        if(sendFailure == false)
        {
            m_loggingObj.buildLogMessage("INFO", 337, "NETWORKING", "Send method successfully sent: " + messageSentAsString + " to " + driveSide + " drive.");
        }

        //This is needed becasue in the absens of a 100 microsecond nap, the messages are
        //all packed into a single ethernet frame, and I begin to have issues. Figured out
        //using wireshark. See:
        //https://stackoverflow.com/questions/158585/how-do-you-add-a-timed-delay-to-a-c-program/25807983#25807983
        usleep(100);

        return sendStatusCode;


	}//sendRequest method


    //After sending message to an ST10, it will send a response. This
    //method collects the entire response and stores it in m_st10Response.
	std::string AmpSteppers::collectResponse(int socket)
	{
        //initialize the string
        std::string st10Response = "";


        //size of temp buffer
        const int bufferSize = 20;

        //create a temp array. This will hold a single line recieved from
        //the ST10. Array is initialized with NULs (which makes making a
        //string out of a char array with nuls a pain in the ass!!). AT no
        //point do I anticipate getting more than 40 chars per response.
        char tempBuffer [bufferSize] = {};

        //fill array with money. this special character will allow us to know
        //the location where the d*/ //end of TESTINGata we wrote in meets the original '$' contents.
        std::fill(&tempBuffer[0], &tempBuffer[bufferSize-1], '$');


        //unlike the arduino the message we get is one single message, and we cannot
        //get only a part of a message. So for now i will still keep the loop, but I
        //will run the loop only once (as if there is no loop).
        int numberOfTimesRecvRuns = 1;


        //recieve the entire ST10 response. The magic number is the number of times
        // we call the recv method (which reads a line from the socket).
        for(int i = 0; i < numberOfTimesRecvRuns; i++)
        {
            //call the recv method over and over as it gets a single ST10 line with
            //every iteration. The data is written into the tempBuffer array which has
            //been pre-filled with $ characters.
            if( recv(socket, tempBuffer , bufferSize , 0) < 0)
            {
                //generate a line in the log.
                //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
                //arg 2 - line number in source file
                //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
                //arg 4 - message for user
                m_loggingObj.buildLogMessage("ERROR", 333, "NETWORKING", "The receive (recv) function has failed");
            }

            //process the response
            for(int j = 0; j < bufferSize; j++)
            {
                //if we extracted all the info, break out of the inner loop.
                if(tempBuffer[j] == '$')
                {
                    //wipe the buffer holding the data recieved. This was important if we
                    //loop more than once... but we dont...
                    std::fill(&tempBuffer[0], &tempBuffer[bufferSize-1], '$');

                    //no need to run anymore as we hit the end.
                    break;
                }

                //if it's the header or footer, don't write it to the string
                if(tempBuffer[j]==0x00 || tempBuffer[j]==0x07 || tempBuffer[j]==0x0d )
                {
                    //std::cout << "saw header or footer" << std::endl;
                    //continue will skip to the next iteration of the loop
                    continue;
                }

                st10Response = st10Response+tempBuffer[j];
            }//inner loop processing response
        }//outer for loop running multiple times in case response is not recieved by single use of recv

        //std::cout << "response recieved is: " << st10Response << std::endl;

        return st10Response;
	}


    //this method configures the stepper drive to have the correct acceleration,
	//deceleration, starting speed, and enabling the motor.
	//NOTE: as per the manual when you set the JA, you also set the JL!!! (see pg 148)
	void AmpSteppers::configureStepperDrives()
	{
	    //*******************set accel & decel for drives*******************************

        //class member veriable to check if configuring the step drives was successful
        m_bothDrivesConfiguredSuccessfullyOnInit = false;

        //local variables to check the success of each configuration
        bool jaOnRightSuccess = false;
        bool jaOnLeftSuccess = false;
        bool jsOnRightSuccess = false;
        bool jsOnLeftSuccess = false;
        bool diOnRightSuccess = false;
        bool diOnLeftSuccess = false;

	    //configure the Jog Acceleration to 5 RPS per second on RIGHT drive.
	    sendRequest(&m_JA5[0], m_JA5_size, m_socketRight, "RIGHT");
	    std::string setJaOnRightDrive = collectResponse(m_socketRight);
	    if(setJaOnRightDrive == "%")
        {
            jaOnRightSuccess = true;
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("DEBUG", 397, "ST10 CONFIG", "JA5 to RIGHT drive was successful.");
        }


        //configure the Jog Acceleration to 5 RPS per second on LEFT drive.
	    sendRequest(&m_JA5[0], m_JA5_size, m_socketLeft, "LEFT");
        std::string setJaOnLeftDrive = collectResponse(m_socketLeft);
	    if(setJaOnLeftDrive == "%")
        {
            jaOnLeftSuccess = true;
            m_loggingObj.buildLogMessage("DEBUG", 407, "ST10 CONFIG", "JA5 to LEFT drive was successful.");
        }

        //********************set running speed for drives*******************************
        //configure the initial Jog speed to 0.5 RPS per second on RIGHT drive (motor shaft speed).
	    sendRequest(&m_JS0dot5[0], m_JS0dot5_size, m_socketRight, "RIGHT");
	    std::string setJsOnRightDrive = collectResponse(m_socketRight);
	    if(setJsOnRightDrive == "%")
        {
            jsOnRightSuccess = true;
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("DEBUG", 422, "ST10 CONFIG", "JS0.5 to RIGHT drive was successful.");
        }


        //configure the initial Jog speed to 0.5 RPS per second on LEFT drive (motor shaft speed).
	    sendRequest(&m_JS0dot5[0], m_JS0dot5_size, m_socketLeft, "LEFT");
	    std::string setJsOnLeftDrive = collectResponse(m_socketLeft);
	    if(setJsOnLeftDrive == "%")
        {
            jsOnLeftSuccess = true;
            m_loggingObj.buildLogMessage("DEBUG", 432, "ST10 CONFIG", "JS0.5 to LEFT drive was successful.");
        }

        //********************set DI for drives (sets direction until CS is called) ***********************
        //configure the DI on right drive to 1. This determins direction when we start jogging.
        //At this setting a positive CS command will drive the robot forward, and negetive command will
        //drive the robot backwards.
	    sendRequest(&m_DI_POS[0], m_DI_POS_size, m_socketRight, "RIGHT");
	    std::string setDiOnRightDrive = collectResponse(m_socketRight);
	    if(setDiOnRightDrive == "%")
        {
            diOnRightSuccess = true;
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("DEBUG", 449, "ST10 CONFIG", "DI1 to RIGHT drive was successful.");
        }


        //configure the DI on left drive to 1. This determins direction when we start jogging.
        //At this setting a positive CS command will drive the robot forward, and negetive command will
        //drive the robot backwards.
	    sendRequest(&m_DI_POS[0], m_DI_POS_size, m_socketLeft, "LEFT");
	    std::string setDiOnLeftDrive = collectResponse(m_socketLeft);
	    if(setDiOnLeftDrive == "%")
        {
            diOnLeftSuccess = true;
            m_loggingObj.buildLogMessage("DEBUG", 461, "ST10 CONFIG", "DI1 to LEFT drive was successful.");
        }

        //check if all four configuration commands were acked by drive
        if(jaOnRightSuccess==true && jaOnLeftSuccess==true && jsOnRightSuccess==true && jsOnLeftSuccess==true
           && diOnRightSuccess==true && diOnLeftSuccess==true)
        {
            //configuratoin was succssful
            m_bothDrivesConfiguredSuccessfullyOnInit = true;
        }

	}


	//reset the step counting register. These registers keep track of the
    //number of steps from a specific set point. Meaning the value grows
    //and shrinks based on the direction the motor is rotating.
	void AmpSteppers::resetStepRegisters()
	{
        //*********************PART 1 of 2 for resetting registers (EP0)**************************
        //send EP0 command to right drive and get response.
        sendRequest(&m_EP0[0], m_EP0_size, m_socketRight, "RIGHT");
        std::string setEpOnRightDrive = collectResponse(m_socketRight);
	    if(setEpOnRightDrive == "%")
        {
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("DEBUG", 491, "ST10 CONFIG", "EP0 to RIGHT drive was successful.");
        }

        //send EP0 command to left drive and get response.
        //NOTE: no response from drive to EP0 command. See pg 24 in command ref manual
	    sendRequest(&m_EP0[0], m_EP0_size, m_socketLeft, "LEFT");
	    std::string setEpOnLeftDrive = collectResponse(m_socketLeft);
	    if(setEpOnLeftDrive == "%")
        {
            m_loggingObj.buildLogMessage("DEBUG", 499, "ST10 CONFIG", "EP0 to LEFT drive was successful.");
        }


        //*********************PART 2 of 2 for resetting registers (SP0)**************************
        //send SP0 command to right drive and get response.
        //NOTE: no response from drive to SP0 command. See pg 24 in command ref manual
        sendRequest(&m_SP0[0], m_SP0_size, m_socketRight, "RIGHT");
        std::string setSpOnRightDrive = collectResponse(m_socketRight);
	    if(setSpOnRightDrive == "%")
        {
            m_loggingObj.buildLogMessage("DEBUG", 511, "ST10 CONFIG", "SP0 to RIGHT drive was successful.");
        }

        //send SP0 command to left drive and get response.
        //NOTE: no response from drive to SP0 command. See pg 24 in command ref manual
	    sendRequest(&m_SP0[0], m_SP0_size, m_socketLeft, "LEFT");
	    std::string setSpOnLeftDrive = collectResponse(m_socketLeft);
	    if(setSpOnLeftDrive == "%")
        {
            m_loggingObj.buildLogMessage("DEBUG", 520, "ST10 CONFIG", "SP0 to LEFT drive was successful.");
        }

        //initialize the variables holding the latest step registers values
        m_stepCountForLeftStepperDrive = 0;
        m_stepCountForRightStepperDrive = 0;
	}


    //get the current value of the step counting register. These registers keep
	//track of the number of steps from a specific set point. Meaning the value grows
    //and shrinks based on the direction the motor is rotating.
    //NOTE: the values will be provided in HEX, as that is much faster.
    //      I will have to convert the hex to decimal.
    //NOTE2: the command is IP (immediate position).
    void AmpSteppers::getStatusOfStepRegisters()
    {
        //send IP command to right drive and get response.
        sendRequest(&m_IP[0], m_IP_size, m_socketRight, "RIGHT");
        //note: this value is in HEX!! Format examples:
        //Ex 1: "IP=00002710" == Absolute position is 10,000 counts (or steps)
        //Ex 2: "IP=FFFFD8F0" == Absolute position is -10,000 counts (or steps)
	    std::string getIpOnRightDrive = collectResponse(m_socketRight);
	    //generate a line in the log.
        //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
        //arg 2 - line number in source file
        //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
        //arg 4 - message for user
        m_loggingObj.buildLogMessage("INFO", 548, "ST10 STATUS", "IP from RIGHT drive: " + getIpOnRightDrive);

        //send IP command to left drive and get response.
	    sendRequest(&m_IP[0], m_IP_size, m_socketLeft, "LEFT");
	    //note: this value is in HEX!!
	    std::string getIpOnLeftDrive = collectResponse(m_socketLeft);
	    m_loggingObj.buildLogMessage("INFO", 554, "ST10 STATUS", "IP from LEFT drive: " + getIpOnLeftDrive);

        //Process string with hex value into a decimal value and store in a variable.
        //first drop the "IP=", and second convert the hex string into a decimal int.
        //Well the int is not decimal, but it displays as such, you get it...
        m_stepCountForLeftStepperDrive = stringFromIpCommandToInt(getIpOnLeftDrive);
        m_stepCountForRightStepperDrive = stringFromIpCommandToInt(getIpOnRightDrive);
    }


    //this method is manipulating, and converting a hex value from a
    //string into a signed long (the data type large enough to hold the value I
    //am getting from the drive). The IP (Immediate Position) command is reporting
    //the status of the steps since the last reset.
    //for String to long see: https://stackoverflow.com/questions/1070497/c-convert-hex-string-to-signed-integer
    //for long size see: https://stackoverflow.com/questions/589575/what-does-the-c-standard-state-the-size-of-int-long-type-to-be
    int AmpSteppers::stringFromIpCommandToInt(std::string inputString)
    {
        //IP command responses have the following format:
        //EX1: "IP=00002710" == Absolute position is 10,000 counts (or steps)
        //EX2: "IP=FFFFD8F0" == Absolute position is -10,000 counts (or steps)
        //this method drops the first 3 characters in the string ("IP=")
        std::string intValueAsString = inputString.substr(3, inputString.size());

        //convert string to long. See:
        //https://stackoverflow.com/questions/1070497/c-convert-hex-string-to-signed-integer
        int numberOfStepsAsInt = std::stoul(intValueAsString, nullptr, 16);

        return numberOfStepsAsInt;
    }


	//This send the ME (motor enable) command to both drives
	void AmpSteppers::enableStepperDrives()
	{
        //**********************Enable both drives to run*********************************

        //initialize variable that makes sure both drives are enabled on system start up
        m_bothDrivesEnabledProperlyOnInit = false;

        bool rightDriveEnabled = false;
        bool leftDriveEnabled = false;
        //Send ME command to right drive
        //NOTE: no response from drive to ME command. See pg 24 in command ref manual
	    sendRequest(&m_ME[0], m_ME_size, m_socketRight, "RIGHT");
	    std::string setMeOnRightDrive = collectResponse(m_socketRight);
	    if(setMeOnRightDrive == "%")
        {
            rightDriveEnabled = true;
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("INFO", 608, "ST10 STATUS", "ME to RIGHT drive was successful.");
        }

        //Send ME command to right drive
	    sendRequest(&m_ME[0], m_ME_size, m_socketLeft, "LEFT");
	    std::string setMeOnLeftDrive = collectResponse(m_socketLeft);
	    if(setMeOnLeftDrive == "%")
        {
            leftDriveEnabled = true;
            m_loggingObj.buildLogMessage("INFO", 617, "ST10 STATUS", "ME to LEFT drive was successful.");
        }

        //both drives recieved ME command and will most likely enable the motors.
        if(rightDriveEnabled==true && leftDriveEnabled==true)
        {
            m_bothDrivesEnabledProperlyOnInit = true;
        }
	}



    //**************************************************************************
    //this method performs a case insensitive comparison of strings. It came from:
    //https://stackoverflow.com/questions/11635/case-insensitive-string-comparison-in-c
    bool caseInsensitiveStringCompare(const std::string& a, const std::string& b)
    {
        //get the size of the first argument
        unsigned int argOneSize = a.size();

        //if the arguments are not the same size, then the strings must not be equal
        if (b.size() != argOneSize)
        {
            return false;
        }

        //compare each character in a for loop
        for (unsigned int i = 0; i < argOneSize; ++i)
        {
            if (tolower(a[i]) != tolower(b[i]))
            {
                return false;
            }
        }

        //since we haven't exited yet, the strings must be the same.
        return true;
    }


    //this method handles getting the status of the ST drives. It does so one drive at a
    //time, and the drive is selected by passing in a string argument.
    void AmpSteppers::getFullStatusOfStepperDrive(struct m_DriveStatus& statusStruct, int socket, std::string rightOrLeft)
    {
        //variables to confirm proper string was give as third arg.
        bool thirdArgIsRight = false;
        bool thirdArgIsLeft = false;
        //if the last string is not a case insensitive "right" or "Left", then exit the method and give an error. no
        //point in continuing
        if(caseInsensitiveStringCompare("Right", rightOrLeft))
        {
            thirdArgIsRight = true;
        }

        if(caseInsensitiveStringCompare("Left", rightOrLeft))
        {
            thirdArgIsLeft = true;
        }

        if(thirdArgIsRight==false && thirdArgIsLeft==false)
        {
            m_loggingObj.buildLogMessage("DEBUG", 735, "SYNTAX ERROR", "arg 3 is expecting a case insensitive \"right\", or \"left\". Ex:\"RIGHT\", or \"Left\". But it got: " + rightOrLeft
                                          + ". Method is exiting in failure!!!");
            return;
        }


        //prepare structure to hold latest values by purging existing data.
        statusStruct.m_st10InAlarm_A = false;//alarm on drive (motion possible)
        statusStruct.m_st10IsDisabled_D = false; //drive disabled
        statusStruct.m_st10IsFaulted_E = false; //drive in fault (no motion)
        statusStruct.m_st10MotorIsMoving_F = false; //motor is moving
        statusStruct.m_st10IsJogging_J = false; //is drive executing a CJ command?
        statusStruct.m_st10MotionCommandExecuting_M = false; //motor is in motion (feed & Jog commands)
        statusStruct.m_st10IsReadyToWork_R = false; //drive enabled and ready


        //send RS command to right drive and get response.
        sendRequest(&m_RS[0], m_RS_size, socket, rightOrLeft);
	    std::string driveResponse = AmpSteppers::collectResponse(socket);

	    //generate a line in the log.
        //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
        //arg 2 - line number in source file
        //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
        //arg 4 - message for user
        m_loggingObj.buildLogMessage("INFO", 759, "ST10 STATUS", "Status of "+ rightOrLeft + " drive is: " + driveResponse);

        //if the drive glitched adn retuned %% instead of RS=ABC, prevent the next line from crashing the program.
        //NOTE: crash happens when I try to substr() from letter 3, but there are only 2 letters, so the program exits.
        if(driveResponse.size() < 4)
        {
            m_loggingObj.buildLogMessage("ERROR", 844, "ST10 STATUS", "THE " + rightOrLeft + " ST10 GAVE BAD RS response (>" +driveResponse + "<) instead of RS=ABC!   !");

            return;
        }

	    //the response will be something like: RS=ABC. This code drops the first
	    //3 characters (the "RS="")
	    std::string statusOfDrive = driveResponse.substr(3, driveResponse.size());


        //what codes are present and which are missing?	The details of the RS command:
        //  Status character codes:
        //  A = An Alarm code is present (use AL command to see code, AR command to clear code)
        //  D = Disabled (the drive is disabled)
        //  E = Drive Fault (drive must be reset by AR command to clear this fault)
        //  F = Motor moving (NOT USED - I spoke to Jim from AMP)
        //  H = Homing (SH in progress)
        //  J = Jogging (CJ in progress)
        //  M = Motion in progress (Feed & Jog Commands)
        //  P = In position
        //  R = Ready (Drive is enabled and ready)
        //  S = Stopping a motion (ST or SK command executing)
        //  T = Wait Time (WT command executing)
        //  W = Wait Input (WI command executing
        std::size_t alarmStatus_A = statusOfDrive.find("A");//is drive in alarm (motion possible)?
        std::size_t disabledStatus_D = statusOfDrive.find("D");//is right drive disabled?
        std::size_t faultStatus_E = statusOfDrive.find("E");//is drive faulted (no motion)?
        std::size_t motorIsMoving_F = statusOfDrive.find("F");//is motor in motion
        std::size_t jogStatus_J = statusOfDrive.find("J"); //is drive executing a CJ command?
        std::size_t motionStatus_M = statusOfDrive.find("M"); //is motor moving (feed & jog commands)?
        std::size_t driveReadyStatus_R = statusOfDrive.find("R");//is drive enabled and ready?

        //letter "A" exists - We have an alarm
        if(alarmStatus_A!=std::string::npos)
        {
            statusStruct.m_st10InAlarm_A = true;//alarm on drive (motion possible)
            m_loggingObj.buildLogMessage("INFO", 910, "ST10 STATUS", rightOrLeft + "ST10 is in ALARM!!! alarm code is: TO IMPLEMENT");
        }

        //letter "D" exists - the drive is disabled and cannot move
        if(disabledStatus_D!=std::string::npos)
        {
            statusStruct.m_st10IsDisabled_D = true; //drive disabled
            m_loggingObj.buildLogMessage("ERROR", 917, "ST10 STATUS", rightOrLeft + "ST10 is DISABLED and cannot move!!!");
        }

        //letter "E" exists - THe drive is in fault and cannot move
        if(faultStatus_E!=std::string::npos)
        {
            statusStruct.m_st10IsFaulted_E = true; //drive in fault (no motion)
            m_loggingObj.buildLogMessage("ERROR", 924, "ST10 STATUS", rightOrLeft + "ST10 is in FAULT, and cannot move!!!");
        }

        //letter "F" exists - We are in motion
        if(motorIsMoving_F!=std::string::npos)
        {
            statusStruct.m_st10MotorIsMoving_F = true; //motor is moving
        }

        //letter "J" exists - We are jogging (a finer distiction of motion)
        if(jogStatus_J!=std::string::npos)
        {
            statusStruct.m_st10IsJogging_J = true; //is drive executing a CJ command?
        }

        //letter "M" exists - motor is in motion (either a jog or feed)
        if(motionStatus_M!=std::string::npos)
        {
            statusStruct.m_st10MotionCommandExecuting_M = true; //motor is in motion (feed & Jog commands)
        }

        //letter "R" exists - THe drive is ready to work
        if(driveReadyStatus_R!=std::string::npos)
        {
            statusStruct.m_st10IsReadyToWork_R = true; //drive enabled and ready
        }

        //check if we are in motion
        //m_loggingObj.buildLogMessage("DEBUG", 845, "TEST", "J: "+std::to_string(statusStruct.m_st10IsJogging_J));
        //m_loggingObj.buildLogMessage("DEBUG", 846, "TEST", "F: "+std::to_string(statusStruct.m_st10MotorIsMoving_F));
        //m_loggingObj.buildLogMessage("DEBUG", 847, "TEST", "M: "+std::to_string(statusStruct.m_st10MotionCommandExecuting_M));
    }


    //this method will convert a Radian per second (speed of wheel roatation)
    //to RPS (rotation per second of the stepper motor shaft).
	bool AmpSteppers::verifyCommandFromController(double commandFromController)
	{
	    //set the value at something I know:
        m_controllerCommandInRadVerifiedGood = 0.0;

        //Consult notes to see how I got the values. The speed of the stepper motor
        //shaft is set at a max of 0.5RPS. since the motor gear is 14 teeth, and the
        //gear on the wheel is 34 teeth, the max RPS of the wheel is 0.5 x (14/34) = 0.20588 RPS.
        //to get this value in radians per second rather than revolutions per minute,
        //simply muliply the max wheel RPS by the radian value of a full revolution:
        //.20588 x 6.283185 = 1.2936 radians per second (which is about 74.14 degrees).
        double topSpeedAllowedByDriveWheelInRadPerSec = (m_topSpeedOfStepperMotorInRPS * m_gearingRatio13over34) * m_oneRevInRad;

        //a boolean to determine if motion command (in rad/sec) from controller is a
        //value that will not cause a fault on the stepper drives (ex: a speed that will
        //cause the driver to fault).
        bool controllerCommandIsGood = false;

        //figure out if forward or reverse motion is excessive
        if((commandFromController > topSpeedAllowedByDriveWheelInRadPerSec)
        || ((commandFromController<0) && (commandFromController < (topSpeedAllowedByDriveWheelInRadPerSec*-1))))
        {
            //value offered by controller is too agressive for the ST10 to execute.
            controllerCommandIsGood = false;

            //NOTE: double to string is not "easy". See: https://stackoverflow.com/questions/332111/how-do-i-convert-a-double-into-a-string-in-c
            m_loggingObj.buildLogMessage("WARN", 850, "INPUT_COMMAND", "Controller command  to this ST10 lib is outside the operating range of the robot: "
                                         + std::to_string(commandFromController));
        }

        //explicitly figure out that the command is good
        if(((commandFromController > 0) && (commandFromController < topSpeedAllowedByDriveWheelInRadPerSec))
        || ((commandFromController < 0) && (commandFromController > topSpeedAllowedByDriveWheelInRadPerSec*-1))
        || commandFromController == 0.0)
        {
            //value offered by controller is good for the ST10 to execute.
            controllerCommandIsGood = true;

            //Since the command from the controller is good, I will store it.
            m_controllerCommandInRadVerifiedGood = commandFromController;

            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            //NOTE: double to string is not "easy". See: https://stackoverflow.com/questions/332111/how-do-i-convert-a-double-into-a-string-in-c
            m_loggingObj.buildLogMessage("INFO", 870, "INPUT_COMMAND", "Controller command  to this ST10 lib is good: "
                                         + std::to_string(commandFromController));

            //std::cout << "controller Command is good: " << commandFromController << std::endl;
        }

        return controllerCommandIsGood;
	}


    //Since the diff_drive_controller is giving us a radians/second for the
    //drive wheels, before I can begin to convert the double into an Applied
    //Motion Products command, I have to convert the radians/second for the
    //drive wheel into RPS (rotations per second) of the motor shaft!
    //NOTE: MAKE SURE YOU VERIFIED THE COMMAND FROM THE CONTROLLER IS GOOD!!
    double AmpSteppers::convertControllerCommandToMotorRPS (double commandFromController)
    {
        //convert wheel rad/sec (as given by controller) to wheel RPS
        double controllerCommandAsWheelRPS = commandFromController/m_oneRevInRad;

        //convert wheel RPS into motor RPS
        double controllerCommandAsMotorRPS = controllerCommandAsWheelRPS / m_gearingRatio13over34;

        //store the value in a member variable
        m_goodControllerCommandInRps = controllerCommandAsMotorRPS;

        return controllerCommandAsMotorRPS;
    }



    //this method stores the controller command for the right wheel in a format which
    //the ST10 drives understands. This value is what my code will send to the ST10 as it
    //has been converted to Motor RPS from drive wheel Rad/sec, and stored as chars in an array.
	void AmpSteppers::storeFinalCommandFromControllerAsMotorRpsRightWheel(double commandFromControllerInMotorRPS)
	{
	    //prefill the buffer with the same character. This will allow me to always know
	    //when the data I am storing has ended.
	    for(int i = 0; i < m_sizeOfBuffer; i++)
	    {
            m_bufferToHoldControllerCommandRightWheel[i] = '$';
	    }

        //generate a line in the log.
        //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
        //arg 2 - line number in source file
        //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
        //arg 4 - message for user
        m_loggingObj.buildLogMessage("INFO", 919, "ST10 COMMAND", "The rotations per second value to send to RIGHT ST10 is: " + std::to_string(commandFromControllerInMotorRPS));


	    //This function converts all the digits in a double to a 'char' data type and pushes
	    //each digit into an array. Basically, what happens here is, c++ will round the
	    //command I get from the controller, and the results of that will be put into the
	    //buffer I have created. The digits of the double will overwrite the '$' chars.
	    sprintf (m_bufferToHoldControllerCommandRightWheel, "%f", commandFromControllerInMotorRPS);
	}


    //this method stores the controller command for the left wheel in a format which
    //the ST10 drives understands. This value is what my code will send to the ST10 as it
    //has been converted to Motor RPS from drive wheel Rad/sec, and stored as chars in an array.
	void AmpSteppers::storeFinalCommandFromControllerAsMotorRpsLeftWheel(double commandFromControllerInMotorRPS)
	{
        //generate a line in the log.
        //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
        //arg 2 - line number in source file
        //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
        //arg 4 - message for user
        m_loggingObj.buildLogMessage("INFO", 1001, "ST10 COMMAND", "The rotations per second pre sign invert for LEFT ST10 is: " + std::to_string(commandFromControllerInMotorRPS));

	    //prefill the buffer with the same character. This will allow me to always know
	    //when the data I am storing has ended.
	    for(int i = 0; i < m_sizeOfBuffer; i++)
	    {
            m_bufferToHoldControllerCommandLeftWheel[i] = '$';
	    }


	    //This function converts all the digits in a double to a 'char' data type and pushes
	    //each digit into an array. Basically, what happens here is, c++ will round the
	    //command I get from the controller, and the results of that will be put into the
	    //buffer I have created. The digits of the double will overwrite the '$' chars.
	    sprintf (m_bufferToHoldControllerCommandLeftWheel, "%f", commandFromControllerInMotorRPS);


        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!KEY ALGORITHM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //the left wheel is rotating in the opposite direction than the right wheel because it
        //is rotated by 180 degrees. Instead of doing fancy footwork in the command building
        //i decided the easiest thing to do is to inverte the signage on the command we get
        //from the controller. If command is positive, we make it negative. If command is
        //negative, we make it positive. This will allow me to use the same exact code for the
        //command construction methods in both right and left.


        //keep track of the signage of the value (is it positive or negetive?)
        bool controllerValueIsNegetive = false;

        //check if value is negative
        if(m_bufferToHoldControllerCommandLeftWheel[0] == '-')
        {
            controllerValueIsNegetive = true;
        }

        //check if value is positive
        if(m_bufferToHoldControllerCommandLeftWheel[0] != '-')
        {
            controllerValueIsNegetive = false;
        }

        //print the char array to a string. If the value is 0, I will not flip anything. See:
        //https://stackoverflow.com/questions/8960087/how-to-convert-a-char-array-to-a-string
        std::string commandAsStringBefore(m_bufferToHoldControllerCommandLeftWheel);

        //Eliminate the negetive sign (unless it is a zero.
        if((controllerValueIsNegetive==true) && (commandAsStringBefore != "0.000000"))
        {
            for(int i = 0; i < m_sizeOfBuffer; i++)
            {
                m_bufferToHoldControllerCommandLeftWheel[i] = m_bufferToHoldControllerCommandLeftWheel[i+1];
            }
        }

        //add a negative sign (unless it is a zero)
        if((controllerValueIsNegetive == false) && (commandAsStringBefore != "0.000000"))
        {
            //char tempBuff
            for(int i = m_sizeOfBuffer-1; i >= 1; i--)
            {
                m_bufferToHoldControllerCommandLeftWheel[i] = m_bufferToHoldControllerCommandLeftWheel[i-1];
            }
            m_bufferToHoldControllerCommandLeftWheel[0] = '-';
        }

        //update the string data that reflects the buffer data
        std::string commandAsStringAfter(m_bufferToHoldControllerCommandLeftWheel);

        //generate a line in the log.
        m_loggingObj.buildLogMessage("INFO", 1064, "ST10 COMMAND", "The rotations per second value to send to left ST10 (with sign inverted if not zero): " + commandAsStringAfter);

	}


    //depending on the desires of the diff_drive_controller, we will deduce what command to send.
    //Some options are: CJ-commence jog, CS-change speed, SJ-stop jog. NOTE: it is assumed
    //that the ST10 drives have already been configured (jog accel, jog decel, the motor is
    //enabled, registers with steps have been reset, etc...). This method assumes that
    //the robot hardware is OK (no faults or damage), and that the controller want the robot
    //to move. There are only 3 logical states I can think of:
    //(1)Wheel is stopped, and we want to move -> CJ command
    //(2)Wheel is moving & we want to change speed -> CS (values are in RPS)
    //(3)Wheel is moving & we want to stop ->JS
    //(4)wheel is moving & we need to change direction -> try CS, and if drive faults
    //   then stop the motor, and CJ in the opposite direction
    //Implementation issue: when testing the code, I came to realize that CJ does not take
    //an argument!!This means if I am stopped, I have 2 options. Option #1 is CJ then CS
    //(commence jog followed by change speed). Option #2 is JS then CJ (set jog speed, then
    //begin the jogging). I am going to opt for option #2 as it is going to be less "jerky"
    //on start up. Fun fact, JS does not accept a negetive value, and to go backwards on startup
    //we have to munipulate the DI command.
    void AmpSteppers::buildCommandToSendToRightStepperDrive(double commandFromController)
    {
        //!!!!!!!!!!!!!!!STEP1 - IS THE DRIVE OK AND READY TO WORK?!!!!!!!!!!!!!!!!!!!!!!!
        //before sending the command to the drive, figure out if I need to update the status of
        //the drive
        getFullStatusOfStepperDrive(rightSt10Drive, m_socketRight, "Right");

        //desired robot direction is forward
        if(commandFromController < 0.0)
        {
            m_controllerRequestsRightWheelBackward = true;
        }

        //desired robot direction is backwards
        if(commandFromController > 0.0)
        {
            m_controllerRequestsRightWheelBackward = false;
        }

        //drive is ready, and not faulted
        bool readyToWork = false;
        bool driveIsInMotion = false;
        bool controllerWantsMotion = false;
        bool controllerWantsMotionStopped = false;



        //check status of all items of interest which let us know we are OK to begin moving
        if((rightSt10Drive.m_st10IsFaulted_E == false) && (rightSt10Drive.m_st10IsDisabled_D == false) && (rightSt10Drive.m_st10IsReadyToWork_R == true))
        {
            //system is ready to do work
            readyToWork = true;
        }


        //check if we are in motion
        if((rightSt10Drive.m_st10IsJogging_J == true)||(rightSt10Drive.m_st10MotorIsMoving_F == true) || (rightSt10Drive.m_st10MotionCommandExecuting_M==true))
        {
            driveIsInMotion = true;
        }


        //check if controller wants us to move. Meaning the command that the controller gave us is not 0.0
        if((commandFromController > 0.0) || (commandFromController < 0.0))
        {
            controllerWantsMotion = true;
        }

        //check if controller wants us to stop. If the command is 0.0 then we need to stop
        if(commandFromController == 0.0)
        {
            controllerWantsMotionStopped = true;
        }

        //!!!!!!!!!!!!!!!!!!!!STEP2 - Figure out what the diff_drive wants
        //an enum to remember what the drive wants.
        enum typeOfControllerCommand
        {
            START_MOVING,//CJ - was stopped & controller now wants motion.
            CHANGE_SPEED,//CS - already runining & a new speed is issued.
            STOP //SJ - running and controller wants wheel stopped.
        };

        //create an enum object that will hold the command type that is desired by
        //the diff drive controller
        typeOfControllerCommand commandTypeDesired;


        //if we are stopped & want to move, then CJ (commence jog) is the command we want.
        if((readyToWork==true) && (driveIsInMotion==false) && (controllerWantsMotion==true));
        {
            commandTypeDesired = START_MOVING;

        }

        //if we are moving & controller just sent a command (faster or slower or event he same speed)
        //then CS (change speed) is the command we want
        if((readyToWork==true) && (driveIsInMotion==true) && (controllerWantsMotion==true))
        {
            commandTypeDesired = CHANGE_SPEED;
        }


        //if the controller wants wheels to stop, we need to do so (via SJ command - stop jogging)
        if(controllerWantsMotionStopped == true)
        {
            commandTypeDesired = STOP;
        }

        //!!!!!!!!!!!!!!!!STEP #3 - BUILD AND SEND THE COMMAND
        //vectors grow dynamically so I will build the command using a vector,
        // and later I will convert to an array. I fill it with money to make it easier to work with (initialization)
        std::vector<char> commandToStepperDriveAsVect {'$', '$','$','$','$','$','$','$','$','$','$','$','$','$','$'};

        //keep track of the number of characters in our command.
        int assembledCommandNumberOfChars = 0;


        //motor is stopped and controller wants it to move. Two things need to happen:
        //1. build and send jog speed (JS) command
        //2. send commence jog (CJ) command
        if(commandTypeDesired == START_MOVING)
        {
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("DEBUG", 1138, "ST10 COMMAND", "RIGHT wheel is stopped & we want motion (JS+CJ+DI for direction)");

            //push header into message
            commandToStepperDriveAsVect[0] = m_HEADER_ONE;
            commandToStepperDriveAsVect[1] = m_HEADER_TWO;

            //push command into message
            commandToStepperDriveAsVect[2] = m_J_HEX;
            commandToStepperDriveAsVect[3] = m_S_HEX;

            assembledCommandNumberOfChars = 4;

            //push the jog speed we have already processed from the diff_drive_controller
            //cycle through the array holding the command and store it in the vector. Since the value
            //has to be over 0.0042 and since the char can be negative, ex: -0.456 (6 chars)or positive
            //0.345 (5 chars) and the value is rounded I am choosing to only take the first 5 chars
            for(int i = 0; i <= 5; i++)
            {
                //if we have written all the values of the command to the vector, and now
                //encounter the first "filler", then exit the for loop.
                if(m_bufferToHoldControllerCommandRightWheel[i] == '$')
                {
                    break;
                }

                //since JS only accepts positive commands, I must eliminate the negative sign,
                //and set a bool letting me know I gotta run in reverse. Once we are running
                //the CS command can accept both positive and negetive and change direction based
                //on the sign.
                if(m_bufferToHoldControllerCommandRightWheel[i] == m_MINUS_HEX)
                {
                    m_controllerRequestsRightWheelBackward = true;

                    //for this time only shift all the values over to overwrite the negative sign but
                    //to keep all the numeric digits
                    for(int j=0; j < 6; j++ )
                    {
                        m_bufferToHoldControllerCommandRightWheel[j] = m_bufferToHoldControllerCommandRightWheel[j+1];
                    }
                }

                //moving from array to vector. All items are already chars.
                commandToStepperDriveAsVect[4+i] =  m_bufferToHoldControllerCommandRightWheel[i];

                assembledCommandNumberOfChars = assembledCommandNumberOfChars + 1;
            }


            //put the end of the transmission in the array. CS Message is now built.
            commandToStepperDriveAsVect[assembledCommandNumberOfChars] = m_FOOTER_HEX;

            //since we put in the message terminator (m_FOOTER_HEX) we need to increase the value.
            assembledCommandNumberOfChars = assembledCommandNumberOfChars + 1;


            //resize the vector to hold the message, the whole message, and nothing but the message.
            commandToStepperDriveAsVect.resize(assembledCommandNumberOfChars);

            //create an array to push vector data into an old fashined c style array, as that
            //is what the send method needs.
            char commandToSendToDrive[commandToStepperDriveAsVect.size()];

            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("DEBUG", 1205, "ST10 COMMAND", "Number of chars in command to right wheel: "
                                         + std::to_string(commandToStepperDriveAsVect.size()));

            //write all the data over from vector to array
            for(unsigned int i=0; i < commandToStepperDriveAsVect.size(); i++)
            {
                commandToSendToDrive[i] = commandToStepperDriveAsVect[i];
            }


            //direction of motor rotatoin is set with DI command. Relative to robot send wheel backwards.
            if(m_controllerRequestsRightWheelBackward == true)
            {
                //send request
                sendRequest(&m_DI_NEG[0], m_DI_NEG_size, m_socketRight, "RIGHT");

                //collect response
                if(collectResponse(m_socketRight) == "%")
                {
                    m_loggingObj.buildLogMessage("DEBUG", 1239, "ST10 COMMAND", "Negative DI to RIGHT ST10 is successful");
                }
            }

            //direction of motor rotatoin is set with DI command. Relative to robot send wheel forwards.
            if(m_controllerRequestsRightWheelBackward == false)
            {
                //send request
                sendRequest(&m_DI_POS[0], m_DI_POS_size, m_socketRight, "RIGHT");

                //collect response
                if(collectResponse(m_socketRight) == "%")
                {
                    m_loggingObj.buildLogMessage("DEBUG", 1239, "ST10 COMMAND", "Positive DI to RIGHT ST10 is successful");
                }
            }


            //Send the jog speed (JS) command with the appropriate rps value to the ST10 Drive
            sendRequest(&commandToSendToDrive[0], commandToStepperDriveAsVect.size(), m_socketRight, "RIGHT");

            //confirm that message was OK & if so, send the CJ command.
            if(collectResponse(m_socketRight) == "%")
            {
                //send the commence jog (CJ) command
                sendRequest(&m_CJ[0], m_CJ_size, m_socketRight, "RIGHT");

                //let user know all is OK
                if(collectResponse(m_socketRight)=="%")
                {
                    m_loggingObj.buildLogMessage("DEBUG", 1256, "ST10 COMMAND", "Right ST10 was given a run command successfully (JSxxxx+CJ)");
                }
            }
        }//end of START_MOVING commands


        //motor is moving and controller wants it to change speed.
        //This builds and sends a CS command (change speed)
        if(commandTypeDesired == CHANGE_SPEED)
        {
            m_loggingObj.buildLogMessage("DEBUG", 1287, "ST10 COMMAND", "Right wheel is in motion & a speed change is desired (CS). The signage of this command controls direction ");

            //push header into message
            commandToStepperDriveAsVect[0] = m_HEADER_ONE;
            commandToStepperDriveAsVect[1] = m_HEADER_TWO;

            //push command into message
            commandToStepperDriveAsVect[2] = m_C_HEX;
            commandToStepperDriveAsVect[3] = m_S_HEX;

            assembledCommandNumberOfChars = 4;

            //push the jog speed we have already processed from the diff_drive_controller
            //cycle through the array holding the command and store it in the vector. Since the value
            //has to be over 0.0042 and since the char can be negative, ex: -0.456 (6 chars)or positive
            //0.345 (5 chars) and the value is rounded I am choosing to only take the first 5 chars
            for(int i = 0; i <= 5; i++)
            {
                //if we have written all the values of the command to the vector, and now
                //encounter the first "filler", then exit the for loop.
                if(m_bufferToHoldControllerCommandRightWheel[i] == '$')
                {
                    break;
                }

                //moving from array to vector. All items are already chars.
                commandToStepperDriveAsVect[4+i] =  m_bufferToHoldControllerCommandRightWheel[i];

                assembledCommandNumberOfChars = assembledCommandNumberOfChars + 1;
            }

            //put the end of the transmission in the array. CS Message is now built.
            commandToStepperDriveAsVect[assembledCommandNumberOfChars] = m_FOOTER_HEX;

            //since we put the message terminator m_FOOTER_HEX, we need to increase the value.
            assembledCommandNumberOfChars = assembledCommandNumberOfChars + 1;


            //resize the vector to hold the message, the whole message, and nothing but the message.
            commandToStepperDriveAsVect.resize(assembledCommandNumberOfChars);

            //create an array to push vector data into an old fashined c style array, as that
            //is what the send method needs.
            char commandToSendToDrive[commandToStepperDriveAsVect.size()];

            //write all the data over from vector to array
            for(unsigned int i=0; i < commandToStepperDriveAsVect.size(); i++)
            {
                commandToSendToDrive[i] = commandToStepperDriveAsVect[i];
            }


            //Send the command to the ST10 Drive
            sendRequest(&commandToSendToDrive[0], commandToStepperDriveAsVect.size(), m_socketRight, "RIGHT");

            //let user know all is OK
            if(collectResponse(m_socketRight)=="%")
            {
                m_loggingObj.buildLogMessage("DEBUG", 1354, "ST10 COMMAND", "Right ST10 was given a run command successfully (CS)");
            }
        }//end of CHANGE_SPEED command


        //motor is moving and controller wants it stopped.
        if(commandTypeDesired == STOP)
        {
            m_loggingObj.buildLogMessage("DEBUG", 1373, "ST10 COMMAND", "Right wheel is in motion and we want a stop (SJ)");

            //the SJ (stop jog) command is always the same, and is easy to send
            sendRequest(&m_SJ[0], m_SJ_size, m_socketRight, "RIGHT");

            if(collectResponse(m_socketRight)=="%")
            {
                m_loggingObj.buildLogMessage("DEBUG", 1380, "ST10 COMMAND", "Right ST10 was given a Stop Jog command successfully (SJ) ");
            }
        }
    }


    //depending on the desires of the diff_drive_controller, we will deduce what command to send.
    //Some options are: CJ-commence jog, CS-change speed, SJ-stop jog. NOTE: it is assumed
    //that the ST10 drives have already been configured (jog accel, jog decel, the motor is
    //enabled, registers with steps have been reset, etc...). This method assumes that
    //the robot hardware is OK (no faults or damage), and that the controller want the robot
    //to move. There are only 3 logical states I can think of:
    //(1)Wheel is stopped, and we want to move -> CJ command
    //(2)Wheel is moving & we want to change speed -> CS (values are in RPS)
    //(3)Wheel is moving & we want to stop ->JS
    //(4)wheel is moving & we need to change direction -> try CS, and if drive faults
    //   then stop the motor, and CJ in the opposite direction
    //Implementation issue: when testing the code, I came to realize that CJ does not take
    //an argument!!This means if I am stopped, I have 2 options. Option #1 is CJ then CS
    //(commence jog followed by change speed). Option #2 is JS then CJ (set jog speed, then
    //begin the jogging). I am going to opt for option #2 as it is going to be less "jerky"
    //on start up. Fun fact, JS does not accept a negetive value, and to go backwards on startup
    //we have to munipulate the DI command.
    void AmpSteppers::buildCommandToSendToLeftStepperDrive(double commandFromController)
    {
        //!!!!!!!!!!!!!!!STEP1 - IS THE DRIVE OK AND READY TO WORK?!!!!!!!!!!!!!!!!!!!!!!!
        //before sending the command to the drive, figure out if I need to update the status of
        //the drive
        getFullStatusOfStepperDrive(leftSt10Drive, m_socketLeft, "Left");

        //since the left wheel is special (with the whole low level negative value actually is a
        //logical forward (positive command) I have to set this bit here.
        if(commandFromController < 0.0)
        {
            m_controllerRequestsLeftWheelBackward = true;
        }

        if(commandFromController > 0.0)
        {
            m_controllerRequestsLeftWheelBackward = false;
        }

        //drive is ready, and not faulted
        bool readyToWork = false;
        bool driveIsInMotion = false;
        bool controllerWantsMotion = false;
        bool controllerWantsMotionStopped = false;

        //check status of all items of interest which let us know we are OK to begin moving
        if((leftSt10Drive.m_st10IsFaulted_E == false) && (leftSt10Drive.m_st10IsDisabled_D == false) && (leftSt10Drive.m_st10IsReadyToWork_R == true))
        {
            //system is ready to do work
            readyToWork = true;
        }

        //is motor in motion?
        if((leftSt10Drive.m_st10IsJogging_J == true)||(leftSt10Drive.m_st10MotorIsMoving_F == true) || (leftSt10Drive.m_st10MotionCommandExecuting_M==true))
        {
            driveIsInMotion = true;
        }


        //check if controller wants us to move. Meaning the command that the controller gave us is not 0.0
        if((commandFromController > 0.0) || (commandFromController < 0.0))
        {
            controllerWantsMotion = true;
        }

        //check if controller wants us to stop. If the command is 0.0 then we need to stop
        if(commandFromController == 0.0)
        {
            controllerWantsMotionStopped = true;
        }

        //!!!!!!!!!!!!!!!!!!!!STEP2 - Figure out what the diff_drive wants
        //an enum to remember what the drive wants.
        enum typeOfControllerCommand
        {
            START_MOVING,//CJ - was stopped & controller now wants motion.
            CHANGE_SPEED,//CS - already runining & a new speed is issued.
            STOP //SJ - running and controller wants wheel stopped.
        };

        //create an enum object that will hold the command type that is desired by
        //the diff drive controller
        typeOfControllerCommand commandTypeDesired;


        //if we are stopped & want to move, then CJ (commence jog) is the command we want.
        if((readyToWork==true) && (driveIsInMotion==false) && (controllerWantsMotion==true));
        {
            commandTypeDesired = START_MOVING;

        }

        //if we are moving & controller just sent a command (faster or slower or event he same speed)
        //then CS (change speed) is the command we want
        if((readyToWork==true) && (driveIsInMotion==true) && (controllerWantsMotion==true))
        {
            commandTypeDesired = CHANGE_SPEED;
        }


        //if the controller wants wheels to stop, we need to do so (via SJ command - stop jogging)
        if(controllerWantsMotionStopped == true)
        {
            commandTypeDesired = STOP;
        }

        //!!!!!!!!!!!!!!!!STEP #3 - BUILD AND SEND THE COMMAND
        //vectors grow dynamically so I will build the command using a vector,
        // and later I will convert to an array. I fill it with money to make it easier to work with
        std::vector<char> commandToStepperDriveAsVect {'$', '$','$','$','$','$','$','$','$','$','$','$','$','$','$'};

        //keep track of the number of characters in our command.
        int assembledCommandNumberOfChars = 0;


        //motor is stopped and controller wants it to move. Two things need to happen:
        //1. build and send jog speed (JS) command
        //2. send commence jog (CJ) command
        if(commandTypeDesired == START_MOVING)
        {
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("DEBUG", 1522, "ST10 COMMAND", "LEFT wheel is stopped & we want motion (JS+CJ+DI for direction)");

            //push header into message
            commandToStepperDriveAsVect[0] = m_HEADER_ONE;
            commandToStepperDriveAsVect[1] = m_HEADER_TWO;

            //push command into message
            commandToStepperDriveAsVect[2] = m_J_HEX;
            commandToStepperDriveAsVect[3] = m_S_HEX;

            assembledCommandNumberOfChars = 4;

            //push the jog speed we have already processed from the diff_drive_controller
            //cycle through the array holding the command and store it in the vector. Since the value
            //has to be over 0.0042 and since the char can be negative, ex: -0.456 (6 chars)or positive
            //0.345 (5 chars) and the value is rounded I am choosing to only take the first 5 chars
            for(int i = 0; i <= 5; i++)
            {
                //if we have written all the values of the command to the vector, and now
                //encounter the first "filler", then exit the for loop.
                if(m_bufferToHoldControllerCommandLeftWheel[i] == '$')
                {
                    break;
                }

                //since JS only accepts positive commands, I must eliminate the negative sign,
                //and set a bool letting me know I gotta run in reverse. Once we are running
                //the CS command can accept both positive and negetive and change direction based
                //on the sign.
                if(m_bufferToHoldControllerCommandLeftWheel[i] == m_MINUS_HEX)
                {
                    //for this time only shift all the values over to overwrite the negative sign but
                    //to keep all the numeric digits
                    for(int j=0; j < 6; j++ )
                    {
                        m_bufferToHoldControllerCommandLeftWheel[j] = m_bufferToHoldControllerCommandLeftWheel[j+1];
                    }
                }

                //moving from array to vector. All items are already chars.
                commandToStepperDriveAsVect[4+i] =  m_bufferToHoldControllerCommandLeftWheel[i];

                assembledCommandNumberOfChars = assembledCommandNumberOfChars + 1;
            }


            //put the end of the transmission in the array. CS Message is now built.
            commandToStepperDriveAsVect[assembledCommandNumberOfChars] = m_FOOTER_HEX;

            //since we put in the message terminator (m_FOOTER_HEX) we need to increase the value.
            assembledCommandNumberOfChars = assembledCommandNumberOfChars + 1;


            //resize the vector to hold the message, the whole message, and nothing but the message.
            commandToStepperDriveAsVect.resize(assembledCommandNumberOfChars);

            //create an array to push vector data into an old fashined c style array, as that
            //is what the send method needs.
            char commandToSendToDrive[commandToStepperDriveAsVect.size()];

            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("DEBUG", 1589, "ST10 COMMAND", "Number of chars in command to left wheel: "
                                         + std::to_string(commandToStepperDriveAsVect.size()));

            //write all the data over from vector to array
            for(unsigned int i=0; i < commandToStepperDriveAsVect.size(); i++)
            {
                commandToSendToDrive[i] = commandToStepperDriveAsVect[i];
            }

            //
            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!TESTING ONLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            /*NOTE: replaced with a log entry in the send method. It's better to log what the send command
            is sending than seeing the message as it's constructed. This makes reading the log a bitch.
            KEEP COMMENTED OUT!!!!
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("DEBUG", 1605, "ST10 COMMAND", "Command that is being sent to left wheel: ");

            for(unsigned int i=0; i < commandToStepperDriveAsVect.size(); i++)
            {
                m_loggingObj.buildLogMessage("DEBUG", 1609, "ST10 COMMAND", "char: " + std::to_string(commandToSendToDrive[i]));
            }
            */
            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!TESTING ONLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            //direction of motor rotation is set with DI command. Relative to robot send wheel backwards.
            if(m_controllerRequestsLeftWheelBackward == true)
            {
                sendRequest(&m_DI_POS[0], m_DI_POS_size, m_socketLeft, "LEFT");
                    if(collectResponse(m_socketLeft) == "%")
                    {
                        m_loggingObj.buildLogMessage("DEBUG", 1621, "ST10 COMMAND", "Positive DI to LEFT ST10 is successful");
                    }
            }

            //direction of motor rotation is set with DI command. Relative to robot send wheel backwards.
            if(m_controllerRequestsLeftWheelBackward == false)
            {
                sendRequest(&m_DI_NEG[0], m_DI_NEG_size, m_socketLeft, "LEFT");
                    if(collectResponse(m_socketLeft) == "%")
                    {
                        m_loggingObj.buildLogMessage("DEBUG", 1621, "ST10 COMMAND", "Negative DI to LEFT ST10 is successful");
                    }
            }


            //Send the command to the ST10 Drive
            sendRequest(&commandToSendToDrive[0], commandToStepperDriveAsVect.size(), m_socketLeft, "LEFT");

            //confirm that message was OK & if so, send the CJ command.
            if(collectResponse(m_socketLeft) == "%")
            {
                //send the CJ command
                sendRequest(&m_CJ[0], m_CJ_size, m_socketLeft, "LEFT");

                //let user know all is OK
                if(collectResponse(m_socketLeft)=="%")
                {
                    m_loggingObj.buildLogMessage("DEBUG", 1638, "ST10 COMMAND", "left ST10 was given a run command successfully (JSxxxx+CJ)");
                }
            }
        }//end of START_MOVING commands


        //motor is moving and controller wants it to change speed.
        //This builds and sends a CS command (change speed)
        if(commandTypeDesired == CHANGE_SPEED)
        {
            m_loggingObj.buildLogMessage("DEBUG", 1670, "ST10 COMMAND", "LEFT wheel is in motion & a speed change is desired (CS). The signage of this command controls direction (I believe)");

            //push header into message
            commandToStepperDriveAsVect[0] = m_HEADER_ONE;
            commandToStepperDriveAsVect[1] = m_HEADER_TWO;

            //push command into message
            commandToStepperDriveAsVect[2] = m_C_HEX;
            commandToStepperDriveAsVect[3] = m_S_HEX;

            assembledCommandNumberOfChars = 4;

            //push the jog speed we have already processed from the diff_drive_controller
            //cycle through the array holding the command and store it in the vector. Since the value
            //has to be over 0.0042 and since the char can be negative, ex: -0.456 (6 chars)or positive
            //0.345 (5 chars) and the value is rounded I am choosing to only take the first 5 chars
            for(int i = 0; i <= 5; i++)
            {
                //if we have written all the values of the command to the vector, and now
                //encounter the first "filler", then exit the for loop.
                if(m_bufferToHoldControllerCommandLeftWheel[i] == '$')
                {
                    break;
                }

                //moving from array to vector. All items are already chars.
                commandToStepperDriveAsVect[4+i] =  m_bufferToHoldControllerCommandLeftWheel[i];

                assembledCommandNumberOfChars = assembledCommandNumberOfChars + 1;
            }

            //put the end of the transmission in the array. CS Message is now built.
            commandToStepperDriveAsVect[assembledCommandNumberOfChars] = m_FOOTER_HEX;

            //since we put the message terminator m_FOOTER_HEX, we need to increase the value.
            assembledCommandNumberOfChars = assembledCommandNumberOfChars + 1;


            //resize the vector to hold the message, the whole message, and nothing but the message.
            commandToStepperDriveAsVect.resize(assembledCommandNumberOfChars);

            //create an array to push vector data into an old fashined c style array, as that
            //is what the send method needs.
            char commandToSendToDrive[commandToStepperDriveAsVect.size()];

            //write all the data over from vector to array
            for(unsigned int i=0; i < commandToStepperDriveAsVect.size(); i++)
            {
                commandToSendToDrive[i] = commandToStepperDriveAsVect[i];
            }

            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!TESTING ONLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            /*NOTE: replaced with a log entry in the send method. It's better to log what the send command
            is sending than seeing the message as it's constructed. This makes reading the log a bitch.
            KEEP COMMENTED OUT!!!!
            m_loggingObj.buildLogMessage("DEBUG", 1722, "ST10 COMMAND", "CS command for LEFT wheel looks like so: ");
            for(unsigned int i=0; i < commandToStepperDriveAsVect.size(); i++)
            {
                m_loggingObj.buildLogMessage("DEBUG", 1725, "ST10 COMMAND", "Char: " + std::to_string(commandToStepperDriveAsVect[i]));
            }
            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!TESTING ONLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            */

            //Send the command to the ST10 Drive
            sendRequest(&commandToSendToDrive[0], commandToStepperDriveAsVect.size(), m_socketLeft, "LEFT");

            //let user know all is OK
            if(collectResponse(m_socketLeft)=="%")
            {
                m_loggingObj.buildLogMessage("DEBUG", 1734, "ST10 COMMAND", "LEFT ST10 was given a run command successfully (CS)");
            }

        }//end of CHANGE_SPEED command


        //motor is moving and controller wants it stopped.
        if(commandTypeDesired == STOP)
        {
            m_loggingObj.buildLogMessage("DEBUG", 1753, "ST10 COMMAND", "LEFT wheel is in motion and we want a stop (SJ)");

            //the SJ (stop jog) command is always the same, and is easy to send
            sendRequest(&m_SJ[0], m_SJ_size, m_socketLeft, "LEFT");

            if(collectResponse(m_socketLeft)=="%")
            {
                m_loggingObj.buildLogMessage("DEBUG", 1760, "ST10 COMMAND", "Left ST10 was given a Stop Jog command successfully (SJ) ");
            }
        }
    }


    //standardized methods that use all the small methods above with a core purpose of
    //receiving a controller command, processing it (make sure it is withing speed bounds,
    //convert radians/sec of wheel to rotations per sec of motor), building a command
    //which the ST10 drives understand, and sending it to the ST10 drive.
    void AmpSteppers::processControllerCommandAndSendToRightDrive(double diffDriveCommandForRightWheel)
    {
        //Figure out if the command we have received from the diff drive controller is within
        //the tolerances I have computed; and store the boolean result. If the controller command
        //is within the tolerance, the method stores it in: m_controllerCommandInRadVerifiedGood.
        bool commandFromDiffDriveControllerIsGood = verifyCommandFromController(diffDriveCommandForRightWheel);

        //command from diff_drive is no good.
        if(commandFromDiffDriveControllerIsGood == false)
        {
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("WARN", 1860, "ST10 COMMAND", "The command from the controller is no good. Not sent to RIGHT ST10.");
        }

        //store the diff drive command after conversion from wheel Rad/sec to motor rev/sec
        double diffDriveCommandAsMotorRps = 0.0;

        //if the command is good, let's keep the process going
        if(commandFromDiffDriveControllerIsGood == true)
        {
            //Since the diff_drive_controller is giving us a radians/second for the
            //drive wheels, before I can begin to convert the double into an Applied
            //Motion Products command, I have to convert the radians/second for the
            //drive wheel into RPS (rotations per second) of the motor shaft!
            diffDriveCommandAsMotorRps = convertControllerCommandToMotorRPS(diffDriveCommandForRightWheel);

            //this method stores the controller command for the right wheel in a format which
            //the ST10 drives understands (pushing each number and the "dot" into an element in an array).
            //This value is what my code will send to the ST10 as it has been converted to Motor
            //RPS from drive wheel Rad/sec, and stored as chars in an array.
            storeFinalCommandFromControllerAsMotorRpsRightWheel(diffDriveCommandAsMotorRps);

            //create socket
            createSocketRight();

            //create a TCP connection to left ST10
            if (connect(m_socketRight, (struct sockaddr *)&m_connectionToRightStepperDrive, sizeof(m_connectionToRightStepperDrive)) < 0)
            {
                //generate a line in the log.
                //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
                //arg 2 - line number in source file
                //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
                //arg 4 - message for user
                m_loggingObj.buildLogMessage("ERROR", 2010, "INIT-NETWORKING", "A TCP connection to the right ST10 has failed to establish.");
            }

            //build the command to send to the right ST10, and send it.
            //NOTE: I am fully aware the argument is the value in radians per second and not rotations
            //per second. All that arg is used for is to infer what the drive is asking for (stopping or not).
            buildCommandToSendToRightStepperDrive(diffDriveCommandForRightWheel);

            close(m_socketRight);
        }
    }


    //standardized methods that use all the small methods above with a core purpose of
    //receiving a controller command, processing it (make sure it is withing speed bounds,
    //convert radians/sec of wheel to rotations per sec of motor), building a command
    //which the ST10 drives understand, and sending it to the ST10 drive.
    void AmpSteppers::processControllerCommandAndSendToLeftDrive(double diffDriveCommandForLeftWheel)
    {
        //Figure out if the command we have received from the diff drive controller is within
        //the tolerances I have computed; and store the boolean result. If the controller command
        //is within the tolerance, the method stores it in: m_controllerCommandInRadVerifiedGood....
        //which on closer look I don't use...
        bool commandFromDiffDriveControllerIsGood = verifyCommandFromController(diffDriveCommandForLeftWheel);

        //command from diff_drive is no good.
        if(commandFromDiffDriveControllerIsGood == false)
        {
            //generate a line in the log.
            //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
            //arg 2 - line number in source file
            //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
            //arg 4 - message for user
            m_loggingObj.buildLogMessage("WARN", 1909, "ST10 COMMAND", "The command from the controller is no good. Not sent to LEFT ST10.");
        }

        //store the diff drive command after conversion from wheel Rad/sec to motor rev/sec
        double diffDriveCommandAsMotorRps = 0.0;

        //if the command is good, let's keep the process going
        if(commandFromDiffDriveControllerIsGood == true)
        {
            //Since the diff_drive_controller is giving us a radians/second for the
            //drive wheels, before I can begin to convert the double into an Applied
            //Motion Products command, I have to convert the radians/second for the
            //drive wheel into RPS (rotations per second) of the motor shaft!
            diffDriveCommandAsMotorRps = convertControllerCommandToMotorRPS(diffDriveCommandForLeftWheel);

            //this method stores the controller command for the right wheel in a format which
            //the ST10 drives understands (pushing each number and the "dot" into an element in an array).
            //This value is what my code will send to the ST10 as it has been converted to Motor
            //RPS from drive wheel Rad/sec, and stored as chars in an array.
            storeFinalCommandFromControllerAsMotorRpsLeftWheel(diffDriveCommandAsMotorRps);

            //create socket
            createSocketLeft();

            //create a TCP connection to left ST10
            if (connect(m_socketLeft, (struct sockaddr *)&m_connectionToLeftStepperDrive, sizeof(m_connectionToLeftStepperDrive)) < 0)
            {
                //generate a line in the log.
                //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
                //arg 2 - line number in source file
                //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
                //arg 4 - message for user
                m_loggingObj.buildLogMessage("ERROR", 2075, "INIT-NETWORKING", "A TCP connection to the left ST10 has failed to establish.");
            }

            //build the command to send to the right ST10, and send it.
            //NOTE: I am fully aware the argument is the value in radians per second and not rotations
            //per second. All that arg is used for is to infer what the drive is asking for (stopping or not).
            buildCommandToSendToLeftStepperDrive(diffDriveCommandForLeftWheel);

            //close connection
            close(m_socketLeft);
        }
    }

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
    void AmpSteppers::timedRightAndLeftWheelTesting(double rightWheelRadians, double leftWheelRadians, long long usSleep)
    {
        //send the initial command through all the verification and translation mechanisms. Motion
        //should commence (assuming all is ok with the request)
        processControllerCommandAndSendToRightDrive(rightWheelRadians);
        processControllerCommandAndSendToLeftDrive(leftWheelRadians);

        //put the method to sleep for a given number of microseconds, where:
        // 1,000us = 1ms
        // 100,000 = 100ms (1/10 of a second)
        // 1,000,000 = 1000ms or 1 second
        usleep(usSleep);

        //send a stop command to the drive
        processControllerCommandAndSendToRightDrive(0.0);
        processControllerCommandAndSendToLeftDrive(0.0);
    }

}//namespace

