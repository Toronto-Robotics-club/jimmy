/*
    Purpose: the "arduinomega" (.cpp & .h) files are a means for the NUC to receive
             data from the Arduino mega (via Ethernet) to which the encoders are connected. The
             protocol is for this package to send a request (currently 'AA~' and 'AB~'
             are the only supported messages that the arduino understands), and
             process the response. The response is data wrapped in letters. Specifically:
             A_int_B - show the motion status of the right wheel. true == moving; false==stopped
             C_int_D - show the direction of the right wheel (0==stopped; 1==forward; 2== backwards)
             E_int_F - The total pulses for the right wheel (additive & subtractive relative to start position)
             G_double_H - Report the position of the right wheel in degrees.
             I_int_J - show the motion status of the right wheel. true == moving; false==stopped
             K_int_L - show the directoin of the right wheel, 0==stopped; 2==LOGICAL forward; 1==LOGICAL backward
                       NOTE: logical forward meaning the wheel is rotating to drive robot forward.
             M_int_N - show the total pulses for the left wheel (additive & subtracive ralative to start position)
             O_double_P - Report the position of the right wheel in degrees.

    Author - admin@TorontoRoboticsClub.com (MO)

    Date - December 20, 2018

    version - 1.00 (this version is the conversion from a main method with methods to a class that I can use
                    as a library... folowing the lead of other great projects).
*/

#include "jimmycpp/arduinomega.h"
#include <iostream> //for std::cout, std::endl, and std::string
#include <arpa/inet.h> //inet_addr
#include <cstring> // for strlen (never use string.h. See https://stackoverflow.com/questions/9257665/difference-between-string-and-string-h#9257719
#include <string> //for std::stoi
#include <ros/ros.h> //for ros logging

namespace jimmycpp
{

	//this is a definition of the class constructor outside of the class. In
   //this case we have ClassName::Constructor() to be able to define the item.
	ArduinoMega::ArduinoMega()
	{
	    //>>>>>>>>>>>>>>>>>>>RIGHT WHEEL VALUES<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        //show the motion status of the right wheel. true == moving; false==stopped (A___B)
        m_rightWheelMotionStatus = -1;

        //show the directoin of the right wheel (0==stopped; 1==forward; 2== backwards) (C___D)
        m_rightWheelDirectionOfMotion = -1;

        //The total pulses for the right wheel (additive & subtracive ralative to start position) (E___F)
        m_rightWheelEncoderCountSinceStart = 0;

        //Report the position of the right wheel in degrees. (G___H)
        m_rightWheelPositionInDegrees = -0.001;


        //>>>>>>>>>>>>>>>>>>>LEFT WHEEL VALUES<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        //show the motion status of the right wheel (true == moving; false==stopped) (I___J)
        m_leftWheelMotionStatus = -1;

        //show the directoin of the right wheel, 0==stopped; 2==LOGICAL forward; 1==LOGICAL backward (K___L)
        m_leftWheelDirectionOfMotion = -1;

        //show the total pulses for the left wheel (additive & subtracive ralative to start position) (M___N)
        m_leftWheelEncoderCountSinceStart = 0;

        //report the position of the right wheel in degrees (O___P)
        m_leftWheelPositionInDegrees = -0.001;

        //create a socket
        createSocket();

        //this code leverages the sockaddr_in structure, where I assign
        //values to a few of the structure's elements. This allows us to communicate with the arduino
        m_connectionToServer.sin_addr.s_addr = inet_addr("192.168.0.21");//arduino IP address
        m_connectionToServer.sin_family = AF_INET; //type of IP addres. where AF_INET is IPv4
        m_connectionToServer.sin_port = htons(23500); //port is set via a method

        //Connect to remote server. ...sys/socket.h, which contains a definition for connect,
        // is part of the O/S code files. I searched for it, and found at least 20 socket.h
        //files all over the O/S.
        if (connect(m_socket , (struct sockaddr *)&m_connectionToServer , sizeof(m_connectionToServer)) < 0)
        {
            std::cout << "connect error" << std::endl;
            ROS_FATAL_STREAM("arduino connect error in jimmycpp ArduinoMega lib");
        }
        
        //
        std::cout << "Connected - TODO: implement a means to pass this info to ROS " << std::endl;
	}

	//destructor
	ArduinoMega::~ArduinoMega()
	{
        //did object end?
        ROS_FATAL_STREAM("arduino object has left the building$$$$$$$$$$$$$$$$$$");
	}

	//this class method generates a socket (which for some reason is stored as an int), and
	//stores it in a member variable.
	void ArduinoMega::createSocket()
	{
	    //initializing the socket with the socket() function.
        //arg 1 - Address Family - AF_INET (this is IP version 4)
        //arg 2 - Type - SOCK_STREAM (this means connection oriented TCP protocol)
        //        SOCK_DGRAM indicates the UDP protocol.
        //arg 3 - Protocol - 0 [ or IPPROTO_IP This is IP protocol]
        m_socket = socket(AF_INET , SOCK_STREAM , 0);

        //make sure the socket we got is OK
        if (m_socket == -1)
        {
            std::cout << "problem creating a socket." << std::endl;
            ROS_FATAL_STREAM("C++ arduino lib failed to connect to arduino. Uh oh.");
        }
	}

	// The send method sends data out via a char array. Basically the contents of a char array
    // is the data that is sent out by the send method. But obviously passing an entire char
    // array is expensive, and the send method requires a pointer to the char array. I already
    // defined a char array with a message, but since I need to pass a pointer, the line below
    // gets a pointer type that points to the memory address of the first character in the
    // char array. Later on in ros, this can be put into a case statement, and the pointer can
    // point to different char arrays based on the "message" from a topic.
	void ArduinoMega::sendRequest(int messageSelection)
	{
	    //define a char pointer to point to the memory address of the message we want to send
	    const char *messageOnePointer = &m_messageOne[0];
	    const char *messageTwoPointer = &m_messageTwo[0];


	    //send m_messageOne aka 'AA~'
	    if(messageSelection == 1)
        {
            //if( send(m_socket , messageOnePointer , strlen(messageOnePointer) , 0) < 0) but might be sending extra...
            if( send(m_socket , messageOnePointer , messageLength , 0) < 0)
            {
                std::cout << "Send failed" << std::endl;
            }

            std::cout << "AA~ Sent\n" << std::endl;

        }

        //user wants to send m_messageTwo aka 'AB~'
        if(messageSelection == 2)
        {
            if( send(m_socket , messageTwoPointer , messageLength , 0) < 0)
            {
                std::cout << "Send failed" << std::endl;
            }

            std::cout << "AB~ Sent\n" << std::endl;
        }
	}//sendRequest method

    //After sending message to the arduino (AA~ or AB~) it will send a response. This
    //method collects the entire response and stores it in m_arduinoResponse.
	void ArduinoMega::collectResponse()
	{
        //initialize a string
        m_arduinoResponse = "";

        //create a temp array. This will hold a single line recieved from
        //the arduino. Array is initialized with NULs (which makes making a
        //string out of a char array with nuls a pain in the ass!!)
        char tempBuffer [300] = {};

        //fill array with money. this special character will allow us to know
        //the location where the data we wrote in meets the original '$' contents.
        std::fill(&tempBuffer[0], &tempBuffer[300], '$');


        //the number of lines I want to recieve from the arduino. This is an unusual
        //value for the following reasons (figured out via println):
        //(1) sometimes the buffer where the recv method reads from has only one value.
        //    ex: letter A only (as per my,*ahem", "protocol".
        //(2) sometimes the \n is all a recv feteches!
        //(3) sometimes the buffer where the recv method reads has multiple values, so
        //    the recv fetches many items that get unpacked in the second loop. This is
        //    why sometimes we increase the value by only 1, but get WAY more values. I
        //    observed this behaviour to be non repeating. Sometimes it reads 5 values,
        //    and sometimes it reads only 3 values.
        // At a value of 60 I am always getting the message, and run the recv command
        // unnecesserily. For this reason I have implemented the "end transmission"
        // characters (~~~), which allow me to kill the for loop once the full message is
        // retrieved.
        int numberOfTimesRecvRuns = 60;

        //number of characters per line. do not reduce as it is needed to be this size to
        // get the full insult if the protocol is not followed.
        int arduinoNumberOfCharsPerLine = 50;

        bool fullResponseRecieved = false;

        //recieve the entire arduino response. The magic number is the number of times
        // we call the recv method (which reads a line from the socket).
        for(int i = 0; i < numberOfTimesRecvRuns; i++)
        {
            //call the recv method over and over as it gets a single arduino line with
            //every iteration. The data is written into the tempBuffer array which has
            //been pre-filled with $ characters.
            if( recv(m_socket, tempBuffer , sizeof(tempBuffer) , 0) < 0)
            {
            std::cout << "recv failed" << std::endl;
            }

            //write out the single line we recieved to a string (which grows on the fly). 300 because
            //i dont believe I will have more than 300 characters per line. However once the data the
            //recv has finished ($ character is seen) or it is the end of the transmission (~), then
            //exit the loop.
            for(int j = 0; j < arduinoNumberOfCharsPerLine; j++ )
            {
                //kill the loop the moment there is a $ character. When i created the
                //array i initialized it with $ characters. so if I am seeing a $
                //character it means that the data recv recieved from the arduino has all been
                //given to the string.
                if(tempBuffer[j] == '$' )
                {
                    //std::cout << "I ran... See ya" << std::endl;
                    break;
                }

                //end of transmission detected
                if(tempBuffer[j] == '~')
                {
                    fullResponseRecieved = true;
                }

                //write the contents of the char array from recv to the string
                m_arduinoResponse = m_arduinoResponse+tempBuffer[j];
            }

            //empty array - see: https://stackoverflow.com/questions/632846/clearing-a-char-array-c
            std::fill(&tempBuffer[0], &tempBuffer[300], '$');

            // A '~' character means the full message has been recieved and there is no
            // need to keep looping for the purpose of running the recv method.
            if(fullResponseRecieved == true)
            {
                //reset the value
                fullResponseRecieved = false;
                //std::cout << "killing recv loop" << std::endl;
                break;
            }
        }//outer for loop
        //ROS_FATAL_STREAM("collecting arduino response...DONE SUCESSFULLY: "<< m_arduinoResponse);
	}//collectResponse method


    //process the string response from the arduino and store in values.
	void ArduinoMega::processResponse()
	{
        //process the int and double values we recieved and push them into memory
        //RIGHT wheel int
        processStringToIntAndAssignToMemberVariables(m_arduinoResponse, "A", "B", m_rightWheelMotionStatus);
        processStringToIntAndAssignToMemberVariables(m_arduinoResponse, "C", "D", m_rightWheelDirectionOfMotion);
        processStringToIntAndAssignToMemberVariables(m_arduinoResponse, "E", "F", m_rightWheelEncoderCountSinceStart);
        //RIGHT wheel double
        processStringToDoubleAndAssignToMemberVariables(m_arduinoResponse, "G", "H", m_rightWheelPositionInDegrees);

        //LEFT wheel int
        processStringToIntAndAssignToMemberVariables(m_arduinoResponse, "I", "J", m_leftWheelMotionStatus);
        processStringToIntAndAssignToMemberVariables(m_arduinoResponse, "K", "L", m_leftWheelDirectionOfMotion);
        processStringToIntAndAssignToMemberVariables(m_arduinoResponse, "M", "N", m_leftWheelEncoderCountSinceStart);
        //LEFT wheel double
        processStringToDoubleAndAssignToMemberVariables(m_arduinoResponse, "O", "P", m_leftWheelPositionInDegrees);
	}



	//This class method receives the full Arduino response, and extracts the int value of interest
	//that is nested in between two letters. The value is then stored in the appropriate
	//member variable. For passing by reference see:
	//https://stackoverflow.com/questions/18147038/passing-object-by-reference-in-c
    void ArduinoMega::processStringToIntAndAssignToMemberVariables(std::string allData, std::string start, std::string finish, int &memberVariable)
    {
        //get the value.See:
        //https://stackoverflow.com/questions/30073839/c-extract-number-from-the-middle-of-a-string#30074453
        //http://www.cplusplus.com/reference/string/string/find/
        int p1 = allData.find(start);
        int p2 = allData.find(finish);

        //arg #1 - start position
        //arg #2 - length
        //See http://www.cplusplus.com/reference/string/string/substr/
        //indeed returns a string....
        std::string stringResult = allData.substr(p1 +1 , p2 - p1 -1);

        //convert the string value to an integer. See claudios answer in:
        //https://stackoverflow.com/questions/7663709/how-can-i-convert-a-stdstring-to-int#7664227
        //also see: http://www.cplusplus.com/reference/string/stoi/
        int finalValue = std::stoi(stringResult);

        //assign the integer I recovered to the global variable to which it belongs
        memberVariable = finalValue;
    }

    //This class method receives the full Arduino response, and extracts the double value of interest
	//that is nested in between two letters. The value is then stored in the appropriate
	//member variable. For passing by reference see:
	//https://stackoverflow.com/questions/18147038/passing-object-by-reference-in-c
    //for passing by reference see: https://stackoverflow.com/questions/18147038/passing-object-by-reference-in-c
    void ArduinoMega::processStringToDoubleAndAssignToMemberVariables(std::string allData, std::string start, std::string finish, double &memberVariable)
    {
        //get the value.See:
        //https://stackoverflow.com/questions/30073839/c-extract-number-from-the-middle-of-a-string#30074453
        //http://www.cplusplus.com/reference/string/string/find/
        int p1 = allData.find(start);
        int p2 = allData.find(finish);

        //arg #1 - start position
        //arg #2 - length
        //See http://www.cplusplus.com/reference/string/string/substr/
        //indeed returns a string....
        std::string stringResult = allData.substr(p1 +1 , p2 - p1 -1);

        //convert the string value to an integer. See claudios answer in:
        //https://stackoverflow.com/questions/7663709/how-can-i-convert-a-stdstring-to-int#7664227
        //also see: http://www.cplusplus.com/reference/string/stoi/
        double finalValue = std::stof(stringResult);

        //assign the integer I recovered to the global variable to which it belongs
        memberVariable = finalValue;
    }

    //this method is a public method the user can call after creating the object.
    //it runs the sequence of sending a request to the arduino, recieving the
    //request, and then processing the request and storing all the data in member
    //variables. The user can now decide what member variable they are interested in.
	void ArduinoMega::getDataFromArduino(int typeOfDataWanted)
	{
        sendRequest(typeOfDataWanted);
        collectResponse();
        processResponse();
	}

	//BOILERPLATE code to fetch private data. Make sure you already ran the
	//getDataFromArduino(), as it makes sure the data all the methods below
	//fetch is the latest data available.

    int ArduinoMega::getRightWheelMotionStatus()
    {
        return m_rightWheelMotionStatus;
    }

    int ArduinoMega::getRightWheelDirectionOfMotion()
    {
        return m_rightWheelDirectionOfMotion;
    }

    int ArduinoMega::getRightWheelEncoderCountSinceStart()
    {
        return m_rightWheelEncoderCountSinceStart;
    }

    double ArduinoMega::getRightWheelPositionInDegrees()
    {
        return m_rightWheelPositionInDegrees;
    }

    int ArduinoMega::getLeftWheelMotionStatus()
    {
        return m_leftWheelMotionStatus;
    }

    int ArduinoMega::getLeftWheelDirectionOfMotion()
    {
        return m_leftWheelDirectionOfMotion;
    }

    int ArduinoMega::getLeftWheelEncoderCountSinceStart()
    {
        return m_leftWheelEncoderCountSinceStart;
    }

    double ArduinoMega::getLeftWheelPositionInDegrees()
    {
        return m_leftWheelPositionInDegrees;
    }

}//namespace

