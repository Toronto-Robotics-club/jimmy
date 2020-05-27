/*
    For Purpose (and comm protocol), Author, and Date, have a look at "lidar_magnetic_encoder.cpp"

    version: 1.00 (conversion to OOP)
*/
#pragma once

//including the ros header file so we can call ROS functions
#include <ros/ros.h>
#include <iostream> //std::cout + std::endl
#include <fstream> //std::ifstream
//NOTE <libserial/SerialStream.h> is not the same as <SerialStream.h>!!!!!!!
//#include </usr/local/include/libserial/SerialStream.h> //the serial port library
//#include </usr/local/include/libserial/SerialPort.h>
#include <libserial/SerialStream.h> //the serial port library
#include <libserial/SerialPort.h>
#include <unistd.h> //for usleep();


class LidarMagneticEncoder
{

   private:

		//=================================VARIABLES===================================

        //create a SerialPort handle.
        LibSerial::SerialPort my_serial_port;

        //define a serial port path as a string. This is the
		//serial port that will be used to communicate with the arduino. Fun fact, even
		//though the arduino uses a USB port to communicate, it looks identical to a serial
		//port as the arduino has a USB bridge to a UART in the Uno processor.
        std::string serial_port_path;

        //this vector collects the response from the arduino as it comes in.
        std::vector <char> m_arduinoResponse;
        std::string m_arduinoResponseAsString;

        //I am limiting the amount of data out of the arduino to 100 characers.
        //1+++++++++++++++++++++YOU++++++++++++++++++
        const int MAX_RESPONSE_SIZE = 50;

        //These variables will hold the data we got from the arduino.
        //2+++++++++++++++++++++YOU++++++++++++++++++
        int m_commToArduinoOk_AA;//0==not ok, 1==OK
        int m_setEncoderHomeRequest_AB; //-1==problem, 1==OK
        int m_degreeData_AC; //range is 0 to 359
        int m_uncompPositionData_AC;//range is 0 to 16383

        std::string packageDescriptor = "Encoder lib: ";


        //================================FUNCTIONs=======================================

		//forward declarations for methods
		void initializeSerialPort(LibSerial::SerialPort*);

        //send request to get data from arduino. The int arg is the message
        //type that we are sending (AA~ or AB~ or etc....)
        void sendRequest(std::string);

        //collect the arduino response
        void getResponse();

        //wipe clean variables holding data during initialization
        void clearAllProcessedResponseVariables();

        //these functions call the right number of functions to extract the numberic values
        //from the message the Arduino sent. add one for each type of message that you have.
        //3+++++++++++++++++++++YOU++++++++++++++++++
        void processResponse_AA();
        void processResponse_AB();
        void processResponse_AC();

        //take the vector data and push it into a string.
        void populateArduinoResponseStringFromVector();

        //extract integer values from the arduino data between two letters.
        void processStringToIntAndAssignToMemberVariables(std::string, std::string, std::string, int&);

        //extract double values from the arduino data between two letters.
        void processStringToDoubleAndAssignToMemberVariables(std::string, std::string, std::string, double&);




	public:
	    LidarMagneticEncoder(std::string);//constructor. Arg is serial port path
        ~LidarMagneticEncoder();//destructor

        //Single fucntions to get data from arduino, and to return the values to you. The
        //string argument is just to reduce errors as the function name is not a siver
        //bullet against mistakes; so an additional input will help.

        //4+++++++++++++++++++++YOU++++++++++++++++++
        //these are very specific to your circumstances. Create a function for each
        //of your commands and define the return type.
        int getDataForCommand_AA(std::string);//result for comm test to arduino
        int getDataForCommand_AB(std::string);//set encoder zero position
        //see: https://www.techiedelight.com/return-multiple-values-functions-cpp/
        std::pair<int, int> getDataForCommand_AC(std::string);//get degree (and error flag) and uncomp position.

};


