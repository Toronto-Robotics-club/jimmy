/*
    For Purpose (and comm protocol), Author, and Date, have a look at 
    "arduinomega.cpp"

    version: 1.00 (conversion to OOP)

*/


#ifndef ARDUINOMEGA_H
#define ARDUINOMEGA_H
#include <iostream> //for std::string
#include <arpa/inet.h> //inet_addr (for sockaddr_in)


//the namespace
namespace jimmycpp
{
   //to access the class use this syntax: jimmycpp::ArduinoMega
	class ArduinoMega
	{
	    private:

            void createSocket();

            //all the messages my arduino can handle right now.
            const char m_messageOne[3] = {'A', 'A', '~'};
            const char m_messageTwo[3] = {'A', 'B', '~'};
            
            //message length. Unfortenutly, when I was using "strlen(messageOnePointer)"
            //instead of this const int, sometimes I would have memory overflow, and I 
            //would send 4 bytes instead of 3, so I was sending 'A'+'A'+'~'+'garbage'. 
            const int messageLength = 3;

            //keeping it private, & using methods to retrieve will keep
            //everthing more secure.
            //>>>>>>>>>>>>>>>>>>>RIGHT WHEEL VALUES<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            //show the motion status of the right wheel. true == moving; false==stopped (A___B)
            int m_rightWheelMotionStatus;

            //show the directoin of the right wheel (0==stopped; 1==forward; 2== backwards) (C___D)
            int m_rightWheelDirectionOfMotion;

            //The total pulses for the right wheel (additive & subtracive ralative to start position) (E___F)
            int m_rightWheelEncoderCountSinceStart;

            //Report the position of the right wheel in degrees. (G___H)
            double m_rightWheelPositionInDegrees;


            //>>>>>>>>>>>>>>>>>>>LEFT WHEEL VALUES<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            //show the motion status of the right wheel (true == moving; false==stopped) (I___J)
            int m_leftWheelMotionStatus;

            //show the directoin of the right wheel, 0==stopped; 2==LOGICAL forward; 1==LOGICAL backward (K___L)
            int m_leftWheelDirectionOfMotion;

            //show the total pulses for the left wheel (additive & subtracive ralative to start position) (M___N)
            int m_leftWheelEncoderCountSinceStart;

            //report the position of the right wheel in degrees (O___P)
            double m_leftWheelPositionInDegrees;

            //the socket is represented as an int (for some reason)
            int m_socket;

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
            struct sockaddr_in m_connectionToServer;

            std::string m_arduinoResponse;

            //For passing by reference see:
            //https://stackoverflow.com/questions/18147038/passing-object-by-reference-in-c
            void processStringToIntAndAssignToMemberVariables(std::string, std::string, std::string, int &);
            void processStringToDoubleAndAssignToMemberVariables(std::string, std::string, std::string, double &);
            void sendRequest(int);
            void collectResponse();
            void processResponse();



		public:
			ArduinoMega();//constructor declaration
            ~ArduinoMega();//destructor declaration
            //where typeOfDataWanted is:
            //1 = AA~
            //2 = AB~
            void getDataFromArduino(int typeOfDataWanted);

            //methods to access the data:
            int getRightWheelMotionStatus();
            int getRightWheelDirectionOfMotion();
            int getRightWheelEncoderCountSinceStart();
            double getRightWheelPositionInDegrees();

            int getLeftWheelMotionStatus();
            int getLeftWheelDirectionOfMotion();
            int getLeftWheelEncoderCountSinceStart();
            double getLeftWheelPositionInDegrees();
	}; //end of class


}//end of namespace

#endif // ARDUINOMEGA_H

