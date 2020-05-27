////////////////////////////////////////////////////////////////////////////////
// Purpose: This library is used to control the stepper system for the 3D     //
//          lidar. This code takes in requests from ROS nodes, and converts   //
//          the requests to a serial message to an Arduino (and collects the  //
//          response). The communication between this code and the arduino is //
//          a custom protocol. The communication messages are:                //
//                                                                            //
//          AA~ Serial comm test. Is the Arduino responding to C++ commands?  //
//              0 == no, 1==yes.                                              //
//          AB~ Is the lidar home? In other words, has the lidar changed the  //
//              shelf state of the limit switch?                              //
//          AC~ Is the limit switch OK? I am treating limit swich as safety   //
//              systems by monitoring NO & NC to make sure they never have    //
//              same status.                                                  //
//          AD~ Command to send lidar home. NOTE: home means that after the   //
//              limit switch is made, the lidar is driven a bit farther to    //
//              prevent mechanical play and vibration from unseating lidar    //
//              from limit switch.                                            //
//          AE~ Sweep front of robot. The sweeping range is hard coded in     //
//              arduino.                                                      //
//          AF~ Sweep back of robot. The sweeping range is hard coded in      //
//              arduino.                                                      //
//          AG~ Go to horizontal. I am not sure that mapping is going to take //
//              3d data, so this is the horizontal position.                  //
//                                                                            //
//  DATE: rewritten Jan 2020.                                                 //
//                                                                            //
//  Author: MO at TorontoRoboticsClub@protonmail.com                          //
//                                                                            //
//                                                                            //
//  Future: in the future, I can see value in the following improvements:     //
//          (1) I am not a programmer, but I know what spaghetti code is. In  //
//              the future I will need to consult with a programmer to figure //
//              out how to clean this up properly, and make it more           //
//              maintainable.                                                 //
////////////////////////////////////////////////////////////////////////////////

#include "jimmycpp/lidar_stepper_motion.h" //my header file for this source file. 

//implement the constructor
LidarStepperPulses::LidarStepperPulses(std::string nameOfSerialPort)
{
    //while object is up, store path to serial port
    serial_port_path = nameOfSerialPort;
    //let user know that an object is being created
    std::cout << packageDescriptor << "creating the " << serial_port_path << " serial port object" << std::endl;

	//initialize the LibSerial::SerialPort my_serial_port object
    initializeSerialPort(&my_serial_port);

    //size the vector
    m_arduinoResponse.resize(MAX_RESPONSE_SIZE);

    //initialize all the variables that will hold arduino responses.
    clearAllProcessedResponseVariables();

    //since every time we open a connection the arduino resets,
    //lets wait some seconds when the initial object is created, or you
    //can cut the reset and not wait. See:
    //https://arduino.stackexchange.com/questions/20426/how-to-stop-arduino-from-reseting-after-serial-connection-lost
    //5+++++++++++++++++++++YOU++++++++++++++++++
    //sleep(3);//seconds.
}

//implement the destructor
LidarStepperPulses::~LidarStepperPulses()
{
	//close the serial port.
	my_serial_port.Close();

	std::cout << packageDescriptor << "destructor has run. Serial port " << serial_port_path << " is closed." << std::endl;
}


//this method is a public method the user can call after creating the object.
//it runs the sequence of sending a request to the arduino, recieving the
//request, and then processing the request and storing all the data in member
//variables. The user can now decide what member variable they are interested in.
//6+++++++++++++++++++++YOU++++++++++++++++++
int LidarStepperPulses::getDataForCommand_AA(std::string typeOfDataWanted)
{
    //initialize all the variables that will hold arduino responses.
    clearAllProcessedResponseVariables();

    //request for AA~ (check comm)
    if(typeOfDataWanted == "AA~")
    {
        //send the request
        sendRequest(typeOfDataWanted);

        //collect response and store in char array
        getResponse();

        //convert char response to string, and extract the data from
        //the string. Store data in member variable
        processResponse_AA();
    }

    //request for AB~ (set encoder zero position)
    if(typeOfDataWanted != "AA~")
    {
        std::cout << packageDescriptor << "You are not using the correct argument for the command " << typeOfDataWanted << std::endl;
    }

    return m_commToArduinoOk_AA;
}

//this method is a public method the user can call after creating the object.
//it runs the sequence of sending a request to the arduino, recieving the
//request, and then processing the request and storing all the data in member
//variables. The user can now decide what member variable they are interested in.
//6+++++++++++++++++++++YOU++++++++++++++++++
int LidarStepperPulses::getDataForCommand_AB(std::string typeOfDataWanted)
{
    //initialize all the variables that will hold arduino responses.
    clearAllProcessedResponseVariables();

    //request for AB~ (set home position)
    if(typeOfDataWanted == "AB~")
    {
        //send the request
        sendRequest(typeOfDataWanted);

        //collect response and store in char array
        getResponse();

        processResponse_AB();
    }

    //request for AB~ (set encoder zero position)
    if(typeOfDataWanted != "AB~")
    {
        std::cout << packageDescriptor << "You are not using the correct argument for the command " << typeOfDataWanted << std::endl;
    }

    return m_platformIsHome_AB;
}


//this method is a public method the user can call after creating the object.
//it runs the sequence of sending a request to the arduino, recieving the
//request, and then processing the request and storing all the data in member
//variables. The user can now decide what member variable they are interested in.
//6+++++++++++++++++++++YOU++++++++++++++++++
int LidarStepperPulses::getDataForCommand_AC(std::string typeOfDataWanted)
{
    //initialize all the variables that will hold arduino responses.
    clearAllProcessedResponseVariables();

    //request for AC~ (check comm)
    if(typeOfDataWanted == "AC~")
    {
        //send the request
        sendRequest(typeOfDataWanted);

        //collect response and store in char array
        getResponse();

        //convert char response to string, and extract the data from
        //the string. Store data in member variable
        processResponse_AC();
    }

    //request for AB~ (set encoder zero position)
    if(typeOfDataWanted != "AC~")
    {
        std::cout << packageDescriptor << "You are not using the correct argument for the command " << typeOfDataWanted << std::endl;
    }

    return m_limitSwitchIsOK_AC;
}

//this method is a public method the user can call after creating the object.
//it runs the sequence of sending a request to the arduino, recieving the
//request, and then processing the request and storing all the data in member
//variables. The user can now decide what member variable they are interested in.
//6+++++++++++++++++++++YOU++++++++++++++++++
int LidarStepperPulses::getDataForCommand_AD(std::string typeOfDataWanted)
{
    //initialize all the variables that will hold arduino responses.
    clearAllProcessedResponseVariables();

    //request for AA~ (check comm)
    if(typeOfDataWanted == "AD~")
    {
        //send the request
        sendRequest(typeOfDataWanted);

        //collect response and store in char array
        getResponse();

        //convert char response to string, and extract the data from
        //the string. Store data in member variable
        processResponse_AD();
    }

    //request for AB~ (set encoder zero position)
    if(typeOfDataWanted != "AD~")
    {
        std::cout << packageDescriptor << "You are not using the correct argument for the command " << typeOfDataWanted << std::endl;
    }

    return m_moveHomeCommand_AD;
}

//this method is a public method the user can call after creating the object.
//it runs the sequence of sending a request to the arduino, recieving the
//request, and then processing the request and storing all the data in member
//variables. The user can now decide what member variable they are interested in.
//6+++++++++++++++++++++YOU++++++++++++++++++
int LidarStepperPulses::getDataForCommand_AE(std::string typeOfDataWanted)
{
    //initialize all the variables that will hold arduino responses.
    clearAllProcessedResponseVariables();

    //request for AA~ (check comm)
    if(typeOfDataWanted == "AE~")
    {
        //send the request
        sendRequest(typeOfDataWanted);

        //collect response and store in char array
        getResponse();

        //convert char response to string, and extract the data from
        //the string. Store data in member variable
        processResponse_AE();
    }

    //request for AB~ (set encoder zero position)
    if(typeOfDataWanted != "AE~")
    {
        std::cout << packageDescriptor << "You are not using the correct argument for the command " << typeOfDataWanted << std::endl;
    }

    return m_sweepFrontCommand_AE;
}

//this method is a public method the user can call after creating the object.
//it runs the sequence of sending a request to the arduino, recieving the
//request, and then processing the request and storing all the data in member
//variables. The user can now decide what member variable they are interested in.
//6+++++++++++++++++++++YOU++++++++++++++++++
int LidarStepperPulses::getDataForCommand_AF(std::string typeOfDataWanted)
{
    //initialize all the variables that will hold arduino responses.
    clearAllProcessedResponseVariables();

    //request for AA~ (check comm)
    if(typeOfDataWanted == "AF~")
    {
        //send the request
        sendRequest(typeOfDataWanted);

        //collect response and store in char array
        getResponse();

        //convert char response to string, and extract the data from
        //the string. Store data in member variable
        processResponse_AF();
    }

    //request for AB~ (set encoder zero position)
    if(typeOfDataWanted != "AF~")
    {
        std::cout << packageDescriptor << "You are not using the correct argument for the command " << typeOfDataWanted << std::endl;
    }

    return m_sweepBackCommand_AF;
}

//this method is a public method the user can call after creating the object.
//it runs the sequence of sending a request to the arduino, recieving the
//request, and then processing the request and storing all the data in member
//variables. The user can now decide what member variable they are interested in.
//6+++++++++++++++++++++YOU++++++++++++++++++
int LidarStepperPulses::getDataForCommand_AG(std::string typeOfDataWanted)
{
    //initialize all the variables that will hold arduino responses.
    clearAllProcessedResponseVariables();

    //request for AA~ (check comm)
    if(typeOfDataWanted == "AG~")
    {
        //send the request
        sendRequest(typeOfDataWanted);

        //collect response and store in char array
        getResponse();

        //convert char response to string, and extract the data from
        //the string. Store data in member variable
        processResponse_AG();
    }

    //request for AB~ (set encoder zero position)
    if(typeOfDataWanted != "AG~")
    {
        std::cout << packageDescriptor << "You are not using the correct argument for the command " << typeOfDataWanted << std::endl;
    }

    return m_moveToHorizontalFrontCommand_AG;
}

//BOILERPLATE code to fetch private data. Make sure you already ran the
//getDataFromArduino(), as it makes sure the data all the methods below
//fetch is the latest data available.
//8+++++++++++++++++++++YOU++++++++++++++++++
/*int LidarMagneticEncoder::getDegreeData()
{
    return m_dataFromArduino_degree;
}

int LidarMagneticEncoder::getUncompPositionData()
{
    return m_dataFromArduino_uncompPosition;
}

bool LidarMagneticEncoder::getArduinoCommStatus()
{
    return m_commToArduinoOk;
}

int LidarMagneticEncoder::getZeroPositionRequestStatus()
{
    return m_dataFromArduino_zeroRequest;
}
*/


//===========================================================================================
//==========================PRIVATE METHODS===================================================
//===========================================================================================

//NOTE all the arguments are enums and are very specific. See:
//see: https://github.com/crayzeewulf/libserial/blob/master/src/libserial/SerialPortConstants.h
void LidarStepperPulses::initializeSerialPort(LibSerial::SerialPort *serialPortObj)
{
    try
    {
        //open a serial port
        serialPortObj->Open(serial_port_path);
    }
    //syntax for catching all exceptions is from:
    //https://www.learncpp.com/cpp-tutorial/144-uncaught-exceptions-catch-all-handlers-and-exception-specifiers/
    catch(LibSerial::OpenFailed)
    {
        std::cerr << "cant open port! Exiting did you: " << std::endl;
        std::cerr << "(1) plug the device in to PC? Is VM running?" << std::endl;
        std::cerr << "(2) $sudo chmod 666 ttyACM0?" << std::endl;
        exit(1) ;
    }
    catch(...)
    {
        std::cerr << "some issue other than opening port...";
        exit(1);
    }

    // Set the baud rate. Note that this library does not support a baud rate of 115200. for
    //Baud rates see: https://github.com/crayzeewulf/libserial/blob/master/src/libserial/SerialPortConstants.h
    serialPortObj->SetBaudRate(LibSerial::BaudRate::BAUD_9600);

    // Set the desired character size (data bits). See SerialPort.h for options.
    // I am opting to use the value of 8 data bits.
    serialPortObj->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8 );

    // Set the desired parity type. See SerialPort.h for options. I am opting to
    // use  no error checking. Parity basically adds a bit to make every
    // "packet" even or odd when all the bits are added. If the "packet" arrives
    // even when odd is selected we can deduce corruption... I am not using this.
    serialPortObj->SetParity( LibSerial::Parity::PARITY_NONE );

    // Set the number of stop bits. This is a bit which signals the end of a "packet".
    serialPortObj->SetStopBits( LibSerial::StopBits::STOP_BITS_1 );

    //hardware flow control allows two devices to synchronize communications. I am
    // not using this option.
    serialPortObj->SetFlowControl( LibSerial::FlowControl::FLOW_CONTROL_NONE );

    if ( !(serialPortObj->IsOpen()) )
    {
        std::cerr << "Issue configuring serial port settings! Exiting.... " << std::endl;
        exit(1) ;
    }
}


// The send method sends the command out using a string.
//7+++++++++++++++++++++YOU++++++++++++++++++
//add all your message types here.
void LidarStepperPulses::sendRequest(std::string messageSelection)
{
    //check communication with arduino
    if(messageSelection == "AA~")
    {
        //push command to the serial port.
        my_serial_port.Write(messageSelection);
        std::cout << packageDescriptor << messageSelection << " Sent" << std::endl;
    }

    //assign a zero value in the encoder for the current position.
    if(messageSelection == "AB~")
    {
        //push the array into the serial port.
        my_serial_port.Write(messageSelection);
        std::cout << packageDescriptor << messageSelection << " Sent" << std::endl;
    }

    //get the degree data and the uncomp position data
    if(messageSelection == "AC~")
    {
        //push the array into the serial port.
        my_serial_port.Write(messageSelection);
        std::cout << packageDescriptor << messageSelection << " Sent" << std::endl;
    }

        //check communication with arduino
    if(messageSelection == "AD~")
    {
        //push command to the serial port.
        my_serial_port.Write(messageSelection);
        std::cout << packageDescriptor << messageSelection << " Sent" << std::endl;
    }

    //assign a zero value in the encoder for the current position.
    if(messageSelection == "AE~")
    {
        //push the array into the serial port.
        my_serial_port.Write(messageSelection);
        std::cout << messageSelection << " Sent" << std::endl;
    }

    //get the degree data and the uncomp position data
    if(messageSelection == "AF~")
    {
        //push the array into the serial port.
        my_serial_port.Write(messageSelection);
        std::cout << packageDescriptor << messageSelection << " Sent" << std::endl;
    }

        //get the degree data and the uncomp position data
    if(messageSelection == "AG~")
    {
        //push the array into the serial port.
        my_serial_port.Write(messageSelection);
        std::cout << packageDescriptor << messageSelection << " Sent" << std::endl;
    }
    //==================NOTE: ADD MORE MESSAGES HERE=======================
    //EXAMPLE:
    //9+++++++++++++++++++++YOU++++++++++++++++++
    /*
    //user wants to send "AD~"
    if(messageSelection == "AD~")
    {
        //push the array into the serial port.
        my_serial_port.Write(messageSelection);
        std::cout << messageSelection << " Sent\n" << std::endl;
    }
    */
}//sendRequest method


//wipe clean all the variables that hold our final data
void LidarStepperPulses::clearAllProcessedResponseVariables()
{
    //10+++++++++++++++++++++YOU++++++++++++++++++
    m_commToArduinoOk_AA = -2;
    m_platformIsHome_AB = -2;
    m_limitSwitchIsOK_AC = -2;
    m_moveHomeCommand_AD = -2;
    m_sweepFrontCommand_AE = -2;
    m_sweepBackCommand_AF = -2;
    m_moveToHorizontalFrontCommand_AG = -2;
}


//method to read response from driver/buffer. It now has a timeout if the serial port
//has no data to return.
void LidarStepperPulses::getResponse()
{
    //wipe all the data from previous readings
    clearAllProcessedResponseVariables();

    //clear the vector of any old responses it is holding
    for(int i = 0; i < (MAX_RESPONSE_SIZE); i++)
    {
        m_arduinoResponse[i] = 0;
    }

    char tempChar; //a temporary storage location for data that we read

    int indexer = 0; //a means to index the char array storing the response.

    int serialCommReadTimeout_ms = 500; //as the name states value is in milliseconds.

    //I will break out of this loop only if i get a ~ character (0x7E). I don't
    //like this...
    while(true)
    {
        //get the first byte available from the port. This is a blocking call.
        try
        {
            //blocking, but at least we have a timeout.
            my_serial_port.ReadByte(tempChar, serialCommReadTimeout_ms);
        }
        catch(LibSerial::ReadTimeout) //if we time out, then we will know.
        {
            std::cerr << "Encountered a timeout" << std::endl;
            break;
        }
        catch(...)
        {
            std::cerr << "Serial port has a non-timeout error. Uh-oh!" << std::endl;
        }

        //push the new value into the response array.
        m_arduinoResponse[indexer] = tempChar;

        usleep(3);

        if (tempChar == 0x7E)
        {
            //========COMMENT OUT TO SEE FULL RESPONSE. this varifies that the
            //arduino is working correcly.
            break;
        }

        //we are about to overflow the array holding the response
        if(indexer == (MAX_RESPONSE_SIZE-1))
        {
            break;
        }

        indexer++;
    }

    /*
    for(int i=0; i < MAX_RESPONSE_SIZE; i++)
    {
        std::cout << m_arduinoResponse[i] << std::endl;
    } */
}

//process the string response from the arduino and store in values. You have to populate this method
//with the right amount of calls to the correct methods.
//11+++++++++++++++++++++YOU++++++++++++++++++
void LidarStepperPulses::processResponse_AA()
{
    //fill string with the vector data we got as a response. This is needed in the
    //next function.
    populateArduinoResponseStringFromVector();

    //process the int value we recieved and push it into memory
    processStringToIntAndAssignToMemberVariables(m_arduinoResponseAsString, "A", "B", m_commToArduinoOk_AA);
}


//process the string response from the arduino and store in values. You have to populate this method
//with the right amount of calls to the correct methods.
//11+++++++++++++++++++++YOU++++++++++++++++++
void LidarStepperPulses::processResponse_AB()
{
    //fill string with the vector data we got as a response. This is needed in the
    //next function.
    populateArduinoResponseStringFromVector();

    //process the int value we recieved and push it into memory
    processStringToIntAndAssignToMemberVariables(m_arduinoResponseAsString, "A", "B", m_platformIsHome_AB);
}

//process the string response from the arduino and store in values. You have to populate this method
//with the right amount of calls to the correct methods.
//11+++++++++++++++++++++YOU++++++++++++++++++
void LidarStepperPulses::processResponse_AC()
{
    //fill string with the vector data we got as a response. This is needed in the
    //next function.
    populateArduinoResponseStringFromVector();

    //process the int value we recieved and push it into memory
    processStringToIntAndAssignToMemberVariables(m_arduinoResponseAsString, "A", "B", m_limitSwitchIsOK_AC);
}


//process the string response from the arduino and store in values. You have to populate this method
//with the right amount of calls to the correct methods.
//11+++++++++++++++++++++YOU++++++++++++++++++
void LidarStepperPulses::processResponse_AD()
{
    //fill string with the vector data we got as a response. This is needed in the
    //next function.
    populateArduinoResponseStringFromVector();

    //process the int value we recieved and push it into memory
    processStringToIntAndAssignToMemberVariables(m_arduinoResponseAsString, "A", "B", m_moveHomeCommand_AD);
}

//process the string response from the arduino and store in values. You have to populate this method
//with the right amount of calls to the correct methods.
//11+++++++++++++++++++++YOU++++++++++++++++++
void LidarStepperPulses::processResponse_AE()
{
    //fill string with the vector data we got as a response. This is needed in the
    //next function.
    populateArduinoResponseStringFromVector();

    //process the int value we recieved and push it into memory
    processStringToIntAndAssignToMemberVariables(m_arduinoResponseAsString, "A", "B", m_sweepFrontCommand_AE);
}


//process the string response from the arduino and store in values. You have to populate this method
//with the right amount of calls to the correct methods.
//11+++++++++++++++++++++YOU++++++++++++++++++
void LidarStepperPulses::processResponse_AF()
{
    //fill string with the vector data we got as a response. This is needed in the
    //next function.
    populateArduinoResponseStringFromVector();

    //process the int value we recieved and push it into memory
    processStringToIntAndAssignToMemberVariables(m_arduinoResponseAsString, "A", "B", m_sweepBackCommand_AF);
}

//process the string response from the arduino and store in values. You have to populate this method
//with the right amount of calls to the correct methods.
//11+++++++++++++++++++++YOU++++++++++++++++++
void LidarStepperPulses::processResponse_AG()
{
    //fill string with the vector data we got as a response. This is needed in the
    //next function.
    populateArduinoResponseStringFromVector();

    //process the int value we recieved and push it into memory
    processStringToIntAndAssignToMemberVariables(m_arduinoResponseAsString, "A", "B", m_moveToHorizontalFrontCommand_AG);
}


//push the vector data holding the arduino response into a string.
void LidarStepperPulses::populateArduinoResponseStringFromVector()
{
    //create a string from our vector of chars.
    std::string tempString(m_arduinoResponse.begin(), m_arduinoResponse.end());

    //pass the data
    m_arduinoResponseAsString = tempString;
}


//This class method receives the full Arduino response, and extracts the int value of interest
//that is nested in between two letters. The value is then stored in the appropriate
//member variable. For passing by reference see:
//https://stackoverflow.com/questions/18147038/passing-object-by-reference-in-c
void LidarStepperPulses::processStringToIntAndAssignToMemberVariables(std::string allData, std::string start, std::string finish, int &memberVariable)
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

    //variable to hold the arduino value
    int finalValue = 0;

    //convert the string value to an integer. See claudios answer in:
    //https://stackoverflow.com/questions/7663709/how-can-i-convert-a-stdstring-to-int#7664227
    //also see: http://www.cplusplus.com/reference/string/stoi/
    try
    {
        finalValue = std::stoi(stringResult);
    }
    catch(...)
    {
        std::cout << "ERROR: arduino response had bad data (non numeric chars found)"
        ", or Char data could not be converted to int as it was missing)" << std::endl;
    }

    //assign the integer I recovered to the global variable to which it belongs
    memberVariable = finalValue;
}

//This class method receives the full Arduino response, and extracts the float value of interest
//that is nested in between two letters. The value is then stored in the appropriate
//member variable. For passing by reference see:
//https://stackoverflow.com/questions/18147038/passing-object-by-reference-in-c
//for passing by reference see: https://stackoverflow.com/questions/18147038/passing-object-by-reference-in-c
void LidarStepperPulses::processStringToDoubleAndAssignToMemberVariables(std::string allData, std::string start, std::string finish, double &memberVariable)
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

    //variable to hold the arduino value
    double finalValue = 0.0;

    //convert the string value to an integer. See claudios answer in:
    //https://stackoverflow.com/questions/7663709/how-can-i-convert-a-stdstring-to-int#7664227
    //also see: http://www.cplusplus.com/reference/string/stoi/
    try
    {
        //std::cout << m_arduinoResponseAsString << std::endl;
        //std::cout << stringResult << std::endl;
        finalValue = std::stold(stringResult);
    }
    catch(...)
    {
        std::cout << "ERROR: arduino response had bad data (non numeric chars found)"
        ", or Char data could not be converted to int as it was missing)" << std::endl;
    }

    //assign the integer I recovered to the global variable to which it belongs
    memberVariable = finalValue;
}

