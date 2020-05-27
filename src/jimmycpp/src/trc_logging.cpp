/*
    Purpose: this tiny library is meant to be a logging library. It only uses the
             std library to achieve it's task. If you are thinking of using this
             then you haven't looked into ros logging: https://wiki.ros.org/roscpp/Overview/Logging

    Author: TorontoRoboticsClub@protonmail.com (MO)

    Date: March 9, 2019

    Key Note: this project will not build unless you are using 5.X gcc and newer. See:
              https://stackoverflow.com/questions/43088414/error-use-of-deleted-function-stdbasic-ofstreamchar-stdbasic-ofstreamc

    Using Library:
    (1)  add the header:
         #include "trc_logging.h"

    (2)  create a logger object:
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
         jimmycpptest::Logging myObj {"new_test.txt", "SCREEN_AND_FILE", "INFO"};

    (3) use the public method to create log entries:
        //arg 1 - severity level of this log entry: DEBUG, INFO, WARN, ERROR, FATAL
        //arg 2 - line number in source file
        //arg 3 - code purpose. Ex: networking, initialization, message assembly, etc...
        //arg 4 - message for user
        myObj.buildLogMessage("DEBUG", 35, "NETWORKING", "Here comes level 0");

    Design considerations:
    (1) Log file name - initially I decided that I am going to use the same name for
        all the files I create. meaning that the path (including the name) is hardcoded.
        However, when I make an object from the library, I will have problems (multiple
        logging code writing to the same file; and that is assuming streams can share
        files).
        >>>>>> DECISION: fix this mess by passing a file name to the constructor, and have
                         the code create the appropriate file on the desktop.

    (2) Logging location- should I push everything to a file, or the screen, or both?
        >>>> decision: initially I wanted to do both. After some consideration I decided to
        implement a constructor with a second argument which defines the output location.the
        options are: file, screen, both. The default is both.

    (3) Filter - since this is a troubleshooting tool, I need to be able to narrow down
        the messages that I am seeing.
        >>>> After some consideration I decided to implement the following:
        (a) Since I am going to use this tiny library with other files, I needed to abstract
            the types in the filter. For example using the string "message assembly" would
            be limited to a single type of node (sending commands to ST10 drives)! I opted
            to copy the message types in AGITR pg 62. ROS has it's own messaging system,
            but I was already pot commited to my little library.
        (b) The file is to recieve the full level of logging regardless of the 3rd constructor
            argument. However, the 3rd argument into the constructor will dictate the lowest
            alarm level to print to the screen (which includes all other alarms that are more
            severe). This system will allow me to ignore certain messages on the screen (the
            messages that have a lower logging level than the 3rd constructor argument).

    References:
    (1) How to create a file in a specific location (ex: desktop):
        https://stackoverflow.com/questions/3359627/create-file-on-desktop-in-c
    (2) Turns out that dates and times are a huge issue with C++. See:
        https://stackoverflow.com/questions/50521052/get-the-current-date-and-time-with-milliseconds-with-only-the-standard-library-i
        also see:
        https://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c
    (3) putting default values in the constructor definition. See:
        https://www.learncpp.com/cpp-tutorial/85-constructors/
    (4) this project will not build unless you are using 5.X gcc and newer. See:
        https://stackoverflow.com/questions/43088414/error-use-of-deleted-function-stdbasic-ofstreamchar-stdbasic-ofstreamc

*/

#include "jimmycpp/trc_logging.h" //my header file
#include <chrono>  //for generating date and time
#include <fstream> //for creating and writing to a file
#include <vector>  //for storing severity level types

namespace jimmycpp
{

	//this is a definition of the class constructor outside of the class. In
   //this case we have ClassName::Constructor() to be able to define the item.
	Logging::Logging(std::string fileName, std::string outputLocation, std::string screenLoggingLevel)
	{
        //create a path to the home directory (in a Linux/mac system)
        m_path = getenv("HOME");

        //append the string to contain the path to the Desktop
        //!!NOTE: You must put it into a directory that exists!
        m_path = m_path + "/Desktop/";

        //append the string to include the filename that we want on the desktop
        m_path = m_path+fileName;

        //use the ofstream (output file stream) to create the file, and open a
        //connection to the file.
        m_fileOutputStream = std::ofstream(m_path, std::ios_base::app);

        //initialize the struct that is holding all data associated with a
        //single line in the log
        //arg 1 - line number in code
        //arg 2 - message type
        //arg 3 - the actual message for the user/programmer
        myLoggingSet = {"DEBUG", 0, "empty", "empty"};

        //store the output location the user has requested (screen, file, or both).
        //NOTE: default is SCREEN_AND_FILE
        m_outputLocation = outputLocation;

        //store the input from the user for the filtering level to the screen.
        m_screenLoggingLevel = screenLoggingLevel;

        //anytime the logger is fired up, it will append the file. This line
        //makes it clear where a new log starts.
        m_fileOutputStream << "********************************************************************************\n"
        << "New program was launched and is appending log file on: " << dateAndTime()
        << "********************************************************************************" << "\n";

        //log into the file, the reason why the log is not being appended.
        if(m_outputLocation == "SCREEN")
        {
            m_fileOutputStream << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~NOTE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"
            << "The logger was commanded to only log to the screen via logger object constructor\n"
            << "arguments. This execution of this program is not logged into the file.\n"
            << "START of log file blackout: " << dateAndTime();
        }

	}//constructor

    //destructor
    Logging::~Logging()
    {
        //log into the file, the reason why the log is not being appended.
        if(m_outputLocation == "SCREEN")
        {
            m_fileOutputStream << "END of log file blackout:   " << dateAndTime()
            << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END OF NOTE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" << std::endl;
        }

        //anytime the logger is fired up, it will append the file. This line
        //makes it clear where a new log ends.
        //NOTE: std::endl flushes the buffer so I am using it at the end of the ofstream.
        m_fileOutputStream << "********************************************************************************\n"
        << "Program is finished appending log file and exited on: " << dateAndTime()
        << "********************************************************************************" << std::endl;
    }

    //get the date and time as a string
    //turns out that dates and times are a huge issue with C++. See:
    //https://stackoverflow.com/questions/50521052/get-the-current-date-and-time-with-milliseconds-with-only-the-standard-library-i
    //also see:
    //https://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c
    //This method gets a chrono lib time, converts to a c type time and date,
    // and stores it in a string.
    std::string Logging::dateAndTime()
    {
        //get date and time in 'time_point' data type
        std::chrono::system_clock::time_point systemTime = std::chrono::system_clock::now();

        //convert date and time to a 'systemTime_t' data type.
        std::time_t systemTime_t = std::chrono::system_clock::to_time_t(systemTime);

        //store as string
        std::string dateAndTime =  std::ctime(&systemTime_t);

        return dateAndTime;

    }


    // convert the logging level to an integer. Where 0 is least severe (DEBUG), and 4 is
    // the most severe (FATAL). This will allow the logic to handle printing to file and
    // screen less verbose. the return is as follows:
    //-1 = the string in the log entry does not match any of our log severity levels
    // 0 - DEBUG
    // 1 - INFO
    // 2 - WARN
    // 3 - ERROR
    // 4 - FATAL
    int Logging::convertLoggingLevelToInt_HigherIsWorse(std::string loggingLevelOfLogEntry)
    {
        //return value
        int returnVal = -1;

        //if the string is not something we recognize, then we will find out about it here
        if(loggingLevelOfLogEntry!="DEBUG" || loggingLevelOfLogEntry!="INFO"|| loggingLevelOfLogEntry!="WARN" ||
           loggingLevelOfLogEntry!="ERROR" || loggingLevelOfLogEntry!="FATAL")
        {
            returnVal = -1;
        }

        //This vector stores the 5 alarm levels from least severe vector[0] to most severe vector[4]
        std::vector<std::string> myVect = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

        //compare the string in the log entry to our vector to figure out the severity level
        for(unsigned int i = 0; i < myVect.size(); i++)
        {
            if(myVect[i] == loggingLevelOfLogEntry)
            {
                returnVal = i;//return severity level
            }
        }

        return returnVal;
    }


    //this method will print the log entry to the screen.
    void Logging::writeLogEntryToScreen(struct m_LoggingSet myStruct)
    {
        std::cout << "[" << myStruct.loggingLevel << "] " << dateAndTime() << "Line " << myStruct.codeBlocksLineNumber << " - "
                  << myStruct.logType << " - " << myStruct.messageContent << "\n";
    }

    //this method writes the log entry to the file
    void Logging::writeLogEntryToFile(struct m_LoggingSet myStruct)
    {
        m_fileOutputStream << "[" << myStruct.loggingLevel << "] " << dateAndTime() << "Line " << myStruct.codeBlocksLineNumber << " - "
                  << myStruct.logType << " - " << myStruct.messageContent << std::endl;
    }

     //an easy way to build the message. This function is public. This is how people using this library will
    //pass log info to the library. See the notes at top of file for details.
    void Logging::buildLogMessage(std::string loggingLevel, int lineNumber, std::string messageType, std::string message)
    {
        myLoggingSet.loggingLevel = loggingLevel;
        myLoggingSet.codeBlocksLineNumber = lineNumber; //line number where the message was generated
        myLoggingSet.logType = messageType; //initialization, network, message assembly, etc....
        myLoggingSet.messageContent = message; //the actual message.

        //with the log data aquired, it's time to write it out.
        writeOutLogEntry(myLoggingSet);
    }

    //this method will filter the log entry based on the constructor arguments, and write the appropriate
    //log entry to the appropriate location
    void Logging::writeOutLogEntry(struct m_LoggingSet myStruct)
    {
        //these variables make sure we dont print the same log twice
        bool doNotPrintToScreenAgain = false;
        bool doNotPrintToFileAgain = false;

        //output to screen only
        if(m_outputLocation == "SCREEN")
        {
            //the return of the method is the filtering level the user has selected. Can be:
            //-1 = the string in the log entry does not match any of our log severity levels
            // 0 - DEBUG
            // 1 - INFO
            // 2 - WARN
            // 3 - ERROR
            // 4 - FATAL
            int logEntrySeverityLevel = convertLoggingLevelToInt_HigherIsWorse(myStruct.loggingLevel);
            int userDefinedFilteringLevel = convertLoggingLevelToInt_HigherIsWorse(m_screenLoggingLevel);

            if(logEntrySeverityLevel >= userDefinedFilteringLevel)
            {
                writeLogEntryToScreen(myStruct);
                doNotPrintToScreenAgain = true;
            }
            doNotPrintToFileAgain = true; //as we are only printing to screen
        }//end of SCREEN

        //output to file only
        if(m_outputLocation == "FILE")
        {
            writeLogEntryToFile(myStruct);
            doNotPrintToFileAgain = true;
            doNotPrintToScreenAgain = true;//as we are only printing to the file
        }//end of FILE

        //if we did not write the log entry out to the screen or a file, then we better do this now!
        if((doNotPrintToScreenAgain==false && doNotPrintToFileAgain==false) || (m_outputLocation == "SCREEN_AND_FILE"))
        {
            //we do not filter the logs that are written to the file
            writeLogEntryToFile(myStruct);

            //the return of the method is the filtering level the user has selected. Can be:
            //-1 = the string in the log entry does not match any of our log severity levels
            // 0 - DEBUG
            // 1 - INFO
            // 2 - WARN
            // 3 - ERROR
            // 4 - FATAL
            int logEntrySeverityLevel = convertLoggingLevelToInt_HigherIsWorse(myStruct.loggingLevel);
            int userDefinedFilteringLevel = convertLoggingLevelToInt_HigherIsWorse(m_screenLoggingLevel);

            if(logEntrySeverityLevel >= userDefinedFilteringLevel)
            {
                writeLogEntryToScreen(myStruct);
            }
        }
    }

}//namespace

