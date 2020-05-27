/*
    For Purpose, Author, and Date, have a look at "trc_logging.cpp"

    version: 1.00 (first Logging lib)

*/


#ifndef TRC_LOGGING_VERSIONONE_H
#define TRC_LOGGING_VERSIONONE_H
#include <string>
#include <iostream> //for std::cout
#include <fstream> //for ofstream

//the namespace
namespace jimmycpp
{
   //to access the class use this syntax: jimmycpp::AmpSteppers
	class Logging
	{
	    private:

            //a string to hold the path and file name
            std::string m_path;

            //location of where to send each line of logging. Options are:
            //"SCREEN" - log output is shown on the screen
            //"FILE" - log output is sent to a file
            //"SCREEN_AND_FILE" - log output is sent to both the screen and a file. (default setting)
            std::string m_outputLocation;

            //filtering criteria. If the user is looking only for specific types of messages, then
            //they can use this functionality to selet which types of messages can be ignored.
            //default is to keep everything. Options are in increasing order of importance:
            //"DEBUG" - Lowerst in importance.
            //"INFO" -
            //"WARN" -
            //"ERROR" -
            //"FATAL" - highest in importance
            std::string m_screenLoggingLevel;

            // ofstream is used for writing files
            // We'll make a file called Sample.dat
            //arg 1 - path to file wiht name
            //arg 2 - mode of writing to file. See: https://www.learncpp.com/cpp-tutorial/186-basic-file-io/
            //        This arg is append to file. without any arg, the default is overwrite the contents.
            std::ofstream m_fileOutputStream;

            //this struct holds a single entry for a message. This is all the data I have on the error.
            //this provide the means to filter messages. Format is something like this:
	        //std::cout << "date & time " << "code line number " << "message type " << "message content " << std::endl;
	        struct m_LoggingSet
	        {
                std::string loggingLevel; //ALL, DEBUG, INFO, WARN, ERROR, FATAL
                int codeBlocksLineNumber; //the location in the code where the error occured.
                std::string logType; //initialization, network, message assembly, etc....
                std::string messageContent; //the actual message.
	        };

	        //assign memory to this type of object.
            m_LoggingSet myLoggingSet;

            //associate an integer with each logging level
            int convertLoggingLevelToInt_HigherIsWorse(std::string);

	        //method to write the line we got from the user to the standard output
	        void writeLogEntryToScreen(struct m_LoggingSet);

	        //method to write the line we got from the user to a file
	        void writeLogEntryToFile(struct m_LoggingSet);

	        //write the log entry to the locations it needs to go based on
	        //the constructor arguments
	        void writeOutLogEntry(struct m_LoggingSet);


            //get the date and time
	        std::string dateAndTime();

	        bool filterScreenOutputAndPrintToScreen(struct m_LoggingSet);


	    public:
	        //this is a constructor with default values, where:
	        //"SCREEN_AND_FILE" - sends log data to both screen and file
	        //"ALL" - no messages are filtered.
            Logging(std::string, std::string = "SCREEN_AND_FILE", std::string = "WARN");
            ~Logging();

            //method for the user to pass a message over to the logging library
            void buildLogMessage(std::string, int, std::string, std::string);



    };
}


#endif

