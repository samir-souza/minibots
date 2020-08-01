#ifndef __LOGGER_HPP 
#define __LOGGER_HPP
#include <fstream>
#include <iostream>
#include <cstdarg>
#include <string>
#include <ctime>

#define LOG Logger::get()->log

/**
*   Singleton Logger Class.
*/
class Logger {
public:
    enum LEVEL {
        ERROR,
        INFO,
        DEBUG
    };

    /**
    *   Logs a message
    *   @param sMessage message to be logged.
    */
    inline void log(LEVEL level, const std::string& sMessage) {
        m_Logfile <<  Logger::currentDateTime() << " [" << getLevelName(level) << "] \t";
        m_Logfile << sMessage << "\n";
    }
    /**
    *   Variable Length Logger function
    *   @param format string for the message to be logged.
    */
    inline void log(LEVEL level, const char * format, ...) {
        va_list args;
        va_start(args, format);
        char buffer[512];
        std::vsprintf( buffer, format, args);
        va_end(args);

        m_Logfile << Logger::currentDateTime() << " [" << getLevelName(level) << "] \t";
        m_Logfile << buffer << std::endl << std::flush;
    } // log

    /**
    *   Funtion to create the instance of logger class.
    *   @return singleton object of Clogger class..
    */
    static Logger* get() {
        if (m_pThis == NULL){
            m_pThis = new Logger();
            m_Logfile.open(m_sFileName.c_str(), std::ios::out | std::ios::app);
        }
        return m_pThis;
    }
private:
    /**
    *    Default constructor for the Logger class.
    */
    inline Logger() {}
    /**
    *   copy constructor for the Logger class.
    */
    Logger(const Logger&){};             // copy constructor is private
    /**
    *   assignment operator for the Logger class.
    */
    Logger& operator=(const Logger&){ return *this; };  // assignment operator is private
    /**
    *   Log file name.
    **/
    inline static const std::string m_sFileName = "log.txt";
    /**
    *   Singleton logger class object pointer.
    **/
    inline static Logger* m_pThis = NULL;
    /**
    *   Log file stream object.
    **/
    inline static std::ofstream m_Logfile;

    inline static LEVEL m_eLevel = INFO;
 
    inline static const std::string getLevelName(LEVEL level) {
        switch(level) {
            case ERROR: return "ERROR";
            case INFO: return "INFO";
            case DEBUG: return "DEBUG";
            default: return "XXINVALIDXX";
        }
    }

    inline static const std::string currentDateTime() {
        time_t     now = time(NULL);
        char       buf[80];
        const tm  *tstruct = localtime(&now);
        strftime(buf, sizeof(buf), "%Y-%m-%d.%X", tstruct);
        return buf;
    }
};

#endif // __LOGGER_HPP
