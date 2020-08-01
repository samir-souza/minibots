#ifndef __ANALYTICS_HPP 
#define __ANALYTICS_HPP

#include <sstream>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <logger.hpp>
#include <thread>
#include <config.hpp>

#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>	// For sleep
#include <atomic>
#include <chrono>
#include <cstring>

#define ANLY Analytics::get()

/**
*   Singleton Analytics Class.
*/
class Analytics {
public:
    enum TOPIC {
        POSE,
        TELEMETRY,
        TRAFFIC_LIGHT,
        BOT_PATH
    };

    struct Payload {
	    int botId; 
	    std::string data;
    };

    void sendRecord( TOPIC streamType, const Payload& payload ) {
        std::string key = std::to_string(payload.botId) + "." + std::to_string(streamType);

        this->records[streamType].push_back( payload );
        if ( this->records[streamType].size() > this->recordsBufferSize ) {
            processRecords(streamType);
            LOG(Logger::INFO, "Analytics::sendRecord - Another batch sent to kinesis" );
        } // if
    }


    /**
    *   Funtion to create the instance of config class.
    *   @return singleton object of Cconfig class..
    */
    static Analytics* get() {
        if (m_pThis == NULL){
            m_pThis = new Analytics();
        }
        return m_pThis;
    }

    inline ~Analytics( void ) {
    }
    inline long long int nanos(void) {
        return std::chrono::duration_cast< std::chrono::nanoseconds >( std::chrono::system_clock::now().time_since_epoch() ).count();
    }

    inline long long int millis(void) {
        return std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::system_clock::now().time_since_epoch() ).count();
    }

private:
    
    std::unordered_map<TOPIC, std::vector<Payload> > records;

    void sendRecords(TOPIC streamName, std::vector<Payload> payloads ) {
        /*
            std::ostringstream data;
            data << p.botId << "|" << p.data;
            std::string body = data.str();
            msg.payload = const_cast<char*>( body.c_str() );
            msg.payloadlen = body.length();
            msg.qos = 0; // Fire and forget - the message may not be delivered 
            msg.retained = 0; // 
        */
    }

    void processRecords(TOPIC streamName) {
        try {
            if ( this->processSendRecords.joinable() ) {
                long long elapsedTime = millis();
                this->processSendRecords.join();
                LOG( Logger::ERROR, "Analytics::processRecords - Joining thread for: %dms",  (millis() - elapsedTime) );
            } // if
            
            this->payloads = this->records[streamName];
            this->records[streamName].clear();
            LOG( Logger::INFO, "Analytics::processRecords - Payloads cleaned!" );
            
            this->processSendRecords = std::thread(&Analytics::sendRecords, this, 
                streamName, std::ref(payloads) );
        } catch ( const std::system_error& e ) {
            LOG( Logger::INFO, "Analytics::processRecords - Something went wrong %s", e.what() );
        } // catch
    } // putRecord

    /**
    *    Default constructor for the Analytics class.
    */
    inline Analytics() {
        this->recordsBufferSize = CONF(unsigned, "recordsBufferSize");
    }
    /**
    *   copy constructor for the Analytics class.
    */
    Analytics(const Analytics&){};             // copy constructor is private
    /**
    *   assignment operator for the Analytics class.
    */
    Analytics& operator=(const Analytics&){ return *this; };  // assignment operator is private

    /**
    *   Singleton config class object pointer.
    **/
    inline static Analytics* m_pThis = NULL;
    unsigned recordsBufferSize; 
    std::vector<std::string> TOPIC_NAMES;
    std::thread processSendRecords;
    std::vector<Payload> payloads;
};
#endif // __ANALYTICS_HPP
