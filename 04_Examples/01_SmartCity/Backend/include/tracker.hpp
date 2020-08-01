#ifndef __TRACKER_HPP
#define __TRACKER_HPP

#include <opencv2/opencv.hpp>
#include <thread>
#include <deque>
#include <chrono>
#include <mutex>
#include <functional>
#include <libconfig.h++>
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <map.hpp>
#include <server.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>

class Tracker {
    private:
        bool running;
        float sizeOfMarker; // side length of the marker in meter
        cv::Size imsize;
        cv::VideoCapture *cap;
        cv::Mat frame;
        
        cv::Mat dist;
        cv::Mat mtx,newMtx;

        std::mutex frameLock;
        Map map;
        apriltag_detector_t *td;
        apriltag_family_t *tf;
        Server server;
        bool preview;
        int socketId;
        struct addrinfo *addrInfo;

        void extractFrames();
        void processFrames();

    public:

        Tracker( int width=800, int height=600, int camid = 0,
                    int fps=40, bool preview=false, std::string serverAddress = "" );

        inline ~Tracker() {
            LOG( Logger::INFO, "Tracker::~Tracker - ~Tracker called" );
            this->stop(); 
            apriltag_detector_destroy(this->td);
            tagStandard41h12_destroy(this->tf);
            LOG( Logger::INFO, "Tracker::~Tracker - Finished" );
            if ( this->cap ) {
                delete this->cap;
                this->cap = NULL;
            } // if
            if ( this->socketId != -1 ) {
                freeaddrinfo(this->addrInfo);
                close(this->socketId);
            } // if
        }

        void start();
        void stop();
};

#endif // __TRACKER_HPP
