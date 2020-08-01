#include <tracker.hpp>
#include <logger.hpp>
#include <config.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core.hpp>

namespace pgpri = powergine::primitives;

Tracker::Tracker( int width, int height, int camid,
                int fps, bool preview, std::string serverAddress ) :
    running(false),sizeOfMarker(0.0375),imsize(width,height),cap(NULL),
    dist(cv::Mat::zeros(5,1, CV_32F)),
    mtx(cv::Mat::zeros(3,3, CV_32F)),newMtx(cv::Mat::zeros(3,3, CV_32F)),
    map(width,height, preview), td(apriltag_detector_create()), tf(tagStandard41h12_create()),
    server("0.0.0.0", 5000 ),
    preview(preview), socketId(-1) {

    apriltag_detector_add_family(this->td, this->tf);
    if ( serverAddress == "" ) {
        this->cap = new cv::VideoCapture(camid);
        this->cap->set(cv::CAP_PROP_FPS, fps);
        this->cap->set(cv::CAP_PROP_FRAME_WIDTH, width );
        this->cap->set(cv::CAP_PROP_FRAME_HEIGHT, height);
        this->cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('B','G','R','3'));

        cv::Mat sample;
        this->cap->read(sample);
        if ( !sample.empty() ) {
            cv::imwrite("sample.jpg", sample );
        } // if
    } else {
        std::replace(serverAddress.begin(), serverAddress.end(), ':', ' ');
        std::stringstream ss(serverAddress);
        std::string host;
        std::string port;
        ss >> host >> port;

        struct addrinfo hints;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_protocol = IPPROTO_UDP;

        int r = getaddrinfo(host.c_str(), port.c_str(), &hints, &this->addrInfo);
        if( r != 0 || this->addrInfo == NULL) {
            throw std::runtime_error(("invalid address or port for UDP socket: \"" + host + ":" + port + "\"").c_str());
        } // if

        this->socketId = socket(this->addrInfo->ai_family, SOCK_DGRAM, IPPROTO_UDP);
        fcntl(this->socketId, O_CLOEXEC);
        if(this->socketId == -1) {
            freeaddrinfo(this->addrInfo);
            throw std::runtime_error(("could not create UDP socket for: \"" + host + ":" + port + "\"").c_str());
        } // if

        r = bind(this->socketId, this->addrInfo->ai_addr, this->addrInfo->ai_addrlen);
        if(r != 0) {
            freeaddrinfo(this->addrInfo);
            close(this->socketId);
            throw std::runtime_error(("could not bind UDP socket with: \"" + host + ":" + port + "\"").c_str());
        } // if
    } // else
} // Tracker

void Tracker::processFrames() {
    Config::ListOfObjects data;
    std::unordered_map<int, std::unordered_map<std::string, std::string>> obstacles, bots, places, manualTargets;
    data = CONF_OBJS("bots");
    for ( auto& i : data ) bots.insert( std::make_pair( std::atoi(i["id"].c_str()), i ) );
    data = CONF_OBJS("obstacles");
    for ( auto& i : data ) obstacles.insert( std::make_pair( std::atoi(i["id"].c_str()), i ) );
    data = CONF_OBJS("places");
    for ( auto& i : data ) places.insert( std::make_pair( std::atoi(i["id"].c_str()), i ) );
    data = CONF_OBJS("manualTargets");
    for ( auto& i : data ) manualTargets.insert( std::make_pair( std::atoi(i["id"].c_str()), i ) );

    float botHeight = CONF(float, "botHeight");
    float obstacleHeight = CONF(float, "obstacleHeight");
    float cameraHeight = CONF(float, "cameraHeight");

    float mapWidth = CONF(float, "width");
    float mapHeight = CONF(float, "height");

    pgpri::Vector3D cameraPos( mapWidth/2.0, mapHeight/2.0, cameraHeight );
    
    long long elapsedTrafficLightTime = 0;
    while( this->running ) {
        long long int startTime = ANLY->millis();
        cv::Mat newFrame;
        // let's grab a new frame to process. 
        // First let's get the lock, clone the frame and unlock
        this->frameLock.lock(); 
        newFrame = this->frame.clone();
        this->frameLock.unlock();
        
        // if the frame is empty (maybe the thread is starting) sleep
        if ( newFrame.empty() ) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
            continue;
     	} // if

        // ok. now let's look for april tags        
        cv::Mat gray;
        cv::cvtColor(newFrame, gray, cv::COLOR_BGR2GRAY);
        // Make an image_u8_t header for the Mat data
        image_u8_t im = { 
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };
        zarray_t *detections = apriltag_detector_detect(this->td, &im);        

        // Iterate through all tags and update the map with the orientation/position
        int numberOfTags = zarray_size(detections);
        for (int i = 0; i < numberOfTags; ++i) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            bool isBot = false, isPlace = false, isObstacle = false, isManualTarget = false;

            if ( !( isBot = bots.find(det->id) != bots.end() ) ) {
                if ( !( isPlace = places.find(det->id) != places.end() ) ) {
                    if ( !( isManualTarget = manualTargets.find(det->id) != manualTargets.end() ) ) {
                        if ( !( isObstacle = obstacles.find(det->id) != obstacles.end() ) ) {
                            LOG( Logger::ERROR, "Unkown object. Ignoring. id %d", det->id );
                            continue;
                        } // if
                    } // if
                } // if
            } // if
    
            // get the tag center and the corners to identify the pos/ori
            cv::Point2f pos(det->c[0], det->c[1] );
            cv::Point2f cornerA(det->p[0][0], det->p[0][1]);
            cv::Point2f cornerB(det->p[3][0], det->p[3][1]);
            cv::Point2f dir = cornerB - cornerA;
            // reduce the orientation vector to its unit
            float mag = cv::norm( dir);
            dir.x /= mag;
            dir.y /= mag;

            if ( isPlace ) {
                this->map.registerPlace(det->id, places[det->id]["name"], pos, dir );                
                // places, not agents
                continue;
            } // if


            float entityHeight = isBot ? botHeight : obstacleHeight;

            // OK, it's a moving entity, so let's  fix it's position
            pgpri::Vector3D botPos( pos.x, pos.y, 0 );
            pgpri::Vector3D sightLine = botPos - cameraPos;
            
            pgpri::Vector3D fixedPos = sightLine;
            fixedPos.normalize();
            fixedPos = sightLine - (fixedPos * entityHeight) + cameraPos;
            pos.x = fixedPos.getX();
            pos.y = fixedPos.getY();

            if ( isManualTarget ) {
                this->map.visibleManualTarget( det->id, manualTargets[det->id]["name"], pos, dir, ANLY->millis() );
                continue;
            } // if

            if ( isObstacle ) {
                this->map.visibleObstacle( det->id, obstacles[det->id]["name"], pos, dir, ANLY->millis() );
                // obstacle
                continue;
            } // else if
            
            bool alreadySeen = this->map.isBotRegistered( det->id );
            this->map.updateAgent( det->id, pos, dir, ANLY->millis() );
            if ( this->server.isClientRegistered( det->id) ) {
                Map::Agent& agent = this->map.getAgent( det->id );
                
                if ( !alreadySeen ) {
                    int botId = agent.getId( );
                    std::stringstream ss;
                    ss << "o;" << botId << ";"
                       << CONF_BOT(botId, "mpuInit") << ";"
                       << CONF_BOT(botId, "leftFreq") << ";"
                       << CONF_BOT(botId, "leftPeriod") << ";"
                       << CONF_BOT(botId, "leftStopped") << ";"
                       << CONF_BOT(botId, "leftForward") << ";"
                       << CONF_BOT(botId, "leftBackward") << ";"
                       << CONF_BOT(botId, "rightFreq") << ";"
                       << CONF_BOT(botId, "rightPeriod") << ";"
                       << CONF_BOT(botId, "rightStopped") << ";"
                       << CONF_BOT(botId, "rightForward") << ";"
                       << CONF_BOT(botId, "rightBackward") << ";"
                       << CONF_BOT(botId, "speed1") << ";"
                       << CONF_BOT(botId, "speed2") << ";"
                       << CONF_BOT(botId, "speed3");
                    this->server.sendMessage( botId, ss.str() );
                        
                } // if
                pos = agent.getPosition();
                if ( pos != agent.getLastSent( Map::Agent::POSITION ) ) {
                    std::stringstream ss;
                    ss << "p;" << pos.x << ";" << pos.y;
                    this->server.sendMessage( det->id, ss.str() );
                    agent.setLastSent( Map::Agent::POSITION, pos );
                } // if
                dir = agent.getDirection();
                if ( dir != agent.getLastSent( Map::Agent::DIRECTION ) ) {
                    std::stringstream ss;
                    ss << "d;" << dir.x << ";" << dir.y;
                    this->server.sendMessage( det->id, ss.str() );
                    agent.setLastSent( Map::Agent::DIRECTION, dir );
                } // if
                
                cv::Point2f seek;
                if ( agent.isBlocked( ) ) {
                    seek = agent.getPosition();
                    if ( seek != agent.getLastSent( Map::Agent::SEEK ) ) {
                        this->server.sendMessage( det->id, "t;;" );
                        agent.setLastSent( Map::Agent::SEEK, seek );
                    } // if
                } else if ( this->map.computeSeekVector( det->id, seek ) ) {
                    if ( seek != agent.getLastSent( Map::Agent::SEEK ) ) {
                        std::stringstream ss;
                        ss << "s;" << seek.x << ";" << seek.y;
                        this->server.sendMessage( det->id, ss.str() );
                        agent.setLastSent( Map::Agent::SEEK, seek );
                    } // if
                } // if
            } // if 

            if ( this->server.isClientRegistered( 1000 ) && ANLY->millis() - elapsedTrafficLightTime > 3000 ) { // trafficlight
                std::vector<unsigned> trafficLightStatus(8);
                for ( unsigned i=0; i < 8; i++ ) trafficLightStatus[i] = 1; // all green
                std::vector<int>::const_iterator itBe;
                std::vector<int> blockedEdges = this->map.getBlockedEdges( );
                for ( itBe = blockedEdges.begin(); itBe != blockedEdges.end(); ++itBe ) {
                    const std::vector<std::pair<int, int>>& affectedPoles = this->map.getAffectedPoles( *itBe );
                    std::vector<std::pair<int, int>>::const_iterator itAp;
                    for ( itAp = affectedPoles.begin(); itAp != affectedPoles.end(); ++itAp ) {
                        trafficLightStatus[(itAp->first * 2) + itAp->second] = 2; // set red
                    } // for
                } // for
                std::ostringstream msg;
                for ( unsigned i=0; i < 8; i++ )  msg << ";" << trafficLightStatus[i];
                
                Analytics::Payload trafLightStatus;
                trafLightStatus.botId = 1000;
                trafLightStatus.data = msg.str();
                ANLY->sendRecord( Analytics::TRAFFIC_LIGHT, trafLightStatus ); 

                this->server.sendMessage( 1000, "t" + msg.str() );
                elapsedTrafficLightTime = ANLY->millis();
            } // if
        } // for
        apriltag_detections_destroy(detections);
        // ok, now let's plot some stats
        long long int elapsedTime = (ANLY->millis() - startTime);
        std::cout << "\r\e[KProcess: Elapsed: " << std::setw(5) << elapsedTime << " TAGS: " << numberOfTags 
                  << " FPS: " << std::setw(5) << (int)(1000.0/elapsedTime)
                  << std::flush;
        map.step();
    } // while
}

void Tracker::extractFrames() {
    bool undistort = CONF(bool, "undistort");
    cv::Mat map1, map2;
    if ( undistort ) {
        cv::Mat newK;
        double m1[3][3], m2[4][1];
        {
            std::istringstream parser( CONF( std::string, "camDist") );
            for ( unsigned i=0; i < 3; ++i ) for ( unsigned j=0; j < 3; ++j ) parser >> m1[i][j];
        }
        cv::Mat K = cv::Mat(3,3, CV_64F, m1);
        K.at<double>(2, 2) = 1.0;

        {
            std::istringstream parser( CONF( std::string, "camMtx") );
            for ( unsigned i=0; i < 4; ++i ) parser >> m2[i][0];
        }
        cv::Mat D = cv::Mat(4, 1, CV_64F, m2 );
        
        double balance = 1.0;
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, this->imsize, cv::noArray(), newK, balance);
        cv::fisheye::initUndistortRectifyMap(K, D, cv::noArray(), newK, this->imsize, CV_16SC2, map1, map2 );
    } // if

    // the idea of this method is to run in an infinite loop
    // copy as much as frames as possible to the global Mat frame
    // There is a mutex controling the write process to it
    // if the trylock fails, throw away the new captured frame
    while ( this->running ) {
        cv::Mat newFrame;
        if ( this->socketId != -1 ) {
            char buffer[1024*24];
            int rc = recv( this->socketId, buffer, sizeof(buffer), 0);
            if( rc < 0 ) {
                LOG( Logger::ERROR, "Server::start - Error trying to reveive message" );
                continue;
            } // if
            std::vector<unsigned char> buf;
            buf.assign(buffer, buffer + rc);

            cv::imdecode(cv::Mat(buf), cv::IMREAD_COLOR, &newFrame);
        } else if ( this->cap != NULL && this->cap->isOpened()) {
            (*this->cap) >> newFrame;
        } else {
            continue;
        } // else

        if (this->frameLock.try_lock()) {
            if ( undistort ) {
                cv::remap(newFrame, this->frame, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT );
            } else {
                this->frame = newFrame.clone();
            } // else
            this->frameLock.unlock();
        } // if
    } // while
}

void Tracker::start() {
    Logger::get( )->log( Logger::INFO, "Tracker::%s - Starting tracker. isOpened? [%d]", __func__,
        (this->cap != NULL && this->cap->isOpened() ) );
    if ( this->socketId == -1 && !this->cap->isOpened() ) {
        throw std::runtime_error( "Camera/UDP server not initialized");
    } // if
    // this method will start two threads: 1) frame capturing;
    // 2) frame processing
    this->running = true;
    std::thread extract(&Tracker::extractFrames, this);
    std::thread process(&Tracker::processFrames, this);
    while (this->running && this->preview) {
        this->map.renderMap();
    } // while
    Logger::get( )->log( Logger::INFO, "Tracker::%s - Extractor and Processor initialized...", __func__ );
    extract.join();
    process.join();
    Logger::get( )->log( Logger::INFO, "Tracker::%s - Threads finished", __func__ );

}

void Tracker::stop() {
    this->running = false;
    
}

