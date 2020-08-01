#include <server.hpp>
#include <algorithm>
#include <config.hpp>
#include <analytics.hpp>
#include <fcntl.h>
#include <netdb.h>
//#include <sys/types.h>
//#include <sys/socket.h>

Server::Server( const std::string& host, int port ) {
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    int r = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &this->addrInfo);
    if(r != 0 || this->addrInfo == NULL) {
        throw std::runtime_error(("invalid address or port for UDP socket: \"" + host + ":" + std::to_string(port) + "\"").c_str());
    } // if

    this->socketId = socket(this->addrInfo->ai_family, SOCK_DGRAM, IPPROTO_UDP);
    fcntl(this->socketId, O_CLOEXEC);
    if(this->socketId == -1) {
        freeaddrinfo(this->addrInfo);
        throw std::runtime_error(("could not create UDP socket for: \"" + host + ":" + std::to_string(port) + "\"").c_str());
    } // if

    r = bind(this->socketId, this->addrInfo->ai_addr, this->addrInfo->ai_addrlen);
    if(r != 0) {
        freeaddrinfo(this->addrInfo);
        close(this->socketId);
        throw std::runtime_error(("could not bind UDP socket with: \"" + host + ":" + std::to_string(port) + "\"").c_str());
    } // if

    LOG( Logger::INFO, "Server::Server Starting main thread");
    this->server = std::thread(&Server::start, this );
}

void Server::start( void ) {
    this->running = true;
    //Config::ListOfObjects data = CONF_OBJS("bots");
    //std::unordered_map<std::string, std::unordered_map<std::string, std::string>> bots;
    //for ( auto& i : data ) bots.insert( std::make_pair( i["ip"], i ) );

    while ( this->running ) {
        char buffer[512];
        struct sockaddr_in client;
        socklen_t l = sizeof(client);
        int rc = recvfrom( this->socketId,buffer, sizeof(buffer), 0, (struct sockaddr *)&client,&l);
        if( rc < 0 ) {
            LOG( Logger::ERROR, "Server::start - Error trying to reveive message" );
            continue;
        } // if
        buffer[rc] = '\0';
        std::string clientIp( inet_ntoa(client.sin_addr) );
        int clientPort = client.sin_port;

        LOG( Logger::INFO, "Server::start - Received message [%s] (bytes: %d) from [%s:%d]", buffer, rc, clientIp.c_str(), clientPort );
        std::string msg(buffer);
        std::replace(msg.begin(), msg.end(), ';', ' ');
        std::stringstream ss(msg);

        char type;
        ss >> type;
        switch( type ) {
            case 'i':  {
                int port=-1;
                int botId;//=std::stoi(bots[clientIp]["id"]);
                std::string host;
                ss >> host >> port >> botId;
                LOG( Logger::INFO, "Server::start - Ping from Bot: %d, Host: %s, Port: %d", botId, host.c_str(), port );
                if ( botId != -1 ) {
                    Server::Client clientMeta;
                    clientMeta.active = false;
                    clientMeta.botId = botId;
                    clientMeta.host = host;
                    clientMeta.port = port;
                    this->clients[botId] = clientMeta;
                    this->clientsHostMap[clientIp] = botId;
                    sendMessage(botId, "r;;");
                } // if
            } break;
            case 'l': { // login
                int botId;//=std::stoi(bots[clientIp]["id"]);

                std::string user,pwd; 
                ss >> user >> pwd >> botId;
                LOG( Logger::INFO, "Server::start - New Bot: %d, User: %s", botId, user.c_str() );
                if ( botId != -1 && user == "minibots" && pwd == "aws@123" ) {
                    this->clients[botId].active = true;
                    std::ostringstream data;
                    data << "o;" << botId << ";";
                    if ( botId != 1000 ) { // 1000 = trafficLight
                        data << CONF_BOT(botId, "mpuInit") << ";"
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
                    } // else 
                    sendMessage(botId, data.str() );
                    if ( botId == 1000 ) {
                        std::ostringstream status;
                        status << "t";
                        for ( unsigned i=0; i < 8; i++ ) status << ";" << 1;
                        sendMessage(botId, status.str() );
                    } // if

                } // if        
            } break;
            case 't': { // telemetry
                auto hostMap = this->clientsHostMap.find(clientIp);
                if ( hostMap == this->clientsHostMap.end() ) {
                    LOG( Logger::ERROR, "Server::start - Client not registered IP: %s", clientIp.c_str() );
                    continue;
                } // if

                int botId = hostMap->second;
                long long ts;
                float roll,yaw,pitch,accelWorldX,accelWorldY,accelWorldZ;
                int voltage, heap;
                ss >> ts >> roll >> yaw >> pitch
                   >> accelWorldX >> accelWorldY >> accelWorldZ
                   >> voltage >> heap;

                Logger::get( )->log( Logger::INFO, "Server::%s - id[%d] ts[%lld] yaw[%f] voltage[%d]", __func__, 
                   botId, ts, yaw, voltage );

                std::ostringstream data;
                data << ts << "|" << yaw << "|" 
                     << voltage << "|" << heap;
                Analytics::Payload tele;
                tele.botId = botId;
                tele.data = data.str();
                ANLY->sendRecord( Analytics::TELEMETRY, tele );
            } break;
            case 'd': { // debug
                Logger::get( )->log( Logger::INFO, "Server::%s - id[%s] Debug message [%s]", __func__, clientIp.c_str(), ss.str().c_str() );
            } break;
            default: break;
        } // switch
    } // while
}

Server::~Server( ) {
    LOG( Logger::INFO, "Server::~Server - Closing server. Stopping clients...");
    this->running = false;
    
    freeaddrinfo(this->addrInfo);
    close(this->socketId);
    
    LOG( Logger::INFO, "Server::~Server - Closing main socket and cleaning.");
    this->server.join();

    LOG( Logger::INFO, "Server::~Server - Finished");
}


bool Server::sendMessage( int clientId, const std::string& message ) {
    auto it = this->clients.find( clientId );
    if ( it == this->clients.end() ) {
        return false;
    } // if
    
    LOG( Logger::DEBUG, "Server::sendMessage - Sending message to id: %d, host: %s, port: %d, msg: %s",
        clientId, it->second.host.c_str(), it->second.port, message.c_str() );

    struct sockaddr_in client;
    client.sin_family = AF_INET;
    client.sin_port = htons(it->second.port);
    client.sin_addr.s_addr = inet_addr(it->second.host.c_str());
    socklen_t l = sizeof(client);
 
    int rp = sendto(this->socketId,message.c_str(),message.length(), 
            0, (struct sockaddr *)&client, l);

    if ( rp < 0 ) {
        LOG( Logger::ERROR, "Server::sendMessage - Error while trying to send a message. code: %d, clientId: %d",
            rp, clientId );
        return false;
    } // if 
    return true;
}

