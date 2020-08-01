#ifndef __SERVER_HPP
#define __SERVER_HPP

#include <cstdio>
#include <unistd.h>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
//#include <openssl/ssl.h>
//#include <openssl/err.h>
#include <thread>
#include <sstream>
#include <iostream>
#include <functional>
#include <algorithm>
#include <unordered_map>
#include <logger.hpp>

class Server {
    public: 
 
        Server( const std::string& host, int port );

        ~Server( );

        bool sendMessage( int clientId, const std::string& message );

        inline bool isClientRegistered( int clientId ) {
            auto it = this->clients.find(clientId);
            return ( it != this->clients.end() && it->second.active );
        }
        struct Client {
            bool active;
            int botId;
            int port;
            std::string host;
        };
    private:
        void start( void );
        int socketId;
        bool running;
        std::thread server;
        
        std::unordered_map<int, Client> clients;
        std::unordered_map<std::string, int> clientsHostMap;
        
        struct addrinfo *addrInfo;

};

#endif // __SERVER_HPP
