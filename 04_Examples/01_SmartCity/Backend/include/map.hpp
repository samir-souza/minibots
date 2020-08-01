#ifndef __MAP_HPP
#define __MAP_HPP

#include <opencv2/opencv.hpp>
#include <map>
#include <thread>
#include <mutex>
#include <powergine/tools/astarpathfinder.h>
#include <logger.hpp>
#include <analytics.hpp>

class Map {
    public:
        class Entity {
            public:
                Entity( int id, const cv::Point2f& pos = cv::Point2f(0,0), const cv::Point2f& dir = cv::Point2f(0,0) ) :
                    id(id), position(pos), direction(dir), timestamp(0) { }
                inline const cv::Point2f& getPosition( void ) const {
                    return this->position;
                }
                inline void setPosition( const cv::Point2f& pos ) {
                    this->position = pos;
                }
                inline const cv::Point2f& getDirection( void ) const {
                    return this->direction;
                }
                inline void setDirection( const cv::Point2f& dir ) {
                    this->direction = dir;
                }
                inline int getId( void ) const {
                    return this->id;
                }
                inline void setTimestamp( long long ts ) {
                    this->timestamp = ts;
                }
                inline const long long getTimestamp( void ) const {
                    return this->timestamp;
                }
                inline long long getLastUpdate( void ) const {
                    return this->lastUpdate;
                }
                
                virtual const char getType( ) = 0;

                inline bool update( const cv::Point2f& position, const cv::Point2f& direction, long long timestamp ) {
                    // TODO: load threshold from file? maybe...
                    bool updated = false;
                    if ( cv::norm( position - this->position ) > 2 ) {
                        this->position = position;
                        this->lastUpdate = timestamp;
                        updated = true;
                    } // if
                    float angle = std::acos(this->direction.dot(direction)) * 180/M_PI;
                    // TODO: load threshold from file? maybe...
                    if ( std::abs(angle) > 2 ) { 
                        this->direction = direction;
                        this->lastUpdate = timestamp;
                        updated = true;
                    } // if
                    if ( updated ) {
                        std::ostringstream data;
                        data << getType() << "|" << position.x << "|" << position.y << "|" << direction.x << "|" << direction.y << "|" << ANLY->nanos();

                        Analytics::Payload pose;
                        pose.botId = id;
                        pose.data = data.str();
                        ANLY->sendRecord( Analytics::POSE, pose );
                    } // id
                    this->timestamp = timestamp;
                    return updated;
                }
                virtual inline ~Entity() { }
            protected:
                int id;
                cv::Point2f position, direction;
                long long timestamp, lastUpdate;
        };

        class Obstacle : public Entity {
            public:
                Obstacle( int id, const std::string& name, const cv::Point2f& pos, const cv::Point2f& dir ) :
                    Entity(id, pos, dir), name(name), blockedEdges() { }
                inline ~Obstacle( ) { } 
                inline const std::string& getName( void ) const {
                    return this->name;
                }
                inline void blockEdge( long edgeId ) {
                    std::vector<long>::const_iterator it = std::find(blockedEdges.begin(), blockedEdges.end(), edgeId );
                    if ( it != blockedEdges.end() )  return;
                    blockedEdges.push_back( edgeId );
                }
                inline const std::vector<long>& getBlockedEdges( void ) const {
                    return this->blockedEdges;
                }
                inline void clearBlockedEdges( void ) {
                    this->blockedEdges.clear();
                }
                const char getType( ) { return 'o'; }
            private:
                std::string name;
                std::vector<long> blockedEdges; 
        };

        class Place : public Entity{
            public:
                Place( int id, const std::string& name, const cv::Point2f& pos, const cv::Point2f& dir ) :
                    Entity(id, pos, dir), name(name) { }
                inline ~Place( ) { } 
                inline const std::string& getName( void ) const {
                    return this->name;
                }
                const char getType( ) { return 'p'; }
            private:
                std::string name;
        };

        class Agent : public Entity {
            public:
                typedef struct {
                    long long ts;
                    float roll,pitch,yaw,accelWorldX, accelWorldY, accelWorldZ;
                    int voltage, heap;  
                } SENSORREADING;
            private:
                cv::Point2f lastSentSeek, lastSentPosition, lastSentDirection;
                std::vector<cv::Point2f> currentPath;
                int currentNode;
                float pathNodeRadius;
                SENSORREADING sensorReading;
                int goalId,previousGoalId;
                bool blocked;
                long long blockedTime;
                std::vector<long> blockedEdges; 
            public:
                enum LASTSENT {
                    SEEK,
                    POSITION,
                    DIRECTION
                };
                inline Agent(int id, float pathNodeRadius = 10.0f ) :
                    Entity(id), currentNode(0), pathNodeRadius(pathNodeRadius), goalId(-1), previousGoalId(-1) { 
                    this->sensorReading.ts = 0;             
                }

                inline ~Agent( ) { }
                inline const SENSORREADING& getSensorReading( void ) const {
                    return this->sensorReading;
                }
                inline const cv::Point2f& getLastSent( LASTSENT l ) const {
                    switch(l) {
                        case SEEK: return this->lastSentSeek;
                        case POSITION: return this->lastSentPosition; 
                        case DIRECTION: return this->lastSentDirection;
                        default: throw std::runtime_error( "Map::Agent::getLastSent - invalid LASTSENT" );
                    }
                }
                inline void setLastSent( LASTSENT l, const cv::Point2f& lastSent ) {
                    switch(l) {
                        case SEEK: this->lastSentSeek = lastSent; break;
                        case POSITION: this->lastSentPosition = lastSent; break;
                        case DIRECTION: this->lastSentDirection = lastSent; break;
                    }
                }
                inline bool isBlocked( void ) const {
                    return this->blocked;
                }
                inline void setBlocked( bool blocked ) {
                    if ( !this->blocked && blocked ) {
                        this->blockedTime = ANLY->millis();
                    } // if
                    this->blocked = blocked;
                }
 
                inline long long getTotalBlockedTime( void ) const {
                    return (ANLY->millis() - this->blockedTime);
                }

                inline void setGoalId( int goalId ) {
                    this->goalId = goalId;
                }

                inline int getPreviousGoalId( void ) const {
                    return this->previousGoalId;
                }

                inline int getGoalId( void ) const {
                    return this->goalId;
                }
                         
                inline const std::vector<cv::Point2f>& getCurrentPath( void ) const {
                    return this->currentPath;
                }

                inline void blockEdge( long edgeId ) {
                    std::vector<long>::iterator it = std::find(blockedEdges.begin(), blockedEdges.end(), edgeId );
                    if ( it != blockedEdges.end() )  return;
                    blockedEdges.push_back( edgeId );
                }
                inline const std::vector<long>& getBlockedEdges( void ) const {
                    return this->blockedEdges;
                }
                inline void clearBlockedEdges( void ) {
                    this->blockedEdges.clear();
                }
                virtual const char getType( ) { return 'b'; }

                inline int followPath( void ) {
                    // A path needs at least two nodes
                    if (this->currentPath.size( ) < 2 || this->currentNode >= this->currentPath.size()  ) {
                        Logger::get( )->log( Logger::INFO, "Map::Agent::%s - agentid[%d] There is no path to follow", __func__, this->id);
                        return -1;
                    } // if
                    // if we reached the 'near' area of a given node, go to the next one 
                    while ( this->currentPath.size() > 0 && cv::norm(this->currentPath[this->currentNode] - this->position) <= this->pathNodeRadius ) {
                        Logger::get( )->log( Logger::INFO, "Map::Agent::%s - agentid[%d] Near node [%d]", __func__, this->id, this->currentNode);
                        this->currentNode += 1;
                        if ( this->currentNode >= (int)this->currentPath.size( ) ) {
                            Logger::get( )->log( Logger::INFO, "Map::Agent::%s - agentid[%d] Path finished! Cleaning old path...", __func__, this->id);
                            // the bot reached the goal
                            this->clearPath();
                            this->previousGoalId = this->goalId;
                            this->goalId = -1;
                            return -1;
                        } // if
                        
                    }
                    return this->currentNode;
                }
                
                void receiveMessage( const std::string& message );

                inline void addPointToPath( const cv::Point2f& point ) {
                    this->currentPath.push_back( point );
                }

                inline void clearPath( void ) {
                    this->currentPath.clear( );
                    this->currentNode = 0;
                }
        };

        class ManualTarget : public Entity {
            public:
                ManualTarget( int id, const std::string& name, const cv::Point2f& pos, const cv::Point2f& dir ) :
                    Entity(id, pos, dir), name(name), placeId(-1) { }
                inline ~ManualTarget( ) { } 
                inline const std::string& getName( void ) const {
                    return this->name;
                }
                inline void setPlace( int id = -1 ) {
                    this->placeId = id;
                }
                inline int getNearPlace( void ) const {
                    return this->placeId;
                }
                inline bool isNearPlace( int id ) const {
                    return this->placeId == id;
                }
                const char getType( ) { return 'm'; }
            private:
                std::string name;
                int placeId;
        };
        Map(int width, int height, bool preview=false);

        ~Map(void);

        void updateAgent( int id, const cv::Point2f& position, const cv::Point2f& direction, long long timestamp );

        void renderMap( );
        
        bool planPath( int id, int goalId );
        
        bool computeSeekVector( int id, cv::Point2f& seek );

        Map::Agent& getAgent( int id ); 
        
        inline bool isPlaceRegistered( int id ) const {
            return this->places.find(id) != this->places.end( );
        }

        inline void registerPlace( int id, const std::string& name, const cv::Point2f& pos, const cv::Point2f& dir ) {
            if (!isPlaceRegistered(id)) {
                LOG(Logger::INFO, "Map::Place::registerPlace - Registering: %d, name: %s, pos: %f,%f, dir: %f,%f",
                    id, name.c_str(), pos.x, pos.y, dir.x, dir.y); 
                this->places.insert( std::pair<int,Map::Place>( id, Map::Place(id, name, pos, dir ) ) );
                std::ostringstream data;
                data << "p|" << name << "|" << pos.x << "|" << pos.y << "|" << dir.x << "|" << dir.y << "|" << ANLY->nanos();

                Analytics::Payload pose;
                pose.botId = id;
                pose.data = data.str();
                ANLY->sendRecord( Analytics::POSE, pose );
            } // if
        }

        void visibleManualTarget( int id, const std::string& name, 
            const cv::Point2f& pos, const cv::Point2f& dir, long long timestamp );

        void visibleObstacle( int id, const std::string& name, 
            const cv::Point2f& pos, const cv::Point2f& dir, long long timestamp );

        void step();

        inline void markAgentsForPathReplanning( const cv::Point2f& v1, const cv::Point2f& v2 ) {
            for ( auto a : this->agents ) {
                if ( a.second.getGoalId() != -1 && isBlocking( a.second, v1, v2 ) ) {
                    a.second.clearPath();
                } // if
            } // for
        }

        inline bool isBlocking( const Map::Agent& agent, const cv::Point2f& pos ) const {
            cv::Point2f center = (agent.getDirection() * this->agentObstacleDetectionRadius) + agent.getPosition();
            return cv::norm(center - pos) < this->agentObstacleDetectionRadius;
        }

        inline bool isBlocking( const Map::Agent& agent, const cv::Point2f& v1, const cv::Point2f& v2 ) const {
            int blockCount = 0;
            for ( auto p : agent.getCurrentPath() ) {
                blockCount += p == v1 ? 1 : p == v2 ? 1 : 0;
            } // for
            return blockCount == 2;
        }
        inline bool isBotRegistered( int botId ) {
            return this->agents.find(botId) != this->agents.end();
        } // if

        inline std::vector<int> getBlockedEdges( void ) {
            std::vector<int> edgeIds;

            const std::vector<powergine::primitives::GraphEdge*>& edges = this->graph.getEdges( );
            std::vector<powergine::primitives::GraphEdge*>::const_iterator it;
            for ( it = edges.begin(); it != edges.end(); ++it ) {
            //for   ( auto e : this->graph.getEdges( ) ) {
                if ( (*it)->getAdditionalCost() > 0 ) edgeIds.push_back( (*it)->getId( ) );
            } // for
            return edgeIds;
        }
        inline const std::vector<std::pair<int, int>>& getAffectedPoles( int id ) {
            return this->trafficLight[id];
        }

    private:
        void start();
        void loadGraph( );
        bool canConnect( const cv::Point2f& pointA, const cv::Point2f& pointB ) const;
        cv::Size dimension;
        std::unordered_map<int, Map::Agent> agents;
        std::unordered_map<int, Map::Place> places;
        std::unordered_map<int, Map::Obstacle> dynamicObstacles;
        std::unordered_map<int, Map::ManualTarget> manualTargets;

        cv::Mat track;
        powergine::primitives::Graph graph;
        cv::Mat obstacles;
        float pathNodeRadius;
        float placeManualTargetRadius;
        float agentObstacleDetectionRadius;
        bool preview;
        long long blockedEdgeByBotTimeout, seekResendTimeout;
        std::unordered_map<int, std::vector< std::pair<int,int> > > trafficLight;
        int width, height;
        long long missingObjectTimeout;
};

#endif // __MAP_HPP
