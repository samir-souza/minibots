#include <opencv2/highgui/highgui.hpp>
#include <map.hpp>
#include <config.hpp>
#include <cstdlib>
#include <analytics.hpp>

namespace pgpri = powergine::primitives;
namespace pgtoo = powergine::tools; 

Map::Map(int width, int height, bool preview) : 
    dimension( width,  height ), preview(preview), blockedEdgeByBotTimeout(0), seekResendTimeout(0) {
    this->loadGraph( );
    this->pathNodeRadius = CONF(float, "pathNodeRadius");
    this->placeManualTargetRadius = CONF(float, "placeManualTargetRadius");
    this->agentObstacleDetectionRadius = CONF(float, "agentObstacleDetectionRadius");

    this->blockedEdgeByBotTimeout = CONF(long long, "blockedEdgeByBotTimeout" ) * 1000;
    this->seekResendTimeout = CONF( long long, "seekResendTimeout" ) * 1000;

    Config::ListOfObjects data = CONF_OBJS("trafficLight");
    for ( auto& i : data ) {   
        {   std::istringstream parser( i["forward"] ); 
            for ( unsigned j=0; j < 4; ++j ) {
                int e; parser >> e; 
                trafficLight[e].push_back( std::make_pair( std::stoi(i["id"]), 0 ) ); 
            } // for
        }
        {   std::istringstream parser( i["right"] ); 
            for ( unsigned j=0; j < 3; ++j ) {
                int e; parser >> e; 
                trafficLight[e].push_back( std::make_pair( std::stoi(i["id"]), 1 ) ); 
            } // for
        }
    } // for

    std::srand (std::time(NULL));
    this->track = cv::imread("track.jpg");
    if ( this->track.empty() ) {
        this->track = cv::Mat::zeros(this->dimension, CV_8UC3);
    } // if

    this->width = CONF(int, "previewWindowWidth" );
    this->height = CONF(int, "previewWindowHeight" );
    this->missingObjectTimeout = CONF(long long, "missingObjectTimeout" ) * 1000;

    cv::namedWindow( "Board MAP", cv::WINDOW_NORMAL);
    cv::resizeWindow( "Board MAP", width, height );
}

Map::~Map() {
    cv::destroyWindow( "Board MAP" );
    LOG( Logger::INFO, "Map::~Map - Closing Map");
    LOG( Logger::INFO, "Map::~Map - Finished");
}

void Map::renderMap( ) {
    cv::Mat image = this->track.clone();
    // draw reference x,y lines
    cv::line( image, cv::Point2f(5,5), cv::Point2f(635,5), cv::Scalar(255,255,255), 1 );
    cv::line( image, cv::Point2f(5,5), cv::Point2f(5,355), cv::Scalar(255,255,255), 1 );
    cv::putText( image, "(0,0)", cv::Point2f(6,10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 1);

    const std::vector<powergine::primitives::GraphEdge*>& edges = this->graph.getEdges( );
    std::vector<powergine::primitives::GraphEdge*>::const_iterator itEd;
    for (itEd = edges.begin(); itEd != edges.end(); ++itEd ) {
        pgpri::Vector3D p1 = (*itEd)->getVertex1( )->getPosition( );
        pgpri::Vector3D p2 = (*itEd)->getVertex2( )->getPosition( );
        pgpri::Vector3D p3 = p1 + (p2 - p1)/2;
        cv::Point c( p3.getX( ), p3.getY( ) );

        cv::putText( image, std::to_string( (*itEd)->getId() ).c_str(), c,
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,0), 1);
    } // for

    // draw all the places
    std::unordered_map<int, Map::Place>::iterator itP;
    std::unordered_map<int, Map::ManualTarget>::iterator itM;
    for ( itP = this->places.begin(); itP != this->places.end(); ++itP ) { 
        cv::Point2f pos = itP->second.getPosition( );
        cv::circle( image, pos, 5, cv::Scalar(0,0,0), -1 );
        
        cv::Scalar color = cv::Scalar(255,255,255); 
        for ( itM = this->manualTargets.begin(); itM != this->manualTargets.end(); ++itM ) {
            if ( itM->second.isNearPlace( itP->first ) ) {
                color = cv::Scalar(255,127,0);
            } // if
        } // for
        cv::circle( image, pos, this->placeManualTargetRadius, color, 1 );
        pos.x -= this->dimension.width * 0.05;
        cv::putText( image, itP->second.getName( ).c_str(), pos, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,255,255), 1);
    } // for


    std::unordered_map<int, Map::Agent>::iterator it;
    for ( it = this->agents.begin(); it != this->agents.end(); ++it ) { 
        int id = it->first;
        cv::Point2f pos = it->second.getPosition();
        const Map::Agent::SENSORREADING& sensorReading = it->second.getSensorReading();
        // plot agent path
        const std::vector<cv::Point2f>& path = it->second.getCurrentPath( );
        for ( unsigned int i=1; i < path.size(); ++i ) {
            cv::Point2f p1( path[i-1] );
            cv::Point2f p2( path[i] );
            cv::line( image, p1, p2, cv::Scalar(127,127,255), 2 );
            cv::circle( image, p1, this->pathNodeRadius, cv::Scalar(255,0,0), 2 );
        } // for
        if ( sensorReading.ts != 0 ) {
            float yaw = sensorReading.yaw;
            cv::Point2f gyroDir( std::cos(yaw), std::sin(yaw) );
            cv::line( image, pos, pos + ( gyroDir * 20), cv::Scalar(0,0,255), 2 );
        } // if
        
        // plot agent
        cv::line( image, pos, pos + (it->second.getDirection() * 30), cv::Scalar(255,255,255), 2 );
        cv::circle( image, pos, 5, cv::Scalar(255,0,0), -1 );
        cv::Scalar color = it->second.isBlocked() ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0);
        cv::circle( image,
            pos + (it->second.getDirection() * this->agentObstacleDetectionRadius ),
            this->agentObstacleDetectionRadius, color, 1 );

        cv::Point2f seek;
        if ( this->computeSeekVector(id, seek ) ) {
            cv::line( image, pos, seek, cv::Scalar(0,127,127), 1 );
        } // if
    } // for

    std::unordered_map<int, Map::Obstacle>::iterator itO;
    for ( itO = this->dynamicObstacles.begin(); itO != this->dynamicObstacles.end(); ++itO ) { 
        cv::Point2f pos = itO->second.getPosition( );
        // TODO: obstacleRadius
        cv::circle( image, pos, 20, cv::Scalar(0,0,255), -1 );
        cv::circle( image, pos, 20, cv::Scalar(255,255,255), 2 );
        pos.x -= this->dimension.width * 0.025;
        cv::putText( image, itO->second.getName( ).c_str(), pos,
            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0,255,255), 1);
    } // for

    cv::imshow( "Board MAP", image );
    cv::waitKey(250);
}

void Map::updateAgent( int id, const cv::Point2f& position, const cv::Point2f& direction, long long timestamp ) {
    Map::Agent &agent = this->getAgent(id);
    bool updated = agent.update( position, direction, timestamp );

    if (!updated ) {
        long long elapsedTime = timestamp - agent.getLastUpdate( );
        if ( elapsedTime > this->seekResendTimeout ) {
           agent.setLastSent( Map::Agent::SEEK, cv::Point2f(-1, -1) ); 
        } // if

        if ( elapsedTime > this->blockedEdgeByBotTimeout ) {
            pgpri::Vector3D p1( position.x, position.y );
            const std::vector<long>& blockedEdges = agent.getBlockedEdges( );
            std::vector<long>::const_iterator itBe;
            for ( itBe = blockedEdges.cbegin(); itBe != blockedEdges.cend(); ++itBe ) {
                pgpri::GraphEdge *blockedEdge = this->graph.getEdge(*itBe);
                blockedEdge->setAdditionalCost( blockedEdge->getAdditionalCost() - 10000.0 );
            } // for
            agent.clearBlockedEdges();

            // check if it blocked a path
            const std::vector<powergine::primitives::GraphEdge*>& edges = this->graph.getEdges( );
            std::vector<powergine::primitives::GraphEdge*>::const_iterator itEd;
            for (itEd = edges.cbegin(); itEd != edges.cend(); ++itEd ) {
                pgpri::Vector3D p2 = (*itEd)->getNearestPointInEdge( p1 );
                float dist = (p2 - p1).magnitude();
                if ( dist < 20 ) {
                    // TODO: remove blocked cost when the obstacle is gone, time?
                    (*itEd)->setAdditionalCost( 10000.0 );
                    agent.blockEdge( (*itEd)->getId( ) );
                    pgpri::Vector3D v1 = (*itEd)->getVertex1()->getPosition();
                    pgpri::Vector3D v2 = (*itEd)->getVertex2()->getPosition();
                    this->markAgentsForPathReplanning( 
                        cv::Point2f(v1.getX(), v1.getY()), cv::Point2f( v2.getX(), v2.getY()  ) );
                } // if
            } // for
        } // if
    } // if    

    agent.setBlocked( false );
    // any agent blocking the path?
    // HACK to try to avoid segfault
    std::unordered_map<int, Map::Agent>::const_iterator itAg;
    for ( itAg = this->agents.cbegin(); itAg != this->agents.cend(); ++itAg ) {
        if ( itAg->first == id ) continue;
        if ( isBlocking( agent, itAg->second.getPosition() ) ) {
            bool reallyBlocked = true;
            int currNode = -1;
            if ( (currNode = agent.followPath()) != -1 ) {
                cv::Point2f currSeek = agent.getCurrentPath()[currNode];
                reallyBlocked =  ( cv::norm( currSeek - agent.getPosition() ) >
                                   cv::norm( currSeek - itAg->second.getPosition() ) );
            } // if
            if ( reallyBlocked ) { agent.setBlocked( true ); break; }
        } // if 
    } // for
    // any obstacle blocking the path?
    std::unordered_map<int, Map::Obstacle>::const_iterator itOb;
    for ( itOb = this->dynamicObstacles.cbegin(); itOb != this->dynamicObstacles.cend(); ++itOb ) {
        if ( isBlocking( agent, itOb->second.getPosition() ) ) {
            agent.setBlocked( true ); 
            break;     
        } // if 
    } // for

    // plan a new path if the agent completed it's mission
    if ( agent.followPath( ) == -1 ) {
        if ( this->places.size( ) > 2 ) {
            int goalId = agent.getGoalId( );
            if ( goalId == -1 ) {
                std::vector<int> freeGoals;
                std::unordered_map<int, Map::Place>::const_iterator itPl;
                for ( itPl = this->places.cbegin(); itPl != this->places.cend(); ++itPl ) {
                    if ( itPl->first != agent.getPreviousGoalId() ) freeGoals.push_back( itPl->first );
                } // for
                for ( itAg = this->agents.cbegin(); itAg != this->agents.cend(); ++itAg ) {
                    int otherGoalId = itAg->second.getGoalId( );
                    if ( otherGoalId != -1 )
                        freeGoals.erase( std::find( freeGoals.begin(), freeGoals.end(), otherGoalId ));
                } // for
                std::unordered_map<int, Map::ManualTarget>::const_iterator itMt;
                for ( itMt = this->manualTargets.cbegin(); itMt != this->manualTargets.cend(); ++itMt ) {
                    int nearPlaceId = itMt->second.getNearPlace();
                    if ( std::find( freeGoals.begin(), freeGoals.end(), nearPlaceId ) != freeGoals.end() ) {
                        goalId = nearPlaceId;
                        break;
                    } // if
                } // for
                if ( goalId == -1 ) goalId = freeGoals[std::rand() % freeGoals.size()];
            } // if

            LOG( Logger::INFO, "Map::updateAgent - Planning a new path for agent[%d] to goal[%d]", id, goalId );
            this->planPath( id, goalId );
        } // if
    } // if           
}

void Map::step() {
    long long now = ANLY->millis();
    std::vector<int> oldObstacles;
    std::vector<int> oldManualTargets;
    std::vector<int> oldBots;
    std::vector<int>::const_iterator itIv;
    std::vector<long>::const_iterator itBe;

    // check if a previous observed manual target was removed from the map
    std::unordered_map<int, Map::ManualTarget>::const_iterator itMt;
    for ( itMt = this->manualTargets.cbegin(); itMt != this->manualTargets.cend(); ++itMt ) {
       if ( (now - itMt->second.getTimestamp()) > missingObjectTimeout ) {
            LOG(Logger::DEBUG, "Map::start - Missing manual target removed [%d] after %d secs", 
                itMt->second.getId(), (missingObjectTimeout/1000));
            oldManualTargets.push_back(itMt->first);
        } // if 
    } // for

    for ( itIv = oldManualTargets.cbegin(); itIv != oldManualTargets.cend(); ++itIv ) {
        this->manualTargets.erase(*itIv);
    } // for

    // check if a previous observed obstacle was removed from the map
    // we need to erase them from the map after a timeout
    std::unordered_map<int, Map::Obstacle>::const_iterator itDo;
    for ( itDo = this->dynamicObstacles.cbegin(); itDo != this->dynamicObstacles.cend(); ++itDo ) {
       if ( (now - itDo->second.getTimestamp()) > missingObjectTimeout ) {
            LOG(Logger::DEBUG, "Map::start - Missing obstacle removed [%d] after %d secs", 
                itDo->second.getId(), (missingObjectTimeout/1000));
            oldObstacles.push_back(itDo->first);
    
            const std::vector<long>& blockedEdges = itDo->second.getBlockedEdges( );
            for ( itBe = blockedEdges.cbegin(); itBe != blockedEdges.cend(); ++itBe ) {
                pgpri::GraphEdge *blockedEdge = this->graph.getEdge(*itBe);
                blockedEdge->setAdditionalCost( blockedEdge->getAdditionalCost() - 10000.0 );
            } // for
       } // if
    } // for

    for ( itIv = oldObstacles.cbegin(); itIv != oldObstacles.cend(); ++itIv ) {
        this->dynamicObstacles.erase(*itIv);
    } // for

    std::unordered_map<int, Map::Agent>::const_iterator itA;
    for ( itA = this->agents.cbegin(); itA != this->agents.cend(); ++itA ) {
       if ( (now - itA->second.getTimestamp()) > missingObjectTimeout ) {
            LOG(Logger::DEBUG, "Map::start - Missing agent removed [%d] after %d secs", 
                itA->second.getId(), (missingObjectTimeout/1000));
            oldBots.push_back(itA->first);

            const std::vector<long>& blockedEdges = itA->second.getBlockedEdges( );
            for ( itBe = blockedEdges.cbegin(); itBe != blockedEdges.cend(); ++itBe ) {
                pgpri::GraphEdge *blockedEdge = this->graph.getEdge(*itBe);
                blockedEdge->setAdditionalCost( blockedEdge->getAdditionalCost() - 10000.0 );
            } // for
        } // if 
        
    } // for

    for ( itIv = oldBots.cbegin(); itIv != oldBots.cend(); ++itIv ) {
        this->agents.erase(*itIv);
    } // for

}

void Map::loadGraph( ) {

    cv::Mat track = cv::imread("track.jpg", cv::IMREAD_GRAYSCALE );
    if ( track.empty() ) throw std::runtime_error( "Error reading track.jpg for generating the obstacles" );
    std::ifstream graphFile( "graph.txt", std::ifstream::in );
    if ( !graphFile.is_open( ) ) throw std::runtime_error( "Error opening file graph.txt" ); 

    cv::threshold(track, this->obstacles, 127,255,cv::THRESH_BINARY);
    // let's keep a map of leaded vertices to avoid redundancy
    std::unordered_map<std::string, pgpri::GraphVertex*> loadedNodes;
    std::unordered_map<std::string, pgpri::GraphVertex*>::const_iterator it;

    while( graphFile.good() ) {
        // read each line from the file and process the nodes and edges
        // each line contains a reference node and its adjacent nodes
        std::string line;
        std::getline(graphFile, line); 
        std::istringstream ss(line);
        cv::Point p;
        int counter = 0;
        pgpri::GraphVertex *n1=NULL,*n2=NULL, *node = NULL;
        while( ss >> p.x and ss >> p.y ) {
            std::string key = std::to_string(p.x) + "." + std::to_string(p.y);
            it = loadedNodes.find( key );
            if ( it == loadedNodes.end() ) {
                pgpri::Vector3D position( p.x, p.y, 0 );
                node = this->graph.addVertex( position );
                loadedNodes[key] = node;
            } else {
                node = it->second;
            } // else

            if ( counter == 0 ) n1 = node;
            else {
                 n2 = node;
                this->graph.addEdge( n1, n2 ); 
            } // else
            ++counter;
        } // while
    } // while
    graphFile.close();
    // let's just do a quick test to see if the planner is working
    pgpri::GraphVertex* start = this->graph.getVertex( 0 );
    pgpri::GraphVertex* end = this->graph.getVertex( 14 );
    //Logger::get( )->log( Logger::DEBUG, std::string("Map::") + __func__ + " - " + this->graph.toString() );
   
    pgtoo::AStarPathFinder* pathFinder = new pgtoo::AStarPathFinder( &this->graph, start, end );
    Logger::get( )->log( Logger::DEBUG, std::string("Map::") +  __func__ + " - " + pathFinder->toString() );

   delete pathFinder;

}

bool Map::canConnect( const cv::Point2f& pointA, const cv::Point2f& pointB ) const {
    // let's  see if we can connect two points with a straight line
    // return false if there is any obstacle between points
    if ( pointA.y < 0 || pointA.y > this->obstacles.rows || 
         pointA.x < 0 || pointA.x > this->obstacles.cols ||
         pointB.y < 0 || pointB.y > this->obstacles.rows || 
         pointB.x < 0 || pointB.x > this->obstacles.cols ) return false; 

    if ( this->obstacles.at<uchar>( pointA.y, pointA.x ) > 0 ||
         this->obstacles.at<uchar>( pointB.y, pointB.x ) > 0 ) return false;
    
    cv::Point2f target = pointB - pointA;
    double length = cv::norm(target);
    cv::Point2f step = target / length;
    step.x *= this->obstacles.cols * 0.01;
    step.y *= this->obstacles.rows * 0.01;

    cv::Point2f pivot = pointA + step;
    for ( ; cv::norm(pivot) < length; pivot += step ) {
        if ( pivot.y < 0 || pivot.y > this->obstacles.rows || 
             pivot.x < 0 || pivot.x > this->obstacles.cols ) return false; 
        if ( this->obstacles.at<uchar>( pivot.y, pivot.x) > 0 ) return false;
    } // for
    return true;
}

bool Map::planPath( int id, int goalId ) {
    std::unordered_map<int, Map::Agent>::iterator it = this->agents.find( id );
    if ( it == this->agents.end( ) ) {
        return false;
    } // if

    Logger::get( )->log( Logger::DEBUG, "Map::planPath - # of nodes[%d] # of edges[%d]",
        this->graph.getVertices().size(), this->graph.getEdges().size() );
    const Map::Place& place = this->places.find(goalId)->second;
    it->second.setGoalId( goalId );
    const cv::Point2f& target = place.getPosition();

    LOG( Logger::INFO, "Map::planPath - Selected place for bot[%d]: %s - idx[%d]", 
        id, place.getName().c_str(), goalId);

    pgtoo::AStarPathFinder* pathFinder = new pgtoo::AStarPathFinder( &this->graph );
    // First, let's  try to find the nearest edge/vertex to the start/target points 
    cv::Point2f pos = it->second.getPosition( );
    
    pgpri::Vector3D originPoint( pos.x, pos.y );
    pgpri::Vector3D targetPoint( target.x, target.y );
    pgpri::GraphVertex *originVertex = NULL;
    pgpri::GraphVertex *targetVertex = NULL;
    
    float minDistanceOrigin = std::numeric_limits<float>::max();
    float minDistanceTarget = std::numeric_limits<float>::max();
    pgpri::Vector3D originRefPoint, targetRefPoint;

    LOG( Logger::DEBUG, "Map::planPath - Try to find good points in graph to connect" );
    std::vector<powergine::primitives::GraphEdge*>::const_iterator itEd;
    const std::vector<powergine::primitives::GraphEdge*>& edges = this->graph.getEdges( );
    for ( itEd = edges.begin(); itEd != edges.end(); ++itEd ) {
        pgpri::Vector3D pointO = (*itEd)->getNearestPointInEdge( originPoint );
        pgpri::Vector3D pointT = (*itEd)->getNearestPointInEdge( targetPoint );
        float distO = ( pointO - originPoint ).magnitude( );
        float distT = ( pointT - targetPoint ).magnitude( );
        if ( distO < minDistanceOrigin && canConnect( pos, cv::Point2f( pointO.getX(), pointO.getY() ) ) ) {
            minDistanceOrigin = distO;
            originRefPoint = pointO;
            originVertex = (*itEd)->getVertex2( );
        } // if
        if ( distT < minDistanceTarget && canConnect(target, cv::Point2f( pointT.getX(), pointT.getY() ) ) ) {
            minDistanceTarget = distT;
            targetRefPoint = pointT;
            targetVertex = (*itEd)->getVertex1( );
        } // if
    } // for
    
    if ( originVertex != NULL && targetVertex != NULL ) {

        LOG( Logger::DEBUG, "Map::planPath - Ok. We found two points. Let's  connect and plan" );
        // ok, we found a possible path. Let's connect the origin and the goal to the graph
        
        // the boolean variables are flags to check if the given position
        // has a vertex attached to graph. if yes, we cannot delete them!!
        // take a look at the cleaning step after path planning!

        std::vector<pgpri::GraphVertex*> newVertices;
        std::vector<pgpri::GraphEdge*> newEdges;
        
        pgpri::GraphVertex *startVertex=NULL,*endVertex=NULL,*startRefVertex=NULL,*endRefVertex=NULL;
        pgpri::GraphEdge *temp1=NULL, *temp1_2=NULL, *temp2=NULL, *temp2_2=NULL;

        if ( (startVertex = this->graph.getVertexByPosition( originPoint ) ) == NULL ) {
            startVertex = this->graph.addVertex( originPoint );
            newVertices.push_back( startVertex );
        } // if
        if ( (startRefVertex = this->graph.getVertexByPosition( originRefPoint ) ) == NULL ) {
            startRefVertex = this->graph.addVertex( originRefPoint );
            newVertices.push_back( startRefVertex );
        } // if
        
        if ( (endVertex = this->graph.getVertexByPosition( targetPoint ) ) == NULL ) {
            endVertex = this->graph.addVertex( targetPoint );
            newVertices.push_back( endVertex );
        } // if
        if ( (endRefVertex = this->graph.getVertexByPosition( targetRefPoint ) ) == NULL ) {
            endRefVertex = this->graph.addVertex( targetRefPoint );
            newVertices.push_back( endRefVertex );
        } // if

        if ( ( temp1 = this->graph.getEdge( startVertex, startRefVertex ) ) == NULL ) {
            temp1 = this->graph.addEdge( startVertex, startRefVertex );
            newEdges.push_back(temp1);
        } // if
        if ( ( temp1_2 = this->graph.getEdge( startRefVertex, originVertex ) ) == NULL ) {
            temp1_2 = this->graph.addEdge( startRefVertex, originVertex );
            newEdges.push_back(temp1_2);
        } // if
        if ( ( temp2 = this->graph.getEdge( targetVertex, endRefVertex ) ) == NULL ) {
            temp2 = this->graph.addEdge( targetVertex, endRefVertex );
            newEdges.push_back(temp2);
        } // if
        if ( ( temp2_2 = this->graph.getEdge( endRefVertex, endVertex ) ) == NULL ) {
            temp2_2 = this->graph.addEdge( endRefVertex, endVertex );
            newEdges.push_back(temp2_2);
        } // if


        pathFinder->setStartVertex( startVertex );
        pathFinder->setEndVertex( endVertex );

        LOG( Logger::DEBUG, "Map::planPath - Nodes were connected. Let's run A*" );
        pathFinder->updatePath( true );
        LOG( Logger::DEBUG, "Map::planPath - Done. Now, let's  get the points" );
        it->second.clearPath( );

        std::ostringstream data;
        std::vector<powergine::primitives::GraphVertex*>::const_iterator itVe;
        const std::vector<powergine::primitives::GraphVertex*>& pathVertices = pathFinder->getPathVertices( );
        //for ( auto v: pathFinder->getPathVertices( ) ) {
        for ( itVe = pathVertices.begin(); itVe != pathVertices.end(); ++itVe ) {
            pgpri::Vector3D p = (*itVe)->getPosition( );
            it->second.addPointToPath( cv::Point2f( p.getX(), p.getY() ) );
            data << "|" << p.getX() << "," << p.getY( );
        } // for
         
        Analytics::Payload botPath;
        botPath.botId = id;
        botPath.data = data.str();
        ANLY->sendRecord( Analytics::BOT_PATH, botPath );        

        LOG( Logger::DEBUG, "Map::planPath - Finished. Let's clean everything!" );
        delete pathFinder;
        for ( auto e : newEdges ) {
            this->graph.removeEdge( e );
            delete e;
        } // for
        for ( auto v : newVertices ) {
            this->graph.removeVertex( v );
            delete v;
        } // for

        return true;
    } // if

    LOG( Logger::DEBUG, "Map::planPath - Ops! Something went wrong. A path wasn't found!" );
    return false;
}

bool Map::computeSeekVector( int id, cv::Point2f& seek ) {
    std::unordered_map<int, Map::Agent>::iterator it = this->agents.find(id);
    if ( it == this->agents.end( ) ) return false;
    // ok, the given id exists, let's get the id of the current node
    // that belongs to the path we're following, if any
    int currentNode = it->second.followPath( );
    if ( currentNode != -1 ) {
        // there is a path to follow. Let's see if we need to move to the
        // next node in the path.
        seek = it->second.getCurrentPath()[currentNode];
        return true;
    } // if
    return false;   
}

Map::Agent& Map::getAgent( int id ) {
    auto it = this->agents.find(id);
    if ( it == this->agents.end( ) ) {
        this->agents.insert( std::make_pair( id, Map::Agent(id, this->pathNodeRadius ) ) );
        it = this->agents.find(id);
        LOG( Logger::DEBUG, "Map::getAgent - new Agent added agentid[%d]", it->second.getId() );
    } // if
    return it->second;
}


void Map::visibleManualTarget( int id, const std::string& name, const cv::Point2f& pos, const cv::Point2f& dir, long long timestamp ) {
    std::unordered_map<int, Map::ManualTarget>::iterator it;
    it = this->manualTargets.find( id );
    bool updated = false;
    if ( it == this->manualTargets.end() ) {
        LOG(Logger::INFO, "Map::registerManualTarget - Registering: %d, name: %s, pos: %f,%f, dir: %f,%f",
            id, name.c_str(), pos.x, pos.y, dir.x, dir.y); 
        Map::ManualTarget manualTarget(id, name, pos, dir );
        manualTarget.setTimestamp( timestamp );
        it = this->manualTargets.insert( std::make_pair(id, manualTarget ) ).first;
        updated = true; 
    } else {
        it->second.setTimestamp( timestamp );
        updated = it->second.update( pos, dir, timestamp );
    } // else

    if ( updated ) {
        bool nearPlace = false;
        std::unordered_map<int, Map::Place>::iterator itP;
        for ( itP = this->places.begin(); itP != this->places.end(); ++itP ) {
            float dist = cv::norm( itP->second.getPosition( ) - it->second.getPosition( ) ); 
            if ( dist <= this->placeManualTargetRadius ) {
                it->second.setPlace( itP->first );
                LOG(Logger::INFO, "Map::registerManualTarget - Manual target [%d] near place [%d]", itP->first, id );
                nearPlace = true;
                break;
            } // if
        } // for
        if ( !nearPlace ) it->second.setPlace();
    } // if
}

void Map::visibleObstacle( int id, const std::string& name, 
    const cv::Point2f& pos, const cv::Point2f& dir, long long timestamp ) {
    bool updateGraph = false; 
    std::unordered_map<int, Map::Obstacle>::iterator it = this->dynamicObstacles.find(id);
    if ( it == this->dynamicObstacles.end() ) {
        LOG( Logger::INFO, "Map::visibleObstacle - New obstacle spoted [%d] pos{%f, %f}", id, pos.x, pos.y );
        Map::Obstacle obstacle(id, name, pos, dir );
        obstacle.setTimestamp( timestamp );
        it = this->dynamicObstacles.insert( std::make_pair(id, obstacle ) ).first;
        updateGraph = true;
    } else {
        it->second.setTimestamp( timestamp );
        updateGraph = it->second.update( pos, dir, timestamp );
    } // else
    if ( updateGraph ) {
        pgpri::Vector3D p1( pos.x, pos.y );
        // this obstacle just appeared or was moved, so, let's unblock old edges to re-evaluate them again
        for ( auto edgeId : it->second.getBlockedEdges( ) ) {
            pgpri::GraphEdge *blockedEdge = this->graph.getEdge(edgeId);
            blockedEdge->setAdditionalCost( blockedEdge->getAdditionalCost() - 10000.0 );
        } // for
        it->second.clearBlockedEdges();

        // check if it blocked a path
        for ( auto edge : this->graph.getEdges() ) {
            pgpri::Vector3D p2 = edge->getNearestPointInEdge( p1 );
            float dist = (p2 - p1).magnitude();
            LOG( Logger::INFO, "Map::visibleObstacle - Dist obst[%d] edge [%d]=%f.Nearest point {%f,%f}", 
                id, edge->getId(), dist, p2.getX(), p2.getY() );
            if ( dist < 20 ) {
                // TODO: remove blocked cost when the obstacle is gone, time?
                edge->setAdditionalCost( 10000.0 );
                it->second.blockEdge( edge->getId( ) );
                LOG( Logger::INFO, "Map::visibleObstacle - blocked edge: %d", edge->getId() );
                pgpri::Vector3D v1 = edge->getVertex1()->getPosition();
                pgpri::Vector3D v2 = edge->getVertex2()->getPosition();
                this->markAgentsForPathReplanning( 
                    cv::Point2f(v1.getX(), v1.getY()), cv::Point2f( v2.getX(), v2.getY()  ) );
            } // if
        } // for
    } // if
}


