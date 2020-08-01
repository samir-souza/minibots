#include <opencv2/opencv.hpp>
#include <trackBuilder.hpp>
#include <unistd.h>
#include <config.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>

bool TrackBuilder::compareArea( std::vector<cv::Point>& p1, std::vector<cv::Point>& p2 ) {
    return cv::contourArea(p1) > cv::contourArea(p2);
}

void TrackBuilder::getBoxFromContour(std::vector<cv::Point>& contour, cv::Rect& box ) {
    cv::RotatedRect minRect = cv::minAreaRect(contour);
    cv::Mat boxPts;
    cv::boxPoints(minRect, boxPts);
    // bottom left, top left, top right, bottom right
    box.x = boxPts.at<float>(1,0);
    box.y = boxPts.at<float>(1,1);
    box.width = boxPts.at<float>(3,0) - boxPts.at<float>(1,0);
    box.height = boxPts.at<float>(3,1) - boxPts.at<float>(1,1);
}

void TrackBuilder::getContours(const cv::Mat& img, std::vector<std::vector<cv::Point>>& contours, cv::Mat& thresh ) {
    cv::Mat blur;
    cv::GaussianBlur(img, blur, cv::Size(5,5),0);
    cv::threshold(blur, thresh, 0,255,cv::THRESH_BINARY+cv::THRESH_OTSU);

    cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE );
    std::sort( contours.begin(), contours.end(), TrackBuilder::compareArea );
}

float TrackBuilder::getCorrectAngle(float angle ) {
    float signal = angle < 0 ? -1 : 1;
    angle = std::abs(angle);
    if (angle < 5 ) angle = 0;
    else if ( angle < 95 ) angle = 90;
    else if ( angle < 185 ) angle = 180;
    return angle * signal;
}

cv::Point2f TrackBuilder::getNextDirection(float angle, float previousAngle) {
    // TODO: compute it dinamicaly using the resolution
    //cv::Point2f ref (38.5/1280*800, 37.5/720*600 );
    float xFactor = CONF(float, "xFactor");
    float yFactor = CONF(float, "yFactor");
    int width = CONF(int, "width");
    int height = CONF(int, "height");
    cv::Point2f ref (xFactor/1280*width, yFactor/720*height );
    if (angle == -90 and previousAngle == 180 ) ref.y *= -1;
    else if ( angle == 0 and previousAngle == -90 ) ;
    else if ( angle == 90 and previousAngle == 0 ) ref.x *= -1;
    else if ( angle == 180 and previousAngle == 90 ) ref *= -1;
    else if ( angle == 90 and previousAngle == 180 ) ref *= -1;
    else if ( angle == 180 and previousAngle == -90 ) ref.y *= -1;
    else if ( angle == 0 and previousAngle == 90 ) ref.x *= -1;
    else if ( angle == -180 and previousAngle == 90 ) ref *= -1;
    else if ( angle == -90 and previousAngle == -180 ) ref.y *= -1;
    else if ( angle == -90 and previousAngle == 0 );
    else
        std::cerr << "ERROR - Angle: " << angle << " Previous angle: " << previousAngle << std::endl;
    return ref;
}

void TrackBuilder::expandPolygons( std::vector<std::vector<cv::Point>>& polygons ) {
    for ( unsigned int i=0; i < polygons.size(); ++i ) {
        std::vector<float> angles;
        for ( unsigned int p1 = 0; p1 < polygons[i].size(); ++p1 ) {
            int p2 = (p1+1) % polygons[i].size();
            
            float angle = std::atan2(polygons[i][p1].y - polygons[i][p2].y, polygons[i][p1].x - polygons[i][p2].x) * 180/M_PI;
            //std::cout << angle << " " << getCorrectAngle(angle) <<  std::endl;
            angles.push_back( getCorrectAngle(angle) );
        } // for
        for ( unsigned int p1 = 0; p1 < angles.size(); ++p1 ) {
            int p2 = (p1+1) % angles.size();
            //std::cout << angles[p1] << " " << angles[p0] << std::endl;
            cv::Point2f nextDir = getNextDirection(angles[p2], angles[p1] );
            polygons[i][p2].x += nextDir.x; 
            polygons[i][p2].y += nextDir.y; 
            //std::cout << angles[p1] << " " << angles[p0] << std::endl;
            //break;
        } // for
    } // for
}

TrackBuilder::TrackBuilder(int width, int height, int camid, int fps) {
    cv::VideoCapture cap(camid);
    cap.set(cv::CAP_PROP_FPS, fps);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width );
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    //cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('J','P','E','G'));
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('B','G','R','3'));

    cap >> this->frame;

    bool undistort = CONF(bool, "undistort");
    if ( undistort ) {
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
        
        cv::Mat newK;
        double balance = 1.0;
        cv::Mat map1, map2;
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, frame.size(), cv::noArray(), newK, balance);
        cv::fisheye::initUndistortRectifyMap(K, D, cv::noArray(), newK, frame.size(), CV_16SC2, map1, map2 );
        cv::Mat newFrame;
        cv::remap(frame, newFrame, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT );
        this->frame = newFrame.clone();
    } // if
    std::cout << this->frame.cols << " " << this->frame.rows << std::endl;
}

TrackBuilder::TrackBuilder(const std::string& fileName ) {
    this->frame = cv::imread(fileName);
}

void TrackBuilder::processMap( ) {
    cv::Mat gray, thresh, track;
    // first, lets convert the image to grayscale
    cv::cvtColor(this->frame, gray, cv::COLOR_BGR2GRAY); 

    // but we need to look for apriltags (places) and remove them from the image
    // Make an image_u8_t header for the Mat data
    image_u8_t im = { 
        .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = gray.data
    };
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tagStandard41h12_create();
    apriltag_detector_add_family(td, tf);

    zarray_t *detections = apriltag_detector_detect(td, &im);        

    // Iterate through all tags and update the map with the orientation/position
    int numberOfTags = zarray_size(detections);
    for (int i = 0; i < numberOfTags; ++i) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        
        cv::Point2f pos( det->c[0], det->c[1] );
        cv::Point2f cornerA(det->p[0][0], det->p[0][1]);
        cv::Point2f cornerB(det->p[2][0], det->p[2][1]);
        float len = cv::norm( cornerA - cornerB ) * 1.4;
        
        std::cout << "Tag detected. id: " << det->id << " " << pos << " " << len << std::endl;

        cv::circle( gray, pos, len, cv::Scalar(0,0,0), -1);
    } // for
    apriltag_detections_destroy(detections);

    apriltag_detector_destroy(td);
    tagStandard41h12_destroy(tf);
    cv::imwrite( "gray_tag_dump.jpg", gray );

    std::vector<std::vector<cv::Point> > contoursA, contoursB;
    // then, we extract the contours of all polygons
    getContours(gray, contoursA, thresh );
    // here, we detect the bounding box around the track limits
    // this is important because we want only the polygons inside the track
    getBoxFromContour(contoursA[0], this->box );

    if ( this->box.x < 0 || this->box.y < 0 || this->box.width < 4 || this->box.width < 4 ) {
        std::cout << " Something went wrong with the contours... maybe the lighting" << std::endl;
    } // if

    // then, we can extract all the polygons that represent
    // the obstacles inside the track. 
    getContours(gray(this->box), contoursB, track );
    this->mask = cv::Mat::zeros(track.rows, track.cols, CV_8UC3 );

    // ok. Now, let's  compute the polygons, based on the contours
    // this will return us a set of points for each polygon.
    std::vector<std::vector<cv::Point>> polygons;
    for (unsigned int i = 1; i < contoursB.size(); ++i ) {
        double epsilon = 0.01 * cv::arcLength(contoursB[i], true);
        std::vector<cv::Point> approx;
        std::vector<std::vector<cv::Point>> contours;
        cv::approxPolyDP(contoursB[i], approx, epsilon, true);
        //std::cout << "Approx: " << approx << std::endl;
        contours.push_back(approx);
        cv::drawContours(this->mask, contours, 0, cv::Scalar(255,255,0), -1);
        polygons.push_back(approx);
    } // for

    // here it comes the magic. we need to compute a graph that represents
    // the free path for the bots. by expanding the polygons, we can find
    // intersecting points and merge them into the graph later.
    expandPolygons(polygons);
    
    std::cout << this->box << " " << this->mask.rows << " " << this->mask.cols << std::endl;
    double THRESH = CONF(double, "trackBuilderAngleThreshold");
    // the next nested loop will look for intersecting points, merge them
    // into the graph and then it will remove duplicate edges.
    for (unsigned int i=0; i < polygons.size(); ++i ) {
        for ( unsigned int p1=0; p1 < polygons[i].size(); ++p1 ) {
            // for each polygon, try to look intersections in other
            // polygons
            int p2 = (p1+1) % polygons[i].size(); 
            bool farP1=true,farP2 = true;
            int n1, n2;
            // if the distance between two corners of distinct polygons is less
            // than a threshold, we'll consider both points the same (merge them)
            for ( unsigned int k=0; (farP1 or farP2) and k < nodes.size(); ++k ) {
                double distP1 = cv::norm(cv::Mat(polygons[i][p1]), cv::Mat(nodes[k]));
                double distP2 = cv::norm(cv::Mat(polygons[i][p2]), cv::Mat(nodes[k]));
                if (farP1 and distP1 < THRESH ) { farP1 = false; n1=k; }
                if (farP2 and distP2 < THRESH ) { farP2 = false; n2=k; }
            } // for
            // here we'll test the distances of the evaluated points
            // and merge or not into the graph
            if ( farP1 and farP2 ) {
                this->nodes.push_back( polygons[i][p1] );
                this->nodes.push_back( polygons[i][p2] );
                n1 = this->nodes.size()-2, n2 = this->nodes.size()-1;
            } else if ( !farP1 and farP2 ) {
                this->nodes.push_back( polygons[i][p2] ); 
                n2 = this->nodes.size()-1;
            } else if ( farP1 and !farP2 ) {
                this->nodes.push_back( polygons[i][p1] );
                n1 = this->nodes.size()-1;
            } // else if

            // ok. now we'll see if the edges between these two points overlap
            // other edges. if that is the case, let's remove the biggest.
            bool ok = true;
            cv::Point vec1 = this->nodes[n2] - this->nodes[n1];
            for ( unsigned int k=0; k < this->edges[n1].size(); ++k ) {
                cv::Point vec2 = this->nodes[edges[n1][k]] - this->nodes[n1];
                // for each edge that is connected to N1, we need to compute the angle between the vectors
                float angle = std::acos((vec1/cv::norm(vec1)).dot((vec2/cv::norm(vec2)))) * 180/M_PI;
                // if the angle is less than a threshold it means that the vectors are very close (almost parallel)
                // so, we can remove the bigger one.
                if ( angle < THRESH ) {
                    ok = false;
                    if ( cv::norm(vec1) < cv::norm(vec2) ) {
                        ok = true;
                        int n3 = this->edges[n1][k];
                        std::vector<int>::iterator itErase = 
                            std::find(this->edges[n3].begin(), this->edges[n3].end(), n1 );
                        this->edges[n3].erase(itErase);
                        this->edges[n1].erase(this->edges[n1].begin()+k);
                    } // else
                    //break;
               } // if 
            } // for
            // if the edges are not parallel or if we removed the biggest one
            // let's add the new edge
            if ( ok ) {
                this->edges[n1].push_back(n2);
                this->edges[n2].push_back(n1);
            } // if
        } // for
    } // for
}

void TrackBuilder::saveAssets( ) {
    processMap( );
    cv::Mat board = cv::Mat::ones(this->frame.rows, this->frame.cols, CV_8UC3) * 255;
    std::unordered_map<int, std::vector<int>>::iterator it;
    std::cout << "Generating evidences!" << std::endl;
    std::ofstream graphFile;
    graphFile.open( "graph.txt" );
    cv::Mat track = cv::Mat::zeros(this->frame.rows, this->frame.cols, CV_8UC3);
    mask.copyTo(track(this->box));


    /**
    * 1) Find the upper left corner of the outside and the inner circuit
    * 2) walk through all nodes trying to close a square and put all nodes into the visited list
    * 2.1) connect all the nodes that aren't in a square path in the following way: right, left, right and left
    * 3) find the upper left corner in the non visited nodes
    * 4) repeat step 2
    */

    // top left from outside circuit
    int pivot = 0;
    float minDist = this->frame.rows * this->frame.cols;
    for ( unsigned i=0; i < this->nodes.size(); ++i ) {
        float distToOrigin = cv::norm( this->nodes[i] - cv::Point(0,0) );
        if ( distToOrigin < minDist ) {
            pivot = i;
            minDist = distToOrigin;
        } // if
    } // for
    // top left from the inner circuit
    int innerPivot = 0;
    minDist = this->frame.rows * this->frame.cols;
    for ( unsigned i=0; i < this->nodes.size(); ++i ) {
        if ( (int)i == pivot ) continue;
        float distToOrigin = cv::norm( this->nodes[i] - this->nodes[pivot] );
        if ( distToOrigin < minDist ) {
            innerPivot = i;
            minDist = distToOrigin;
        } // if
    } // for

    // ok. now that we have both pivots, let's create the edges
    std::unordered_map<int, std::vector<int> > newEdges;
    bool innerCircuit = false;
    int thresh = 10;
    bool running = true;
    std::vector<int> processed;
    bool exitOrEntrance = true;
    int prevDirStatus = -1;
    int dirStatus = 0; // 0=right,1=down,2=left,3=up
    while( running && processed.size() < this->nodes.size() ) {
        // for each node look for adjacent nodes in the right order, based on dirStatus
        processed.push_back(pivot);
        cv::Point p1 = this->nodes[pivot];
        const std::vector<int>& e = this->edges[pivot];
        bool status[] = {false,false,false,false};
        int ids[] = {-1,-1,-1,-1};
        for ( unsigned i=0; i < e.size(); ++i ) {
            cv::Point p2 = this->nodes[e[i]];
            cv::Point dir = p2 - p1;
            
            if ( dir.x > thresh && std::abs(dir.y) < thresh ) { // right
                status[0] = true; ids[0] = e[i];
            } else if ( dir.y > thresh && std::abs(dir.x) < thresh ) { // bottom
                status[1] = true; ids[1] = e[i];
            } else if ( std::abs(dir.x) > thresh && std::abs(dir.y) < thresh ) { // left
                status[2] = true; ids[2] = e[i];
            } else if ( std::abs(dir.y) > thresh && std::abs(dir.x) < thresh ) { // top
                status[3] = true; ids[3] = e[i];
            } else { // error
                std::cerr << "TrackBuilder::saveAssets - wrong adjacent state p1[" 
                          << p1 << "] p2[" << p2 << "]" << std::endl;
                running = false;
                break;
            } // else
        } // for
        if ( !status[dirStatus] ) {
            running = false; // expecting node at dirStatus but it was found
            std::cerr << "TrackBuilder::saveAssets - expecting node at " << dirStatus << " but it wasn't found" << std::endl;
            break;
        } // if
        // ok. we found a new valid edge
        std::cout << "Dir status: " << dirStatus << std::endl;
        std::cout << this->nodes[pivot] << this->nodes[ids[dirStatus]] << std::endl;
        if ( innerCircuit ) {
            newEdges[ids[dirStatus]].push_back( pivot);
        } else { 
            newEdges[pivot].push_back( ids[dirStatus]);
        } // else

        unsigned altDirStatus = (dirStatus+1) % 4;
        unsigned prevAltDirStatus = (dirStatus+2) % 4;
        // if there is a third path, it can be an exit or an entrance,
        // let's connect it too
        if ( status[altDirStatus] && status[prevAltDirStatus] ) { // there is an exit or an entrance
            int n1,n2;
            if ( exitOrEntrance ) { 
                n1=pivot; n2=ids[altDirStatus]; 
            } else { 
                n1=ids[altDirStatus]; n2=pivot; 
            } // else
            newEdges[n1].push_back( n2 );
            exitOrEntrance  = !exitOrEntrance;
        } // if
        pivot = ids[dirStatus];
        if ( std::find( processed.begin(), processed.end(), pivot ) != processed.end() ) {
            // if we reached here, we completed the outer circuit, let's change the pivot 
            // and process the inner circuit with the same logic in mind
            pivot = innerPivot;
            innerCircuit = true;
        } // if
        prevDirStatus == dirStatus ? dirStatus = (dirStatus + 1 ) % 4 : prevDirStatus = dirStatus;

    } // while
    // ok, now that we have all the edges in the right direction, let's
    // save the graph and plot it to the image
    int edgeCounter = 0;
    std::unordered_map<int, std::vector<int>>::iterator itE;
    for ( itE = newEdges.begin(); itE != newEdges.end(); ++itE ) {
        cv::Point p1 = this->nodes[itE->first];
        cv::Point p = p1 + this->box.tl();
        graphFile << p.x << " " << p.y << " ";
        for ( unsigned i=0; i < itE->second.size(); ++i ) {
            cv::Point p2 = this->nodes[itE->second[i]];
            cv::Point p = p2 + this->box.tl();
            graphFile << p.x << " " << p.y << " ";
            cv::putText( this->mask, std::to_string(edgeCounter++), ((p2-p1)/2)+p1, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 1);
            cv::line( this->mask, p1, p2, cv::Scalar(0,0,255), 2);
            cv::circle( this->mask, p1 + ((p2-p1)*0.85), 2, cv::Scalar(255,255,0), -1, 10 );
        } // for
        graphFile << std::endl;
    } // for

    /*
    unsigned edgeCounter = 0;
    std::vector<cv::Point> visited; 
    //for ( it = this->edges.begin(); it != this->edges.end(); ++it ) {
    for ( unsigned j=0; j < newEdges.size(); ++j ) {
        //cv::Point p1 = this->nodes[newEdges[j].first];
        cv::Point p1 = this->nodes[it->first];
        cv::Point p = p1 + this->box.tl();
        if ( visited.end() != std::find(visited.begin(), visited.end(), p1 ) ) {
            continue;
        } // if
        visited.push_back(p1);
        std::vector<cv::Point> newPoints;
        newPoints.push_back(p);
        //graphFile << p.x << " " << p.y << " ";
        for ( unsigned int i=0; i < it->second.size(); ++i ) {
            cv::Point p2 = this->nodes[it->second[i]];
            cv::Point p = p2 + this->box.tl();
            if ( visited.end() != std::find(visited.begin(), visited.end(), p2 ) ) {
                continue;
            } // if
            //graphFile << p.x << " " << p.y << " ";
            newPoints.push_back(p);
        
           
            //visited.push_back(p1);
            cv::putText( this->mask, std::to_string(edgeCounter), ((p2-p1)/2)+p1, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 1);
            cv::line( this->mask, p1, p2, cv::Scalar(0,0,255), 2);
            cv::circle( this->mask, p1 + ((p2-p1)*0.85), 2, cv::Scalar(255,255,0), -1, 10 );
            ++edgeCounter;
        } // for
        if (newPoints.size() > 1 ) {
            for (unsigned i=0; i < newPoints.size(); ++i ) {
                graphFile << newPoints[i].x << " " << newPoints[i].y << " ";
            } // for
            graphFile << std::endl;
        } // if
        cv::circle( this->mask, this->nodes[it->first], 5, cv::Scalar(0,255,255), -1, 10 );
        cv::putText( this->mask, std::to_string(it->first), this->nodes[it->first], cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0,255,0), 1);
    } // for
    */


    graphFile.close();
    mask.copyTo(board(this->box));

    cv::imwrite("board.jpg", board );
    cv::imwrite("track.jpg", track );
    cv::imwrite("frame.jpg", this->frame );
}

int main(int argc, char** argv){
    try {
        std::string usage = argv[0] + std::string(" [-f <int> -w <int> -h <int> -c <int> -i <img_filename>]");
        int fps = CONF(int,"fps"), width = CONF(int, "width"), height = CONF(int, "height"), camid = CONF(int, "camid");
        bool ok = true;
        std::string imgFilename;
        int opt;
        while (ok and (opt = getopt(argc, argv, "f:w:h:c:i:")) != -1) {
            switch (opt) {
                case 'f': fps = atoi(optarg); break;
                case 'w': width = atoi(optarg); break;
                case 'h': height = atoi(optarg); break;
                case 'c': camid = atoi(optarg); break;
                case 'i': imgFilename = optarg; break;
                default: std::cerr << usage << std::endl; ok = false;
            } // switch
        } // while
        if (ok) {
            if ( imgFilename == "" ) {
                TrackBuilder(width, height, camid, fps).saveAssets(); 
            } else {
                TrackBuilder(imgFilename).saveAssets(); 
            } // else
        } // if
    } catch( const cv::Exception& ex ) {
        std::cerr << ex.code << " " << ex.err <<  " " << ex.file << " " << ex.func << " " << ex.line << " " << ex.msg << std::endl;
    } catch( const std::exception& ex ) {
        std::cerr << ex.what() << std::endl;
    } // catch

    return 0;
}
