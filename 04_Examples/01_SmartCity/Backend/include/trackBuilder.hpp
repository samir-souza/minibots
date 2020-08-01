#ifndef __TRACKBUILDER_HPP
#define __TRACKBUILDER_HPP

#include <map>
#include <vector>
#include <opencv2/opencv.hpp>

class TrackBuilder {
    private:
        static bool compareArea( std::vector<cv::Point>& p1, std::vector<cv::Point>& p2 );

        void getBoxFromContour(std::vector<cv::Point>& contour, cv::Rect& box );

        void getContours(const cv::Mat& img, std::vector<std::vector<cv::Point>>& contours, cv::Mat& thresh );

        float getCorrectAngle(float angle );

        cv::Point2f getNextDirection(float angle, float previousAngle);

        void expandPolygons( std::vector<std::vector<cv::Point>>& polygons );

        inline TrackBuilder() { }

        cv::Mat frame, mask;
        cv::Rect box;
        std::vector<cv::Point> nodes;
        std::unordered_map<int, std::vector<int>> edges;
    public:
        TrackBuilder(int width=1280, int height=720, int camid = 0, int fps=60);
        TrackBuilder(const std::string& fileName );
        
        void processMap( );

        void saveAssets( );

        inline ~TrackBuilder() {};
};

#endif // __TRACKBUILDER_HPP
