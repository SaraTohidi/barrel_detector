#include "ros/ros.h"
#include <cmath>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "barrel_detector/barrel_detector.h"


using namespace std;


namespace visualize_scanner {

VisualizeScanner::VisualizeScanner() :counter_(0),node_private("~/")
{
    timer_ = ros::Time::now();
    initializeVisualizeScanner();
}

//-------initializing---------
void VisualizeScanner::initializeVisualizeScanner()
{

    node_private.param("debuging", debuging_, false);
    node_private.param("probeblisitic", probeblisitic_, true);
    node_private.param("min_radius", min_radius_, 20.0);
    node_private.param("max_radius", max_radius_, 40.0);
    node_private.param("min_circle_percentage", min_circle_percentage_, 0.50);

    scan_sub_ = node_private.subscribe("/scan", 5, &VisualizeScanner::scanCallback, this);

    barrels_vis_pub_ = node_private.advertise<visualization_msgs::MarkerArray>("/circles" , 100);

    barrels_pub_ = node_private.advertise<geometry_msgs::PoseArray>("/barrels" , 100);

    dsrv_ = new dynamic_reconfigure::Server<barrel_detector::BarrelDetectorConfig>(node_private);
    dynamic_reconfigure::Server<barrel_detector::BarrelDetectorConfig>::CallbackType cb = boost::bind(&VisualizeScanner::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    ros::Rate loop(10);

    while(ros::ok())
    {
        if( (ros::Time::now() - timer_) > ros::Duration(2.0,0) )
        {
            timer_ = ros::Time::now();
            ros::spinOnce();
        }
        loop.sleep();
    }

    ros::waitForShutdown();
}

//-------config---------
void VisualizeScanner::reconfigureCB(barrel_detector::BarrelDetectorConfig &config, uint32_t level)
{
    debuging_ = config.debuging;
    probeblisitic_ = config.probeblisitic;
    min_radius_ = config.min_radius;
    max_radius_ = config.max_radius;
    min_circle_percentage_ = config.min_circle_percentage;
    ROS_INFO_STREAM(debuging_ << "   "  << probeblisitic_ << "   "  << min_radius_ << "   " << max_radius_ << "   " << min_circle_percentage_);
}

//-------calling functions--------
void VisualizeScanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_data)
{
    ROS_WARN_ONCE("Recived Laser Data");

    std::vector<cv::Point2f> circles;
    cv::Mat scan_mat;
    geometry_msgs::PoseArray detected_barrel;

    scan_mat = convertLaserScanPointsToMatrix(*laser_data);

    if (debuging_)
    {
        saveImage(scan_mat, "scan");
    }

    removeLines(scan_mat, probeblisitic_);

    if (debuging_)
    {
        saveImage(scan_mat, "line");
    }
    cv::Mat out;
    circles = detectCircles(scan_mat, circles, out);
    if (debuging_)
    {
        saveImage(out, "circle");
    }

    detected_barrel = convertToWorld(circles);

    visualizeBarrels(detected_barrel);

    publishBarrels(detected_barrel);
}

//-------converting laserscanner's data to matrix---------
cv::Mat VisualizeScanner::convertLaserScanPointsToMatrix(const sensor_msgs::LaserScan&  data)
{
    float angles = data.angle_min;

    cv::Mat scan_mat(800 , 400, CV_8UC1, 0.0 );

    for(int i = 0; i < data.ranges.size()  ; i++)
    {
        angles += (data.angle_increment);

        if(angles > -1.57 && angles < 1.57)
        {
            if (data.ranges[i] > 0 && data.ranges[i] < 4)
            {

                float pointf_x = ( (cos(angles) * (data.ranges[i])) * 100 );
                float pointf_y = ( (sin(angles) * (data.ranges[i])) * 100 );

                int point_x = pointf_x ;
                int point_y = pointf_y + 400;

                if(point_x < 0 || point_y < 0)
                {
                    ROS_ERROR_STREAM(point_x << "  " << point_y);
                    ROS_INFO_STREAM(data.ranges[i] << "   " << angles);
                }

                scan_mat.at<uchar>(point_y , point_x) = 255;
            }else continue;
        }
    }

    return scan_mat;

}

//-------saving the Matrix images---------
bool VisualizeScanner::saveImage(const cv::Mat image, const std::string& name)
{

    std::ostringstream sstream;
    sstream << "/home/delnia/pics/" << counter_++<< name  << ".png";

    cv::imwrite(sstream.str() , image);
}

//-------remove lines-------
bool VisualizeScanner::removeLines(cv::Mat &src, bool probeblisitic)
{
    cv::Mat gray,color;
    if (src.channels() == 3)
    {
        cvtColor(src, gray, CV_RGB2GRAY);
        color = src;
    }
    else
    {
        gray = src;
        cvtColor(src, color, CV_GRAY2RGB);
    }

    if(probeblisitic)
    {
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(gray, lines, 1, CV_PI/90, 20, 25, 30);
        for(size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            cv::line( color, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,0), 3, CV_AA );
        }
    }
    else
    {
        std::vector<cv::Vec2f> lines;
        cv::HoughLines(gray, lines, 1, CV_PI/90, 5, 100, 100);
        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0], theta = lines[i][1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 100*(-b));
            pt1.y = cvRound(y0 + 100*(a));
            pt2.x = cvRound(x0 - 100*(-b));
            pt2.y = cvRound(y0 - 100*(a));
            line( color, pt1, pt2, cv::Scalar(0,0,0), 3, CV_AA);
        }
    }

    if (src.channels() == 3)
    {
        src = color;
    }
    else
    {
        cvtColor(color, src, CV_RGB2GRAY);
    }

}

//-------detecting circles---------
std::vector<cv::Point2f> VisualizeScanner::detectCircles( cv::Mat &src, std::vector<cv::Point2f> &circles, cv::Mat &out)
{
    cv::Mat gray;
    if (src.channels() == 3)
    {
        out = src;
        cvtColor(src, gray, CV_RGB2GRAY);
    }
    else
    {
        gray = src;
        cvtColor(src, out, CV_GRAY2RGB);
    }
    //TODO REMOVE DETECTED CIRCLES FROM IMAGE OR REMOVE THEM BEFORE EVEN START CONVERTING TO BINARY?
    std::vector<cv::Point2f> get_position;
    get_position = getPointPositions(gray);

    cv::Mat dt;
    cv::distanceTransform(255-gray, dt,CV_DIST_L1, 3);

    //    float min_radius = 20;   // TODO: ADJUST THIS PARAMETER TO YOUR NEEDS, otherwise smaller circles wont be detected or "small noise circles" will have a high percentage of completion
    //    float max_radius = 40;   // TODO: ADJUST THIS PARAMETER TO YOUR NEEDS, otherwise smaller circles wont be detected or "small noise circles" will have a high percentage of completion

    //    float min_circle_percentage = 0.50f;  // at least 5% of a circle must be present? maybe more...

    for(unsigned int its=0; its< 1000; ++its)
    {
        if(get_position.size() > 10){
            //RANSAC: randomly choose 3 point and create a circle:
            //TODO: choose randomly but more intelligent,
            //so that it is more likely to choose three points of a circle.
            //For example if there are many small circles, it is unlikely to randomly choose 3 points of the same circle.
            unsigned int idx1 = rand()%get_position.size();
            unsigned int idx2 = rand()%get_position.size();
            unsigned int idx3 = rand()%get_position.size();

            // we need 3 different samples:
            if(idx1 == idx2) continue;
            if(idx1 == idx3) continue;
            if(idx3 == idx2) continue;

            // create circle from 3 points:
            cv::Point2f center; float radius;
            getCircle(get_position[idx1],get_position[idx2],get_position[idx3],center,radius);

            if(radius < min_radius_ || radius > max_radius_)
            {
                // if the detected circle dosen't match with our maximum and minimum limits, we stop here.
                // TODO don't even chose the points that their circles is invalid.
                continue;
            }

            // inlier set unused at the moment but could be used to approximate a (more robust) circle from alle inlier
            std::vector<cv::Point2f> inlier_set;

            //verify or falsify the circle by inlier counting:
            if(!isnan(radius) || !isinf(radius));
            float c_perc = verifyCircle(dt,center,radius, inlier_set);

            if(c_perc >= min_circle_percentage_)
            {
                //                ROS_INFO_STREAM(c_perc);

                // std::cout << "accepted circle with " << cPerc*100.0f << " % inlier" << std::endl;
                // first step would be to approximate the circle iteratively from ALL INLIER to obtain a better circle center
                // but that's a TODO
                cv::circle(out, center,radius, cv::Scalar(255,0,0),1);
                // accept circle => remove it from the edge list
                cv::circle(gray,center,radius + 10,cv::Scalar(0),-1);
                //update edge positions and distance transform
                get_position.clear();
                get_position = getPointPositions(gray);
                cv::distanceTransform(255-gray, dt,CV_DIST_L1, 3);
                circles.push_back(center);

                detected_barrels_.insert(detected_barrels_.end(), circles.begin(), circles.end());

            }
        }
    }
    return circles;
}

//-------get position--------
std::vector<cv::Point2f> VisualizeScanner::getPointPositions(cv::Mat &binary_image)
{
    std::vector<cv::Point2f> point_positions;

    for(unsigned int y=0; y<binary_image.rows; y++)
    {
        for(unsigned int x=0; x<binary_image.cols; x++)
        {
            if(binary_image.at<unsigned char>(y,x) > 0) point_positions.push_back(cv::Point2f(x,y));
        }
    }

    return point_positions;

}

//-------get circles--------
inline void VisualizeScanner::getCircle(cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &p3, cv::Point2f &center, float &radius)
{
    float x1 = p1.x;
    float x2 = p2.x;
    float x3 = p3.x;

    float y1 = p1.y;
    float y2 = p2.y;
    float y3 = p3.y;

    // PLEASE CHECK FOR TYPOS IN THE FORMULA :)
    center.x = (x1*x1+y1*y1)*(y2-y3) + (x2*x2+y2*y2)*(y3-y1) + (x3*x3+y3*y3)*(y1-y2);
    center.x /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

    center.y = (x1*x1 + y1*y1)*(x3-x2) + (x2*x2+y2*y2)*(x1-x3) + (x3*x3 + y3*y3)*(x2-x1);
    center.y /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

    radius = sqrt((center.x-x1)*(center.x-x1) + (center.y-y1)*(center.y-y1) );
}

//-------verify circles--------
float VisualizeScanner::verifyCircle(cv::Mat dt, cv::Point2f center, float radius, std::vector<cv::Point2f> &inlier_set)
{
    unsigned int counter = 0;
    unsigned int inlier = 0;
    //    float minInlierDist = 3.0f;
    //    float maxInlierDistMax = 50.0f;
    float maxInlierDist = radius/6.0f;
    //    if(maxInlierDist<minInlierDist) maxInlierDist = minInlierDist;
    //    if(maxInlierDist>maxInlierDistMax) maxInlierDist = maxInlierDistMax;
    // choose samples along the circle and count inlier percentage
    for(float t =0; t<2*3.14159265359f; t+= 0.005f)
    {
        counter++;
        float cX = radius*cos(t) + center.x;
        float cY = radius*sin(t) + center.y;

        if(cX < dt.cols)
            if(cX >= 0)
                if(cY < dt.rows)
                    if(cY >= 0)
                        if(dt.at<float>(cY,cX) < maxInlierDist)
                        {
                            inlier++;
                            inlier_set.push_back(cv::Point2f(cX,cY));
                        }
    }
    return (float)inlier/float(counter);
}

//------convert circles pose to world-------
geometry_msgs::PoseArray  VisualizeScanner::convertToWorld(std::vector<cv::Point2f> circles)
{
    geometry_msgs::Pose centers;
    geometry_msgs::PoseArray  detected_barrels;

    for(int i = 0 ; i < circles.size() ; i++)
    {
        float point_x = circles[i].x ;
        float point_y = circles[i].y - 400 ;

        point_x = point_x / 100 ;
        point_y = point_y / 100 ;

        centers.position.x = point_x;
        centers.position.y = point_y;
        centers.position.z = 0;

        centers.orientation = tf::createQuaternionMsgFromYaw(0);

        detected_barrels.poses.push_back(centers);
    }
    detected_barrels.header.frame_id = "/laser";
    detected_barrels.header.stamp = ros::Time::now();

    return detected_barrels;
}

//-------------------visualize barrels-------------
void VisualizeScanner::visualizeBarrels(geometry_msgs::PoseArray  detected_barrels)
{
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray barrel_center;

    marker.header.frame_id = "/laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_barrels";

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.7;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.b = 1.0;
    marker.color.g = 0.0;

    marker.lifetime = ros::Duration(2.0,0);

    for(int i = 0 ; i < detected_barrels.poses.size() ; i++)
    {
        marker.id = i;

        marker.pose = detected_barrels.poses.at(i);

        barrel_center.markers.push_back(marker);
    }

    barrels_vis_pub_.publish(barrel_center);
}

//-------------------publishing barrels -------------
void VisualizeScanner::publishBarrels(geometry_msgs::PoseArray  detected_barrels)
{
    if(detected_barrels.poses.size() > 0)
        barrels_pub_.publish(detected_barrels);
}

VisualizeScanner::~VisualizeScanner(){}

}

//----------------The main-------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "VisualizeScanner" );

    visualize_scanner::VisualizeScanner visualize_scanner;

    return 0;
}



