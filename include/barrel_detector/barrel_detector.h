#include <tf/tf.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/features2d/features2d.hpp>
#include <hector_worldmodel_msgs/AddObject.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <barrel_detector/BarrelDetectorConfig.h>


namespace visualize_scanner {

class VisualizeScanner
{

public:

    VisualizeScanner();
    ros::NodeHandle node_private;
    ~VisualizeScanner();

    bool saveImage(const cv::Mat image, const std::string &name);

private:

    void initializeVisualizeScanner();
    void reconfigureCB(barrel_detector::BarrelDetectorConfig &config, uint32_t level);

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_data);
    cv::Mat convertLaserScanPointsToMatrix(const sensor_msgs::LaserScan&  data);
    bool removeLines(cv::Mat &src, bool probeblisitic);
    std::vector<cv::Point2f> detectCircles(cv::Mat &src, std::vector<cv::Point2f> &circles, cv::Mat &out);
    std::vector<cv::Point2f> getPointPositions( cv::Mat &binary_image);
    inline void getCircle(cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &p3, cv::Point2f &center, float &radius);
    float verifyCircle(cv::Mat dt, cv::Point2f center, float radius, std::vector<cv::Point2f> &inlier_set);
    geometry_msgs::PoseArray convertToWorld(std::vector<cv::Point2f> circles);
    void publishBarrels(geometry_msgs::PoseArray detected_barrels);
    void visualizeBarrels(geometry_msgs::PoseArray  detected_barrels);
    dynamic_reconfigure::Server<barrel_detector::BarrelDetectorConfig> *dsrv_;

    ros::Publisher barrels_vis_pub_ ;
    ros::Publisher barrels_pub_;
    ros::Subscriber scan_sub_;
    ros::Time timer_;


    std::vector<cv::Point2f> detected_barrels_;

    double origin_x_, origin_y_, resolution_;
    double min_circle_percentage_;
    bool probeblisitic_;
    double min_radius_;
    double max_radius_;
    bool debuging_;
    int counter_;


};

}


