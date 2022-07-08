#include "../head.h"

class OdomCompute
{
public:
    OdomCompute() {}
    OdomCompute(ros::NodeHandle &nh);
    ~OdomCompute() {}

private:
    void LaserCloudSharpCallback(const sensor_msgs::PointCloud2Ptr &msg);
    void LaserCloudLessSharpCallback(const sensor_msgs::PointCloud2Ptr &msg);
    void LaserCloudFlatCallback(const sensor_msgs::PointCloud2Ptr &msg);
    void LaserCloudLessFlatCallback(const sensor_msgs::PointCloud2Ptr &msg);
    void LaserCloudFullResCallback(const sensor_msgs::PointCloud2Ptr &msg);

private:
    ros::Subscriber _conerPointsSharp_subscriber;
    ros::Subscriber _conerPointsLessSharp_subscriber;
    ros::Subscriber _surfPointsFlat_subscriber;
    ros::Subscriber _surfPointsLessFlat_subscriber;
    ros::Subscriber _laserCloudFullRes_subscriber;

    queue<sensor_msgs::PointCloud2ConstPtr> _cornerSharpBuf;
    queue<sensor_msgs::PointCloud2ConstPtr> _cornerLessSharpBuf;
    queue<sensor_msgs::PointCloud2ConstPtr> _surfFlatBuf;
    queue<sensor_msgs::PointCloud2ConstPtr> _surfLessFlatBuf;
    queue<sensor_msgs::PointCloud2ConstPtr> _fullPointsBuf;

    mutex mBuf;
};