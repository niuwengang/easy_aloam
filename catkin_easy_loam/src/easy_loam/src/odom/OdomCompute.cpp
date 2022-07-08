#include "./OdomCompute.h"

OdomCompute::OdomCompute(
    ros::NodeHandle &nh)
{
    _conerPointsSharp_subscriber = nh.subscribe("/cornerPointsSharp", 100, &OdomCompute::LaserCloudSharpCallback, this);
    _conerPointsLessSharp_subscriber = nh.subscribe("/cornerPointsLessSharp", 100, &OdomCompute::LaserCloudLessSharpCallback, this);
    _surfPointsFlat_subscriber = nh.subscribe("/surfPointsFlat", 100, &OdomCompute::LaserCloudFlatCallback, this);
    _surfPointsLessFlat_subscriber = nh.subscribe("/surfPointsLessFlat", 100, &OdomCompute::LaserCloudLessFlatCallback, this);
    _laserCloudFullRes_subscriber = nh.subscribe("/pointCloud_2", 100, &OdomCompute::LaserCloudFullResCallback, this);

    ros::Rate sleep(100);
    while (ros::ok)
    {
        /**回调函数接收点云数据**/
        ros::spinOnce();
        if (_cornerSharpBuf.size() != 0 &&_cornerLessSharpBuf.size() != 0 &&_surfFlatBuf.size() != 0 &&_surfLessFlatBuf.size()!= 0 &&_fullPointsBuf.size() != 0)
        {
            Info("数据被完整接收到");
           // int timeCornerPointsSharp=_cornerSharpBuf.front()
        }
    }
}

void OdomCompute::LaserCloudSharpCallback(const sensor_msgs::PointCloud2Ptr &msg)
{
    mBuf.lock();
    _cornerSharpBuf.push(msg);
    mBuf.unlock();
}

void OdomCompute::LaserCloudLessSharpCallback(const sensor_msgs::PointCloud2Ptr &msg)
{
    mBuf.lock();
    _cornerLessSharpBuf.push(msg);
    mBuf.unlock();
}

void OdomCompute::LaserCloudFlatCallback(const sensor_msgs::PointCloud2Ptr &msg)
{
    mBuf.lock();
    _surfFlatBuf.push(msg);
    mBuf.unlock();
}

void OdomCompute::LaserCloudLessFlatCallback(const sensor_msgs::PointCloud2Ptr &msg)
{
    mBuf.lock();
    _surfLessFlatBuf.push(msg);
    mBuf.unlock();
}

void OdomCompute::LaserCloudFullResCallback(const sensor_msgs::PointCloud2Ptr &msg)
{
    mBuf.lock();
    _fullPointsBuf.push(msg);
    mBuf.unlock();
}
