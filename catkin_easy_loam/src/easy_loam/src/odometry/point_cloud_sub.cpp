#include "point_cloud_sub.h"

pointcloudSub::pointcloudSub(
    ros::NodeHandle &nh,
    const std::string &topic_name,
    size_t buff_size)
{

    _subscriber = nh.subscribe(topic_name, buff_size, &pointcloudSub::pointcloudCallback, this);
}

void pointcloudSub::pointcloudCallback(
    const sensor_msgs::PointCloud2Ptr &msg)
{
    _buff_mutex.lock();
    Information("进入点云接收回调函数");
    TicToc t_whole;
    TicToc t_prepare;

    vector<int> scanStartInd(N_SCANS, 0);
    vector<int> scanEndInd(N_SCANS, 0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*msg,laserCloudIn);

    
    _buff_mutex.unlock();
}