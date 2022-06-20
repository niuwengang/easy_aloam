#include "../head.hpp"

class pointcloudPreprocess
{
public:
    pointcloudPreprocess(ros::NodeHandle &nh);

private:
    std::mutex _buff_mutex;
    ros::Subscriber _pointcloud_subscriber;
    ros::Publisher _pointcloud_publisher;
    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float threshold_dis);
    void pointcloudCallback(const sensor_msgs::PointCloud2Ptr &msg);
    template <typename PointT>
    void pointcloudPub(pcl::PointCloud<PointT> &pointcloud, const sensor_msgs::PointCloud2Ptr &msg);
};

template <typename PointT>
void pointcloudPreprocess::removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float threshold_dis)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }
    size_t current_index = 0;
    for (size_t i = 0; i < cloud_in.points.size(); i++)
    {
        if ((pow(cloud_in.points[i].x, 2) + pow(cloud_in.points[i].y, 2) + pow(cloud_in.points[i].z, 2)) < pow(threshold_dis, 2))
            continue;
        cloud_out[current_index] = cloud_in.points[i];
        current_index++;
    }
    if (current_index != cloud_in.points.size())
    {
        cloud_out.points.resize(current_index);
    }
    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(current_index);
    cloud_out.is_dense = true;
}

pointcloudPreprocess::pointcloudPreprocess(
    ros::NodeHandle &nh)
{
    _pointcloud_subscriber = nh.subscribe("/velodyne_points", 100, &pointcloudPreprocess::pointcloudCallback, this);
    _pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);
}

void pointcloudPreprocess::pointcloudCallback(
    const sensor_msgs::PointCloud2Ptr &msg)
{
    _buff_mutex.lock();

    Info("读取原始点云数据");

    TicToc t_whole;
    TicToc t_prepare;

    vector<int> scanStartInd(N_SCANS, 0);
    vector<int> scanEndInd(N_SCANS, 0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*msg, laserCloudIn);
    /*序列索引*/
    vector<int> indices;
    /*
    滤波操作
    1.如果非稠密点云先去除无效点 有一个无效点云则为非dense
    2.去除距离较近的点 这里先设置为0.3
    */
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, 0.3);

  
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[laserCloudIn.points.size() - 1].y, laserCloudIn.points[laserCloudIn.points.size() - 1].x) + 2 * M_PI;
    
    if(endOri-startOri>3*M_PI)
    endOri-=2*M_PI;
    else if(endOri-startOri>3*M_PI)
    endOri+=2*M_PI;
    
    /*>>>>>>>>>>>>>>>debug测试>>>>>>>>>>>>>>>>>>*/
    Debug("点云起始角度", startOri);
    Debug("点云终止角度", endOri)
    /*>>>>>>>>>>>>>>>debug测试>>>>>>>>>>>>>>>>>>*/
        /*
        可视化发布点云信息
        */
        pointcloudPub(laserCloudIn, msg);
    _buff_mutex.unlock();
}

template <typename PointT>
void pointcloudPreprocess::pointcloudPub(pcl::PointCloud<PointT> &pointcloud, const sensor_msgs::PointCloud2Ptr &msg)
{
    sensor_msgs::PointCloud2 pointcloudMsg;
    pcl::toROSMsg(pointcloud, pointcloudMsg);
    pointcloudMsg.header.stamp = msg->header.stamp;
    pointcloudMsg.header.frame_id = "/camera_init";
    _pointcloud_publisher.publish(pointcloudMsg);
}