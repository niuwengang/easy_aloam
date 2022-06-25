#include "../head.hpp"

class FeatureExtract
{
public:
    FeatureExtract() {}                  //构造函数
    FeatureExtract(ros::NodeHandle &nh); //初始化工作
    ~FeatureExtract() {}                 //析构函数空

private:
    void PointCloudCallback(const sensor_msgs::PointCloud2Ptr &msg);
    void RemoveClosedPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out, float threshold_dis);

private:
    std::mutex _buff_mutex;                 //变量锁
    ros::Subscriber _pointcloud_subscriber; //点云订阅器
    ros::Publisher _pointcloud_publisher;   //点云发布器

     pcl::PointCloud<pcl::PointXYZ> laserCloud;//原始点云信息

    // template <typename PointT>
    // void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float threshold_dis);
    // void pointcloudCallback(const sensor_msgs::PointCloud2Ptr &msg);
    // template <typename PointT>
    // void pointcloudPub(pcl::PointCloud<PointT> &pointcloud, const sensor_msgs::PointCloud2Ptr &msg);
};
