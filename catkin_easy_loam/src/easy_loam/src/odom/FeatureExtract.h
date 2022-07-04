#include "../head.h"

   struct cloudTagType
    {
        float cloudCurvature;
        int cloudSortInd;
        int cloudLabel;
        int cloudNeighborPicked;
    } cloudTag[400000];

class FeatureExtract
{
public:
    FeatureExtract() {}                  //构造函数
    FeatureExtract(ros::NodeHandle &nh); //初始化工作
    ~FeatureExtract() {}                 //析构函数空

private:
    void PointCloudCallback(const sensor_msgs::PointCloud2Ptr &msg);
    void RemoveClosedPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out, float threshold_dis);
    template <typename PointT>
    void PointCloudPub(pcl::PointCloud<PointT> &pointcloud, const sensor_msgs::PointCloud2Ptr &msg, ros::Publisher &pub);

private:
    std::mutex _buff_mutex;                                  //变量锁
    ros::Subscriber _pointCloud_subscriber;                  //点云订阅器
    ros::Publisher _pointCloud_publisher;                    //总体点云发布器
    array<ros::Publisher, 16> _pointCloudScansVec_publisher; //每线点云发布器
    ros::Publisher _cornerPointsSharp_publisher;
    ros::Publisher _cornerPointsLessSharp_publisher;
    ros::Publisher _surfPointsFlat_publisher;
    ros::Publisher _surfPointsLessFlat_publisher;

    pcl::PointCloud<pcl::PointXYZ> _laserCloud;                                //原始点云信息
    array<pcl::PointCloud<pcl::PointXYZI>, CONST_SCANS_N> _laserCloudScansVec; //点云线数
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp;
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsLessSharp;
    pcl::PointCloud<pcl::PointXYZI> _surfPointsFlat;
    pcl::PointCloud<pcl::PointXYZI> _surfPointsLessFlat;
};
