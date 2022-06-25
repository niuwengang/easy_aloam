#include "./FeatureExtract.h"

/**
 * @brief 初始化发布器及订阅器
 * @param 节点句柄
 * @return None
 * @note
 */
FeatureExtract::FeatureExtract(
    ros::NodeHandle &nh)
{
    _pointcloud_subscriber = nh.subscribe("/velodyne_points", 1, &FeatureExtract::PointCloudCallback, this); //可从rosbag中读取点云信息
    _pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);
}

/**
 * @brief  去除距离较近点
 * @param
 * @return
 * @note
 * 1.这里是模仿pcl库removeNaNFromPointCloud的写法但是去掉了模板适配
 * 2.通常是认为激光雷达周围点容易被障碍物遮挡
 */
void FeatureExtract::RemoveClosedPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloudIn, pcl::PointCloud<pcl::PointXYZ> &cloudOut, float thresholdDis)
{

    //根据输入调整下输出的格式
    // question? 这里的&写法是为什么
    if (&cloudIn != &cloudOut)
    {
        cloudOut.header = cloudIn.header;
        cloudOut.points.resize(cloudIn.points.size());
    }

    //滤去过远点
    size_t current_index = 0;
    for (size_t i = 0; i < cloudIn.points.size(); i++)
    {
        if ((pow(cloudIn.points[i].x, 2) + pow(cloudIn.points[i].y, 2) + pow(cloudIn.points[i].z, 2)) < pow(thresholdDis, 2))
            continue;
        cloudOut[current_index] = cloudIn.points[i];
        current_index++;
    }

    //滤波后点云会减少进行一下数据size压缩
    if (current_index != cloudIn.points.size())
    {
        cloudOut.points.resize(current_index);
    }
    //标记下点云信息 height=1表示是无序点云
    cloudOut.height = 1;
    cloudOut.width = static_cast<uint32_t>(current_index);
    cloudOut.is_dense = true;
}

/**
 * @brief 特征点提取回调函数
 * @param
 * @return
 * @note
 */
void FeatureExtract::PointCloudCallback(const sensor_msgs::PointCloud2Ptr &msg)
{
    //加变量锁
    _buff_mutex.lock();

    // todo:数据等待缓冲

    Info("读取原始点云数据");

    //起点终点索引
    vector<int> scanStartIndexVec(CONST_SCANS_N, 0);
    vector<int> scanEndIndexVec(CONST_SCANS_N, 0);

    //点云格式转换
   
    pcl::fromROSMsg(*msg, laserCloud);

    //滤波
    vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloud, laserCloud, indices);
    RemoveClosedPointCloud(laserCloud, laserCloud, CONST_MINIMUM_RANGE);
    //计算一下起始角度和终止角度
    float startOri = -atan2(laserCloud.points[0].y, laserCloud.points[0].x);
    float endOri = -atan2(laserCloud.points[laserCloud.points.size() - 1].y, laserCloud.points[laserCloud.points.size() - 1].x) + 2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
        endOri -= 2 * M_PI;
    else if (endOri - startOri < 1 * M_PI)
        endOri += 2 * M_PI;
    //根据俯仰角归算点云到相应线
    //根据当前角度计算相对时间为点云加上相对时间戳
    vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScansVec(CONST_SCANS_N);
    int count = laserCloud.points.size();
    bool halfPassed = false;
    for (int i = 0; i < laserCloud.points.size(); i++)
    {
        pcl::PointXYZI pointTemp;
        pointTemp.x = laserCloud.points[i].x;
        pointTemp.y = laserCloud.points[i].y;
        pointTemp.z = laserCloud.points[i].z;

        float pitchCurrent = atan(pointTemp.z / sqrt(pow(pointTemp.x, 2) + pow(pointTemp.y, 2))) * 180 / M_PI;
        int scanID = 0;
        if (CONST_SCANS_N == 16)
        {
            scanID = int((pitchCurrent + 15.0) / 2 + 0.5); 
            if (scanID > (CONST_SCANS_N - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            Error("激光雷达线数设置错误");
            ROS_BREAK();
        }

        float currentOri = -atan2(pointTemp.y, pointTemp.x);

        if (!halfPassed) ///过半处理一下
        {
            if (currentOri < startOri - M_PI / 2)
            {
                currentOri += 2 * M_PI;
            }
            else if (currentOri > startOri + M_PI * 3 / 2)
            {
                currentOri -= 2 * M_PI;
            }

            if (currentOri - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            currentOri += 2 * M_PI;
            if (currentOri < endOri - M_PI * 3 / 2)
            {
                currentOri += 2 * M_PI;
            }
            else if (currentOri > endOri + M_PI / 2)
            {
                currentOri -= 2 * M_PI;
            }
        }

        float relativeTime = (currentOri - startOri) / (endOri - startOri); 

        pointTemp.intensity = scanID + CONST_SCANS_PERIOD * relativeTime;
        laserCloudScansVec[scanID].push_back(pointTemp);
    }
}






















    //     /*
    //    laserCloudScans组合成laserCloud
    //    [5,size-6]
    //    每个scanID去掉的点序
    //    前端：[0] [1] [2] [3] [4] 5个点云数据 从第五个开始
    //    后端: [size-1] [size-2] [size-3] [size-4] [size-5] 5个点云数据 从第[size-6]个开始
    //    */
    //     int cloudSize = count;
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
    //     for (int i = 0; i < N_SCANS; i++)
    //     {
    //         scanStartInd[i] = laserCloud->size() + 5;
    //         *laserCloud += laserCloudScansVec[i];
    //         scanEndInd[i] = laserCloud->size() - 6;

    //         // cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
    //         // LOG(TRACE) << "scanID: " << i;
    //         // LOG(TRACE) << "scanStartInd:" << scanStartInd[i];
    //         // LOG(TRACE) << "scanStartInd:" << scanEndInd[i];
    //         // cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
    //     }

    //     struct cloudTag_template
    //     {
    //         float cloudCurvature;
    //         int cloudSortInd;
    //         int cloudNeighborPicked;
    //         int cloudLabel;
    //     } cloudTag[400000];
    //     /*
    //       模拟曲率每段算一个曲率作为提取特征点的参考
    //       对整体的点云做一个滑窗口laserCloud(单纯提取特征点无需考虑点云所在的scanID)
    //       [i-5]-[i] [i-4]-[i] [i-3]-[i] [i-2]-[i] [i-1]-[i]
    //       [i+5]-[i] [i+4]-[i] [i+3]-[i] [i+2]-[i] [i+1]-[i]
    //       曲率越来越来则差值和越大

    //       */
    //     for (int i = 5; i < cloudSize; i++)
    //     {
    //         float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x\ 
//         - 10 * laserCloud->points[i].x +
    //                       laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;

    //         float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y\ 
//         - 10 * laserCloud->points[i].x +
    //                       laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;

    //         float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z\ 
//         - 10 * laserCloud->points[i].z +
    //                       laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

    //         cloudTag[i].cloudCurvature = pow(diffX, 2) + pow(diffY, 2) + pow(diffZ, 2);
    //         cloudTag[i].cloudSortInd = i;
    //         cloudTag[i].cloudLabel = 0;
    //     }
    //     /*
    // 根据曲率提取四种特征点
    // 角点/弱化的角点
    // 平面点/弱化的平面点
    // */
    //     pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat;

    //     for (int i = 0; i < N_SCANS; i++)
    //     {
    //         if (scanEndInd[i] - scanStartInd[i] < 6)
    //             continue;
    //         for (int j = 0; j < 6; j++)
    //         {
    //         }
    //     }

    //     /*可视化发布点云信息*/
    //     pointcloudPub(laserCloudIn, msg);
    //     _buff_mutex.unlock();
//}

// template <typename PointT>
// void pointcloudPreprocess::pointcloudPub(pcl::PointCloud<PointT> &pointcloud, const sensor_msgs::PointCloud2Ptr &msg)
// {
//     sensor_msgs::PointCloud2 pointcloudMsg;
//     pcl::toROSMsg(pointcloud, pointcloudMsg);
//     pointcloudMsg.header.stamp = msg->header.stamp;
//     pointcloudMsg.header.frame_id = "/camera_init";
//     _pointcloud_publisher.publish(pointcloudMsg);
// }