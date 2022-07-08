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
    _pointCloud_subscriber = nh.subscribe("/velodyne_points", 1, &FeatureExtract::PointCloudCallback, this); //可从rosbag中读取点云信息
    
    _cornerPointsSharp_publisher = nh.advertise<sensor_msgs::PointCloud2>("/cornerPointsSharp", 100);
    _cornerPointsLessSharp_publisher = nh.advertise<sensor_msgs::PointCloud2>("/cornerPointsLessSharp", 100);
    _surfPointsFlat_publisher = nh.advertise<sensor_msgs::PointCloud2>("/surfPointsFlat", 100);
    _surfPointsLessFlat_publisher = nh.advertise<sensor_msgs::PointCloud2>("/surfPointsLessFlat", 100);
    _pointCloud_publisher= nh.advertise<sensor_msgs::PointCloud2>("/pointCloud_2", 100);
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
 * @param msg
 * @return 原始的点云信息
 * @note
 */
void FeatureExtract::PointCloudCallback(const sensor_msgs::PointCloud2Ptr &msg)
{

    /*清空类内全局变量*/
    _laserCloud.clear();
    for (int index = 0; index < CONST_SCANS_N; index++)
    {
        _laserCloudScansVec[index].clear();
    }
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfPointsFlat.clear();
    _surfPointsLessFlat.clear();

    /*加变量锁*/
    _buff_mutex.lock();

    Info("读取原始点云数据");

    /*起点终点索引*/
    vector<int> scanStartIndexVec(CONST_SCANS_N, 0);
    vector<int> scanEndIndexVec(CONST_SCANS_N, 0);

    /*点云格式转换*/
    pcl::fromROSMsg(*msg, _laserCloud);

    /*滤波 去除无效点+去除较近点*/
    vector<int> indices;
    pcl::removeNaNFromPointCloud(_laserCloud, _laserCloud, indices);
    RemoveClosedPointCloud(_laserCloud, _laserCloud, CONST_MINIMUM_RANGE);

    /*计算一下起始角度和终止角度*/
    float startOri = -atan2(_laserCloud.points[0].y, _laserCloud.points[0].x);
    float endOri = -atan2(_laserCloud.points[_laserCloud.points.size() - 1].y, _laserCloud.points[_laserCloud.points.size() - 1].x) + 2 * M_PI;

    endOri += endOri - startOri > 3 * M_PI ? -2 * M_PI : (endOri - startOri < 1 * M_PI ? 2 * M_PI : 0);

    /* 根据pitch角归类点云所在scanID+过圈处理+对点云打上相对时间戳*/
    int count = _laserCloud.points.size();
    bool halfPassed = false;
    for (int i = 0; i < _laserCloud.points.size(); i++)
    {
        pcl::PointXYZI pointTemp;
        pointTemp.x = _laserCloud.points[i].x;
        pointTemp.y = _laserCloud.points[i].y;
        pointTemp.z = _laserCloud.points[i].z;

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

        if (!halfPassed)
        {
            currentOri += currentOri - startOri < -M_PI / 2 ? 2 * M_PI : (currentOri - startOri > M_PI * 3 / 2 ? -2 * M_PI : 0);
            halfPassed = currentOri - startOri > M_PI ? true : false;
        }
        else
        {
            currentOri += 2 * M_PI;
            currentOri += currentOri - endOri < -M_PI * 3 / 2 ? 2 * M_PI : (currentOri - endOri > M_PI / 2 ? -2 * M_PI : 0);
        }

        float relativeTime = (currentOri - startOri) / (endOri - startOri);

        pointTemp.intensity = scanID + CONST_SCANS_PERIOD * relativeTime;
        _laserCloudScansVec[scanID].push_back(pointTemp);
    }
    /*去除每个ID前后5点重组点云*/
    int cloudSize = count;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i < CONST_SCANS_N; i++)
    {
        scanStartIndexVec[i] = laserCloud->size() + 5; //删去[0][1][2][3][4]从[5]开始
        *laserCloud += _laserCloudScansVec[i];
        scanEndIndexVec[i] = laserCloud->size() - 6; //删去size-1] [size-2] [size-3] [size-4] [size-5] 从[size-6]开始
    }

    /*计算曲率作为提取特征点的参考并打标记 */
    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudTag[i].cloudCurvature = pow(diffX, 2) + pow(diffY, 2) + pow(diffZ, 2); //取L2范数
        cloudTag[i].cloudSortInd = i;
        cloudTag[i].cloudNeighborPicked = 0; //邻居点是否被筛选过
        cloudTag[i].cloudLabel = 0;
    }

    /*根据曲率分类出4种特征点*/
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfPointsFlat.clear();
    _surfPointsLessFlat.clear();

    for (int i = 0; i < CONST_SCANS_N; i++)
    {
        /**不足6个点时，弃用该线**/
        if (scanEndIndexVec[i] - scanStartIndexVec[i] < 6)
            continue;
        pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
        /**每个scan上再6等份**/
        for (int j = 0; j < 6; j++)
        {
            /***计算每段上(第j段)的起点终点***/
            int startPoint = scanStartIndexVec[i] + (scanEndIndexVec[i] - scanStartIndexVec[i]) * j / 6;         // 6*0/6=0
            int endPoint = scanStartIndexVec[i] + (scanEndIndexVec[i] - scanStartIndexVec[i]) * (j + 1) / 6 - 1; // 6*（0+1)/6-1=5

            /***按照曲率排序 从大到小***/
            sort(cloudTag + startPoint, cloudTag + endPoint + 1, [](cloudTagType A, cloudTagType B)
                 { return A.cloudCurvature < B.cloudCurvature; });

            /***1.提取角点***/
            int largetstPickNum = 0;
            for (int k = endPoint; k >= startPoint; k--) //注意是闭集合
            {
                int indexReal = cloudTag[k].cloudSortInd;
                if (cloudTag[indexReal].cloudNeighborPicked == 0 && cloudTag[indexReal].cloudCurvature > 0.1)
                {
                    largetstPickNum++;
                    if (largetstPickNum <= 2)
                    {
                        cloudTag[indexReal].cloudLabel = 2;
                        _cornerPointsSharp.push_back(laserCloud->points[indexReal]);
                        _cornerPointsLessSharp.push_back(laserCloud->points[indexReal]);
                    }
                    else if (largetstPickNum <= 20)
                    {
                        cloudTag[indexReal].cloudLabel = 1;
                        _cornerPointsLessSharp.push_back(laserCloud->points[indexReal]);
                    }
                    else
                    {
                        break;
                    }
                    /****标记一下选中点****/
                    cloudTag[indexReal].cloudNeighborPicked = 1;
                    /****避免角点过于集中 先向右找****/
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[indexReal + l].x - laserCloud->points[indexReal + l - 1].x;
                        float diffY = laserCloud->points[indexReal + l].y - laserCloud->points[indexReal + l - 1].y;
                        float diffZ = laserCloud->points[indexReal + l].z - laserCloud->points[indexReal + l - 1].z;
                        if (pow(diffX, 2) + pow(diffY, 2) + pow(diffZ, 2) > 0.05)
                        {
                            break;
                        }
                        cloudTag[indexReal + l].cloudNeighborPicked = 1;
                    }
                    /****避免角点过于集中 再向左找****/
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[indexReal + l].x - laserCloud->points[indexReal + l + 1].x;
                        float diffY = laserCloud->points[indexReal + l].y - laserCloud->points[indexReal + l + 1].y;
                        float diffZ = laserCloud->points[indexReal + l].z - laserCloud->points[indexReal + l + 1].z;
                        if (pow(diffX, 2) + pow(diffY, 2) + pow(diffZ, 2) > 0.05)
                        {
                            break;
                        }
                        cloudTag[indexReal + l].cloudNeighborPicked = 1;
                    }
                }
            }
            /***2.提取平面点***/
            int smallestPickedNum = 0;
            for (int k = startPoint; k <= endPoint; k++)
            {
                int indexReal = cloudTag[k].cloudSortInd;
                if (cloudTag[indexReal].cloudNeighborPicked == 0 && cloudTag[indexReal].cloudCurvature < 0.1)
                {
                    cloudTag[indexReal].cloudLabel = -1;
                    _surfPointsFlat.push_back(laserCloud->points[indexReal]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }
                    /****标记一下选中点****/
                    cloudTag[indexReal].cloudNeighborPicked = 1;
                    /****避免角点过于集中 先向右找****/
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[indexReal + l].x - laserCloud->points[indexReal + l - 1].x;
                        float diffY = laserCloud->points[indexReal + l].y - laserCloud->points[indexReal + l - 1].y;
                        float diffZ = laserCloud->points[indexReal + l].z - laserCloud->points[indexReal + l - 1].z;
                        if (pow(diffX, 2) + pow(diffY, 2) + pow(diffZ, 2) > 0.05)
                        {
                            break;
                        }
                        cloudTag[indexReal + l].cloudNeighborPicked = 1;
                    }
                    /****避免角点过于集中 再向左找****/
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[indexReal + l].x - laserCloud->points[indexReal + l + 1].x;
                        float diffY = laserCloud->points[indexReal + l].y - laserCloud->points[indexReal + l + 1].y;
                        float diffZ = laserCloud->points[indexReal + l].z - laserCloud->points[indexReal + l + 1].z;
                        if (pow(diffX, 2) + pow(diffY, 2) + pow(diffZ, 2) > 0.05)
                        {
                            break;
                        }
                        cloudTag[indexReal + l].cloudNeighborPicked = 1;
                    }
                }
            }

            /***3.提取剩余点***/
            for (int k = startPoint; k <= endPoint; k++)
            {
                if (cloudTag[k].cloudLabel <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }
        /**滤波**/
        pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanFiltered;
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);  //输入滤波对象
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);             //设置滤波体素
        downSizeFilter.filter(surfPointsLessFlatScanFiltered); //输出滤波结果
        _surfPointsLessFlat += surfPointsLessFlatScanFiltered; //归于集合
    }
    /*点云发布*/
    PointCloudPub(*laserCloud, msg, _pointCloud_publisher);
    PointCloudPub(_cornerPointsSharp, msg, _cornerPointsSharp_publisher);
    PointCloudPub(_cornerPointsLessSharp, msg, _cornerPointsLessSharp_publisher);
    PointCloudPub(_surfPointsFlat, msg, _surfPointsFlat_publisher);
    PointCloudPub(_surfPointsLessFlat, msg, _surfPointsLessFlat_publisher);

    _buff_mutex.unlock();
}

template <typename PointT>
void FeatureExtract::PointCloudPub(pcl::PointCloud<PointT> &pointcloud, const sensor_msgs::PointCloud2Ptr &msg, ros::Publisher &pub)
{
    sensor_msgs::PointCloud2 pointcloudMsg;
    pcl::toROSMsg(pointcloud, pointcloudMsg);
    pointcloudMsg.header.stamp = msg->header.stamp;
    pointcloudMsg.header.frame_id = "/velodyne";
    pub.publish(pointcloudMsg);
}
