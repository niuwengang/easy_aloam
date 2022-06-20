#include "../head.h"

class pointcloudSub
{
public:
    pointcloudSub(
        ros::NodeHandle &nh,
        const std::string &topic_name,
        size_t buff_size);

private:
    std::mutex _buff_mutex;
    ros::Subscriber _subscriber;

    void pointcloudCallback(const sensor_msgs::PointCloud2Ptr &msg);
};