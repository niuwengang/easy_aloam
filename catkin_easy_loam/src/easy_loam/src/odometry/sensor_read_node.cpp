#include "../head.hpp"
#include "pointcloud_preprocess.hpp"

shared_ptr<pointcloudPreprocess> pointcloudPreprocess_ptr;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_read_node");
    ros::NodeHandle nh;

    pointcloudPreprocess_ptr = make_shared<pointcloudPreprocess>(nh);
    ros::spin();
    return 0;
}
