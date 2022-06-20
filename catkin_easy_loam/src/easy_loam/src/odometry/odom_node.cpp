#include "../head.h"
#include "point_cloud_sub.h"

shared_ptr<pointcloudSub> pointcloudSub_ptr;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_node");
    ros::NodeHandle nh;

    pointcloudSub_ptr = make_shared<pointcloudSub>(nh, "/velodyne_points", 1);
    ros::spin();
    return 0;
}
