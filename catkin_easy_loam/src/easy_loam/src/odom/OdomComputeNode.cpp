#include "../head.h"
#include "OdomCompute.h"

shared_ptr<OdomCompute> OdomComputePtr;//特征点提取

int main(int argc, char **argv)
{

    el::Configurations conf(ros::package::getPath("easy_loam") + "/src/log/log.conf");
    el::Loggers::reconfigureAllLoggers(conf);
    LOG(TRACE) << "里程计计算启动";
    ros::init(argc, argv, "OdomCompute");
    ros::NodeHandle nh;

    OdomComputePtr = make_shared<OdomCompute>(nh);

    ros::spin();
    return 0;
}
