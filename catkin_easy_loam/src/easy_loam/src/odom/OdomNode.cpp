#include "../head.hpp"
#include "FeatureExtract.h"

shared_ptr<FeatureExtract> featureExtractPtr;//特征点提取

int main(int argc, char **argv)
{

    el::Configurations conf(ros::package::getPath("easy_loam") + "/src/log/log.conf");
    el::Loggers::reconfigureAllLoggers(conf);
    LOG(TRACE) << "easy-loam程序启动";
    ros::init(argc, argv, "OdomNode");
    ros::NodeHandle nh;

    featureExtractPtr = make_shared<FeatureExtract>(nh);

    ros::spin();
    return 0;
}
