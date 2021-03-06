#ifndef _HEAD_H
#define _HEAD_H

/*
c++库文件
*/
#include "bits/stdc++.h"
using namespace std;

/*
ros库文件
*/
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/package.h>

/*
pcl库文件
*/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

/* 
easylog库
*/
#include "./log/easylogging++.h"
INITIALIZE_EASYLOGGINGPP

/*
字体颜色
*/
#define fontColorReset "\033[0m"
#define fontColorBlack "\033[30m"   /* Black */
#define fontColorRed "\033[31m"     /* Red */
#define fontColorGreen "\033[32m"   /* Green */
#define fontColorYellow "\033[33m"  /* Yellow */
#define fontColorBlue "\033[34m"    /* Blue */
#define fontColorMagenta "\033[35m" /* Magenta */
#define fontColorCyan "\033[36m"    /* Cyan */
#define fontColorWhite "\033[37m"   /* White */

#define fontColorBlackBold "\033[1m\033[30m"   /* Bold Black */
#define fontColorRedBold "\033[1m\033[31m"     /* Bold Red */
#define fontColorGreenBold "\033[1m\033[32m"   /* Bold Green */
#define fontColorYellowBold "\033[1m\033[33m"  /* Bold Yellow */
#define fontColorBlueBold "\033[1m\033[34m"    /* Bold Blue */
#define fontColorMagentaBold "\033[1m\033[35m" /* Bold Magenta */
#define fontColorCyanBold "\033[1m\033[36m"    /* Bold Cyan */
#define fontColorWhiteBold "\033[1m\033[37m"   /* Bold White */

/*
参数宏定义
*/
#define CONST_SCANS_N  16    
#define CONST_MINIMUM_RANGE 0.3
#define CONST_SCANS_PERIOD  0.1 

/*
计时功能
*/
class TicToc
{
public:
    TicToc()
    {
        time_start = chrono::system_clock::now();
    }
    double toc()
    {
        time_end = chrono::system_clock::now();
        return chrono::duration_cast<chrono::milliseconds>(time_end - time_start).count();
    }
    // todo 这里其实可以重载<<直接打印

private:
    chrono::time_point<chrono::system_clock> time_start, time_end;
};



/*
代码调试信息
*/
#define Info(str) cout << fontColorBlueBold << "[Info]:" << str << fontColorReset << endl;
#define Debug(str, num) cout << fontColorGreen << "[Debug]:" << str << " " << num << fontColorReset << endl;
#define Error(str) cout << fontColorRedBold << "[Error]:" << str << fontColorReset << endl;


#endif