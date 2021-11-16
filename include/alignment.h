#pragma once

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "common/common.hpp"
#include "TYImageProc.h"

class Alignment
{
private:
    /* data */
public:
    Alignment(/* args */);
    ~Alignment();

    void cameraInfoToTyCalibInfo(const sensor_msgs::CameraInfo::ConstPtr &cameraInfo, TY_CAMERA_CALIB_INFO &depth_calib, bool is_color = true);
    void doRegister(const TY_CAMERA_CALIB_INFO &depth_calib, const TY_CAMERA_CALIB_INFO &color_calib, const cv::Mat &depth, cv::Mat &out);
};
