#pragma once

#include <ros/ros.h>
#include <ros/spinner.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include "alignment.h"

class AlignmentServer
{
public:
    using CameraInfo = sensor_msgs::CameraInfo;
    using DepthImageMsg = sensor_msgs::Image;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<CameraInfo, CameraInfo, DepthImageMsg>;

private:
    /* data */
    ros::NodeHandle nh_;
    ros::Publisher pub_;

    message_filters::Subscriber<DepthImageMsg> *depth_img_sub_;
    message_filters::Subscriber<CameraInfo> *color_info_sub_;
    message_filters::Subscriber<CameraInfo> *depth_info_sub_;
    message_filters::Synchronizer<SyncPolicy> *sync_;

    Alignment align_;
    TY_CAMERA_CALIB_INFO color_calib_;
    TY_CAMERA_CALIB_INFO depth_calib_;

public:
    AlignmentServer(std::string color_info_topic_name, std::string depth_info_topic_name, std::string depth_topic_name, std::string pub_topic_name);
    ~AlignmentServer();

    void subCallBack(const CameraInfo::ConstPtr &color_info, const CameraInfo::ConstPtr &depth_info, const DepthImageMsg::ConstPtr &depth_img);
};