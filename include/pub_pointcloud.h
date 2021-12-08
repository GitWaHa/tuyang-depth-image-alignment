#pragma once

#include <mutex>

#include <ros/ros.h>
#include <ros/spinner.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

class Publisher
{
public:
    using CameraInfo = sensor_msgs::CameraInfo;
    using ColorImageMsg = sensor_msgs::Image;
    using DepthImageMsg = sensor_msgs::Image;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<CameraInfo, ColorImageMsg, CameraInfo, DepthImageMsg>;

public:
    Publisher() = delete;
    Publisher(std::string color_info_topic_name, std::string color_topic_name,
              std::string depth_info_topic_name, std::string depth_topic_name,
              std::string pub_topic_name);
    ~Publisher();

    void subCallBack(const CameraInfo::ConstPtr &color_info, const ColorImageMsg::ConstPtr &color_img,
                     const CameraInfo::ConstPtr &depth_info, const DepthImageMsg::ConstPtr &depth_img);

    void show();

private:
    void readRgbImage(const sensor_msgs::Image::ConstPtr &msgImage, cv::Mat &image) const
    {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        cv::cvtColor(pCvImage->image, image, cv::COLOR_BGR2RGB);
    }

    void readDepthImage(const sensor_msgs::Image::ConstPtr &msgImage, cv::Mat &image) const
    {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        pCvImage->image.copyTo(image);
    }

    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr &cameraInfo, cv::Mat &cameraMatrix) const
    {
        double *itC = cameraMatrix.ptr<double>(0, 0);
        for (size_t i = 0; i < 9; ++i, ++itC)
        {
            *itC = cameraInfo->K[i];
        }
    }

    void createLookup(size_t width, size_t height)
    {
        const float fx = 1.0f / camera_matrix_color_.at<double>(0, 0);
        const float fy = 1.0f / camera_matrix_color_.at<double>(1, 1);
        const float cx = camera_matrix_color_.at<double>(0, 2);
        const float cy = camera_matrix_color_.at<double>(1, 2);
        float *it;

        lookup_y_ = cv::Mat(1, height, CV_32F);
        it = lookup_y_.ptr<float>();
        for (size_t r = 0; r < height; ++r, ++it)
        {
            *it = (r - cy) * fy;
        }

        lookup_x_ = cv::Mat(1, width, CV_32F);
        it = lookup_x_.ptr<float>();
        for (size_t c = 0; c < width; ++c, ++it)
        {
            *it = (c - cx) * fx;
        }
    }

    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
    {
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

        // #pragma omp parallel for
        for (int r = 0; r < depth.rows; ++r)
        {
            pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
            const uint16_t *itD = depth.ptr<uint16_t>(r);
            const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
            const float y = lookup_y_.at<float>(0, r);
            const float *itX = lookup_x_.ptr<float>();

            for (size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
            {
                register const float depthValue = *itD / 1000.0f;
                // Check for invalid measurements
                if (*itD == 0)
                {
                    // not valid
                    itP->x = itP->y = itP->z = badPoint;
                    itP->rgba = 0;
                    continue;
                }
                itP->z = depthValue;
                itP->x = *itX * depthValue;
                itP->y = y * depthValue;
                itP->b = itC->val[0];
                itP->g = itC->val[1];
                itP->r = itC->val[2];
                itP->a = 255;
            }
        }
    }

    void cloudViewer()
    {
        // cv::Mat color, depth;
        pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
        const std::string cloudName = "rendered";

        // lock.lock();
        // color = this->color_;
        // depth = this->depth_;
        // updateCloud = false;
        // lock.unlock();

        visualizer->addPointCloud(cloud_, cloudName);
        visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
        visualizer->initCameraParameters();
        visualizer->setBackgroundColor(0, 0, 0);
        // visualizer->setSize(color.cols, color.rows);
        visualizer->setShowFPS(true);
        visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

        for (; !ros::isShuttingDown();)
        {
            lock_.lock();
            if (is_update_)
            {

                visualizer->updatePointCloud(cloud_, cloudName);
                is_update_ = false;
            }
            lock_.unlock();

            visualizer->spinOnce(100);
        }

        visualizer->close();
    }

private:
    /* data */
    ros::NodeHandle nh_;
    ros::Publisher pub_;

    message_filters::Subscriber<ColorImageMsg> *color_img_sub_;
    message_filters::Subscriber<DepthImageMsg> *depth_img_sub_;
    message_filters::Subscriber<CameraInfo> *color_info_sub_;
    message_filters::Subscriber<CameraInfo> *depth_info_sub_;
    message_filters::Synchronizer<SyncPolicy> *sync_;

    cv::Mat color_, depth_;
    cv::Mat camera_matrix_color_, camera_matrix_depth_;
    cv::Mat lookup_x_, lookup_y_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;

    std::mutex lock_;
    bool is_update_;
};