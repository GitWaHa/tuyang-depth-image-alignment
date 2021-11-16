#include "alignment_server.h"

#include <cv_bridge/cv_bridge.h>
#include <ctime>

AlignmentServer::AlignmentServer(std::string color_info_topic_name, std::string depth_info_topic_name, std::string depth_topic_name, std::string pub_topic_name)
{
    pub_ = nh_.advertise<sensor_msgs::Image>(pub_topic_name, 10);

    color_info_sub_ = new message_filters::Subscriber<CameraInfo>(nh_, color_info_topic_name, 1);
    depth_info_sub_ = new message_filters::Subscriber<CameraInfo>(nh_, depth_info_topic_name, 1);
    depth_img_sub_ = new message_filters::Subscriber<DepthImageMsg>(nh_, depth_topic_name, 1);

    sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *color_info_sub_, *depth_info_sub_, *depth_img_sub_);
    sync_->registerCallback(boost::bind(&AlignmentServer::subCallBack, this, _1, _2, _3));
}

AlignmentServer::~AlignmentServer()
{
}

void AlignmentServer::subCallBack(const CameraInfo::ConstPtr &color_info, const CameraInfo::ConstPtr &depth_info, const DepthImageMsg::ConstPtr &depth_img)
{
    static int counter = 0;
    align_.cameraInfoToTyCalibInfo(color_info, color_calib_, true);
    align_.cameraInfoToTyCalibInfo(depth_info, depth_calib_, false);

    cv_bridge::CvImageConstPtr pCvImage;
    cv::Mat input;
    cv::Mat out;
    pCvImage = cv_bridge::toCvShare(depth_img, depth_img->encoding);
    pCvImage->image.copyTo(input);
    align_.doRegister(depth_calib_, color_calib_, input, out);

    std_msgs::Header header = depth_img->header;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, depth_img->encoding, out).toImageMsg();

    // std::cout << "[info] "
    //           << "width:" << msg->width << ", height:" << msg->height << ", encoding:" << depth_img->encoding << std::endl;

    // cv_bridge
    pub_.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "alignment_server");

    AlignmentServer server("/camera/rgb/camera_info", "/camera/depth/camera_info",
                           "/camera/depth/image_raw",
                           "/camera/depth/image_align");

    ROS_INFO("alignment_server is started");
    ros::spin();
}
