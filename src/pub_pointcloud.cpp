#include "pub_pointcloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>

Publisher::Publisher(std::string color_info_topic_name, std::string color_topic_name,
                     std::string depth_info_topic_name, std::string depth_topic_name,
                     std::string pub_topic_name)
    : cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>())
{
    camera_matrix_color_ = cv::Mat::zeros(3, 3, CV_64F);
    camera_matrix_depth_ = cv::Mat::zeros(3, 3, CV_64F);

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_topic_name, 10);

    color_info_sub_ = new message_filters::Subscriber<CameraInfo>(nh_, color_info_topic_name, 5);
    color_img_sub_ = new message_filters::Subscriber<ColorImageMsg>(nh_, color_topic_name, 5);
    depth_info_sub_ = new message_filters::Subscriber<CameraInfo>(nh_, depth_info_topic_name, 5);
    depth_img_sub_ = new message_filters::Subscriber<DepthImageMsg>(nh_, depth_topic_name, 5);

    sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *color_info_sub_, *color_img_sub_, *depth_info_sub_, *depth_img_sub_);
    sync_->registerCallback(boost::bind(&Publisher::subCallBack, this, _1, _2, _3, _4));
}

Publisher::~Publisher()
{
    delete color_img_sub_;
    delete depth_img_sub_;
    delete color_info_sub_;
    delete depth_info_sub_;
    delete sync_;
}

void Publisher::subCallBack(const CameraInfo::ConstPtr &color_info, const ColorImageMsg::ConstPtr &color_img,
                            const CameraInfo::ConstPtr &depth_info, const DepthImageMsg::ConstPtr &depth_img)
{
    static bool is_first = true;
    if (is_first)
    {
        readCameraInfo(color_info, camera_matrix_color_);
        readCameraInfo(depth_info, camera_matrix_depth_);
        createLookup(color_info->width, color_info->height);
        cloud_->height = color_info->height;
        cloud_->width = color_info->width;
        cloud_->is_dense = false;
        cloud_->points.resize(cloud_->height * cloud_->width);
        is_first = false;
    }

    readRgbImage(color_img, color_);
    readDepthImage(depth_img, depth_);

    lock_.lock();
    createCloud(depth_, color_, cloud_);
    is_update_ = true;
    lock_.unlock();

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_, msg);

    msg.header = color_img->header;
    pub_.publish(msg);
}

void Publisher::show()
{
    auto cloudReceiverThread = std::thread(&Publisher::cloudViewer, this); // 获取和生成点云
    cloudReceiverThread.detach();                                          // 将子线程从主线程里分离,子线程执行完成后会自己释放掉资源
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_pointcloud");
    ros::NodeHandle nh("~");

    bool is_show;
    std::string topicColorInfo, topicColor, topicDepthInfo, topicDepth, topicPointcloud;
    nh.param("is_show", is_show, false);
    nh.param("topicColorInfo", topicColorInfo, std::string("/camera/rgb/camera_info"));
    nh.param("topicColor", topicColor, std::string("/camera/rgb/image_rect_color"));
    nh.param("topicDepthInfo", topicDepthInfo, std::string("/camera/depth/camera_info"));
    nh.param("topicDepth", topicDepth, std::string("/camera/depth/image_align"));
    nh.param("topicPointcloud", topicPointcloud, std::string("/camera/pointcloud"));

    Publisher pub(topicColorInfo, topicColor, topicDepthInfo, topicDepth, topicPointcloud);
    if (is_show)
        pub.show();

    ROS_INFO("pub_pointcloud is started");
    ros::spin();
    return 0;
}