#include "alignment.h"

static void doRegister_(const TY_CAMERA_CALIB_INFO &depth_calib, const TY_CAMERA_CALIB_INFO &color_calib, const cv::Mat &depth, cv::Mat &out)
{
    // do register
    cv::Mat depth_process = depth.clone();
    for (int i = 0; i < depth_process.rows; i++)
    {
        for (int j = 0; j < depth_process.cols; j++)
        {
            depth_process.at<uint16_t>(i, j) = depth_process.at<uint16_t>(i, j) / 4;
        }
    }
    out = cv::Mat::zeros(depth_process.size(), CV_16U);
    ASSERT_OK(
        TYMapDepthImageToColorCoordinate(
            &depth_calib,
            depth.cols, depth.rows, depth_process.ptr<uint16_t>(),
            &color_calib,
            out.cols, out.rows, out.ptr<uint16_t>()));

    //you may want to use median filter to fill holes in projected depth image
    //or do something else here
    // cv::Mat temp;
    // cv::medianBlur(out, temp, 5);
    // out = temp;
}

Alignment::Alignment(/* args */)
{
}

Alignment::~Alignment()
{
}

// TODO: 这里的rgb内参，rgb与ir外参，depth内参，畸变参数，需要修改成自己的相机参数
// FIXME: rgb内参，depth内参目前是固定的，效果好一些，应该是自己标定的相机不如官方导致的
// 查看方式可参考test.cpp代码查看
void Alignment::cameraInfoToTyCalibInfo(const sensor_msgs::CameraInfo::ConstPtr &cameraInfo, TY_CAMERA_CALIB_INFO &calib, bool is_color)
{
    static int width = 1280, height = 960;

    calib.intrinsicHeight = cameraInfo->height;
    calib.intrinsicWidth = cameraInfo->width;

    float ratio = height * 1.0 / calib.intrinsicHeight;

    if (is_color)
    {
        // [3*3]
        // 1431.62,0,668.166,
        // 0,1432.07,504.324,
        // 0,0,1,
        float tmp[9] = {1431.62 / ratio, 0, 668.166 / ratio, 0, 1432.07 / ratio, 504.324 / ratio, 0, 0, 1};
        for (int i = 0; i < 3 * 3; i++)
        {
            // calib.intrinsic.data[i] = cameraInfo->K.at(i);
            calib.intrinsic.data[i] = tmp[i];
        }
    }
    else
    {
        // 1116.41,0,639.956,
        // 0,1116.41,519.495,
        // 0,0,1,
        float tmp[9] = {1116.41 / ratio, 0, 639.956 / ratio, 0, 1116.41 / ratio, 519.495 / ratio, 0, 0, 1};
        for (int i = 0; i < 3 * 3; i++)
        {
            // calib.intrinsic.data[i] = cameraInfo->K.at(i);
            calib.intrinsic.data[i] = tmp[i];
        }
    }

    if (is_color)
    {
        // [4*4]
        // 0.99995,-0.00927714,0.00384947,-23.8701,
        // 0.00929822,0.999942,-0.0054949,0.0784604,
        // -0.00379827,0.00553042,0.999977,-2.41233,
        // 0,0,0,1,
        calib.extrinsic.data[0 * 4 + 0] = 0.99995;
        calib.extrinsic.data[1 * 4 + 0] = 0.00929822;
        calib.extrinsic.data[2 * 4 + 0] = -0.00379827;
        calib.extrinsic.data[3 * 4 + 0] = 0;

        calib.extrinsic.data[0 * 4 + 1] = -0.00927714;
        calib.extrinsic.data[1 * 4 + 1] = 0.999942;
        calib.extrinsic.data[2 * 4 + 1] = 0.00553042;
        calib.extrinsic.data[3 * 4 + 1] = 0;

        calib.extrinsic.data[0 * 4 + 2] = 0.00384947;
        calib.extrinsic.data[1 * 4 + 2] = -0.0054949;
        calib.extrinsic.data[2 * 4 + 2] = 0.999977;
        calib.extrinsic.data[3 * 4 + 2] = 0;

        calib.extrinsic.data[0 * 4 + 3] = -23.8701;
        calib.extrinsic.data[1 * 4 + 3] = 0.0784604;
        calib.extrinsic.data[2 * 4 + 3] = -2.41233;
        calib.extrinsic.data[3 * 4 + 3] = 1;

        // [12]
        float tmp[12] = {0.0164927, -0.399465, 2.37799e-05, 0.000710421, -0.166281, 0.404057, -0.401382, -0.387009, 0.00117576, -0.000400179, 0.00082404, -0.000144004};
        for (int i = 0; i < 12; i++)
            calib.distortion.data[i] = tmp[i];
    }
}

void printArray(const float *data, int r, int c)
{
    for (int i = 0; i < r; i++)
    {
        for (int j = 0; j < c; j++)
        {
            std::cout << data[i * c + j] << ",";
        }
        std::cout << std::endl;
    }
}

void printCalib(const TY_CAMERA_CALIB_INFO &calib)
{
    std::cout << "calib.intrinsicWidth:" << calib.intrinsicWidth << std::endl;
    std::cout << "calib.intrinsicHeight:" << calib.intrinsicHeight << std::endl;

    std::cout << "calib.intrinsic" << std::endl;
    printArray(calib.intrinsic.data, 3, 3);

    std::cout << "calib.extrinsic" << std::endl;
    printArray(calib.extrinsic.data, 4, 4);

    std::cout << "calib.distortion" << std::endl;
    printArray(calib.distortion.data, 1, 12);
}

void Alignment::doRegister(const TY_CAMERA_CALIB_INFO &depth_calib, const TY_CAMERA_CALIB_INFO &color_calib, const cv::Mat &depth, cv::Mat &out)
{
    // printCalib(color_calib, )
    doRegister_(depth_calib, color_calib, depth, out);
}