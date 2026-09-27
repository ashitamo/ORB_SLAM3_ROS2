#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
cv::Mat g_mask_left;
cv::Mat g_mask_right;
bool g_use_mask = false;
}

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (doRectify){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }
    const std::string mask_left_path = ORB_SLAM3_ROS2_CONFIG_DIR "/mask_left.png";

    const std::string mask_right_path = ORB_SLAM3_ROS2_CONFIG_DIR "/mask_right.png";

    g_mask_left = cv::imread(mask_left_path, cv::IMREAD_GRAYSCALE);
    g_mask_right = cv::imread(mask_right_path, cv::IMREAD_GRAYSCALE);

    if (g_mask_left.empty() || g_mask_right.empty())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Mask loading failed. ORB-SLAM3 will run without masks.");
    }
    else if (g_mask_left.size() != cv::Size(848, 480) ||
            g_mask_right.size() != cv::Size(848, 480))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Mask size must be 848x480. Left=%dx%d Right=%dx%d",
            g_mask_left.cols,
            g_mask_left.rows,
            g_mask_right.cols,
            g_mask_right.rows);

        g_mask_left.release();
        g_mask_right.release();
    }
    else
    {
        // 確保只存在 0 與 255
        cv::threshold(
            g_mask_left,
            g_mask_left,
            127,
            255,
            cv::THRESH_BINARY);

        cv::threshold(
            g_mask_right,
            g_mask_right,
            127,
            255,
            cv::THRESH_BINARY);

        g_use_mask = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Stereo masks loaded successfully.");
    }
    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    image_qos.reliable();
    image_qos.durability_volatile();

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
        this,
        "/camera/camera/infra1/image_rect_raw",
        image_qos.get_rmw_qos_profile());

    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
        this,
        "/camera/camera/infra2/image_rect_raw",
        image_qos.get_rmw_qos_profile());

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat imLeft;
    cv::Mat imRight;

    if (doRectify)
    {
        cv::remap(
            cv_ptrLeft->image,
            imLeft,
            M1l,
            M2l,
            cv::INTER_LINEAR);

        cv::remap(
            cv_ptrRight->image,
            imRight,
            M1r,
            M2r,
            cv::INTER_LINEAR);
    }
    else
    {
        imLeft = cv_ptrLeft->image.clone();
        imRight = cv_ptrRight->image.clone();
    }

    if (g_use_mask)
    {
        // 白色區域遮掉，黑色區域保留
        imLeft.setTo(0, g_mask_left);
        imRight.setTo(0, g_mask_right);
    }

    m_SLAM->TrackStereo(
        imLeft,
        imRight,
        Utility::StampToSec(msgLeft->header.stamp));
}
