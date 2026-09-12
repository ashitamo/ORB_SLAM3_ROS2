#include "rgbd-slam-node.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <functional>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

using std::placeholders::_1;
namespace
{
cv::Mat g_mask_rgbd;
bool g_use_rgbd_mask = false;
}


RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM): Node("ORB_SLAM3_ROS2"), m_SLAM(pSLAM)
{
    const std::string maskPath = ORB_SLAM3_ROS2_CONFIG_DIR "/mask_left.png";

    g_mask_rgbd =
        cv::imread(
            maskPath,
            cv::IMREAD_GRAYSCALE);

    if (g_mask_rgbd.empty())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "RGB-D mask loading failed: %s. Running without mask.",
            maskPath.c_str());
    }
    else if (g_mask_rgbd.size() != cv::Size(848, 480))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "RGB-D mask size must be 848x480, but received %dx%d",
            g_mask_rgbd.cols,
            g_mask_rgbd.rows);

        g_mask_rgbd.release();
    }
    else
    {
        /*
        * 確保遮罩只有 0 與 255：
        * 0   = 保留影像
        * 255 = 遮掉影像
        */
        cv::threshold(
            g_mask_rgbd,
            g_mask_rgbd,
            127,
            255,
            cv::THRESH_BINARY);

        g_use_rgbd_mask = true;

        RCLCPP_INFO(
            this->get_logger(),
            "RGB-D mask loaded successfully: %s",
            maskPath.c_str());
    }
    auto imageQos =
        rclcpp::QoS(rclcpp::KeepLast(10));

    imageQos.reliable();
    imageQos.durability_volatile();

    subRgb_ =
        this->create_subscription<ImageMsg>(
            "camera/rgb",
            imageQos,
            std::bind(
                &RgbdSlamNode::GrabRGB,
                this,
                _1));

    subDepth_ =
        this->create_subscription<ImageMsg>(
            "camera/depth",
            imageQos,
            std::bind(
                &RgbdSlamNode::GrabDepth,
                this,
                _1));

    syncThread_ =
        new std::thread(
            &RgbdSlamNode::SyncRGBD,
            this);

    RCLCPP_INFO(
        this->get_logger(),
        "RGB-D subscriptions and synchronization thread started");
}

RgbdSlamNode::~RgbdSlamNode()
{
    stopRequested_.store(true);

    if (syncThread_ != nullptr)
    {
        if (syncThread_->joinable())
        {
            syncThread_->join();
        }

        delete syncThread_;
        syncThread_ = nullptr;
    }

    if (m_SLAM != nullptr)
    {
        m_SLAM->Shutdown();

        m_SLAM->SaveKeyFrameTrajectoryTUM(
            "KeyFrameTrajectory.txt");
    }
}

void RgbdSlamNode::GrabRGB(
    const ImageMsg::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    const double stamp =
        Utility::StampToSec(
            msg->header.stamp);

    if (!std::isfinite(stamp) ||
        stamp <= 0.0)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Rejected RGB image with invalid timestamp");
        return;
    }

    ++rgbReceivedCount_;

    std::lock_guard<std::mutex> lock(
        rgbMutex_);

    if (lastRgbStamp_ > 0.0)
    {
        const double dt =
            stamp - lastRgbStamp_;

        if (dt <= 0.0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejected non-monotonic RGB timestamp: "
                "current=%.9f previous=%.9f dt=%.9f",
                stamp,
                lastRgbStamp_,
                dt);

            return;
        }

        if (dt > 0.05)
        {
            std::cerr
                << std::fixed
                << std::setprecision(9)
                << "[RGB INPUT GAP]"
                << " previous=" << lastRgbStamp_
                << " current=" << stamp
                << " dt=" << dt
                << std::endl;
        }
    }

    lastRgbStamp_ = stamp;

    while (rgbBuf_.size() >= 10)
    {
        const double droppedStamp =
            Utility::StampToSec(
                rgbBuf_.front()->header.stamp);

        std::cerr
            << std::fixed
            << std::setprecision(9)
            << "[RGB QUEUE] Overflow, dropping image: "
            << droppedStamp
            << std::endl;

        rgbBuf_.pop();
    }

    rgbBuf_.push(msg);
}

void RgbdSlamNode::GrabDepth(
    const ImageMsg::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    const double stamp =
        Utility::StampToSec(
            msg->header.stamp);

    if (!std::isfinite(stamp) ||
        stamp <= 0.0)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Rejected depth image with invalid timestamp");
        return;
    }

    ++depthReceivedCount_;

    std::lock_guard<std::mutex> lock(
        depthMutex_);

    if (lastDepthStamp_ > 0.0)
    {
        const double dt =
            stamp - lastDepthStamp_;

        if (dt <= 0.0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejected non-monotonic depth timestamp: "
                "current=%.9f previous=%.9f dt=%.9f",
                stamp,
                lastDepthStamp_,
                dt);

            return;
        }

        if (dt > 0.05)
        {
            std::cerr
                << std::fixed
                << std::setprecision(9)
                << "[DEPTH INPUT GAP]"
                << " previous=" << lastDepthStamp_
                << " current=" << stamp
                << " dt=" << dt
                << std::endl;
        }
    }

    lastDepthStamp_ = stamp;

    while (depthBuf_.size() >= 10)
    {
        const double droppedStamp =
            Utility::StampToSec(
                depthBuf_.front()->header.stamp);

        std::cerr
            << std::fixed
            << std::setprecision(9)
            << "[DEPTH QUEUE] Overflow, dropping image: "
            << droppedStamp
            << std::endl;

        depthBuf_.pop();
    }

    depthBuf_.push(msg);
}

cv::Mat RgbdSlamNode::GetIntensityImage(
    const ImageMsg::SharedPtr& msg)
{
    if (!msg)
    {
        return cv::Mat();
    }

    try
    {
        const cv_bridge::CvImageConstPtr cvPtr =
            cv_bridge::toCvShare(
                msg,
                sensor_msgs::image_encodings::MONO8);

        if (!cvPtr ||
            cvPtr->image.empty())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "cv_bridge returned empty intensity image");

            return cv::Mat();
        }

        if (cvPtr->image.type() != CV_8UC1)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unexpected intensity image type: %d",
                cvPtr->image.type());

            return cv::Mat();
        }

        /*
         * clone() 避免 ROS message callback 結束後，
         * cv::Mat 還引用原 message 記憶體。
         */
        return cvPtr->image.clone();
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Intensity cv_bridge exception: %s",
            e.what());

        return cv::Mat();
    }
}

cv::Mat RgbdSlamNode::GetDepthImage(
    const ImageMsg::SharedPtr& msg)
{
    if (!msg)
    {
        return cv::Mat();
    }

    try
    {
        const cv_bridge::CvImageConstPtr cvPtr =
            cv_bridge::toCvShare(
                msg,
                sensor_msgs::image_encodings::TYPE_16UC1);

        if (!cvPtr ||
            cvPtr->image.empty())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "cv_bridge returned empty depth image");

            return cv::Mat();
        }

        if (cvPtr->image.type() != CV_16UC1)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unexpected depth image type: %d, encoding=%s",
                cvPtr->image.type(),
                msg->encoding.c_str());

            return cv::Mat();
        }

        return cvPtr->image.clone();
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Depth cv_bridge exception: %s",
            e.what());

        return cv::Mat();
    }
}

void RgbdSlamNode::SyncRGBD()
{
    /*
     * infra1 與 depth 是同一個 RealSense depth geometry，
     * camera_info 相同，depth_to_infra1 為 identity。
     *
     * 先容許 5 ms。
     * 正常情況理想上應接近 0。
     */
    constexpr double maxRgbdTimeDiff = 0.005;

    while (rclcpp::ok() &&
           !stopRequested_.load())
    {
        ImageMsg::SharedPtr rgbMsg;
        ImageMsg::SharedPtr depthMsg;

        double rgbStamp = 0.0;
        double depthStamp = 0.0;

        /*
         * 第一階段：
         * 檢查兩個 queue front 是否屬於同一個 frame。
         * 尚未確認前不要同時 pop。
         */
        {
            std::scoped_lock lock(
                rgbMutex_,
                depthMutex_);

            if (!rgbBuf_.empty() &&
                !depthBuf_.empty())
            {
                const auto candidateRgb =
                    rgbBuf_.front();

                const auto candidateDepth =
                    depthBuf_.front();

                rgbStamp =
                    Utility::StampToSec(
                        candidateRgb->header.stamp);

                depthStamp =
                    Utility::StampToSec(
                        candidateDepth->header.stamp);

                const double syncDiff =
                    std::abs(
                        rgbStamp -
                        depthStamp);

                if (syncDiff >
                    maxRgbdTimeDiff)
                {
                    /*
                     * 較舊的一筆永遠不可能再和較新的 front 配對，
                     * 因此安全丟棄較舊訊息。
                     */
                    if (rgbStamp < depthStamp)
                    {
                        rgbBuf_.pop();
                    }
                    else
                    {
                        depthBuf_.pop();
                    }

                    std::cerr
                        << std::fixed
                        << std::setprecision(9)
                        << "[RGBD RESYNC]"
                        << " rgb=" << rgbStamp
                        << " depth=" << depthStamp
                        << " abs_diff=" << syncDiff
                        << " rgb_received="
                        << rgbReceivedCount_
                        << " depth_received="
                        << depthReceivedCount_
                        << std::endl;
                }
                else
                {
                    rgbMsg =
                        candidateRgb;

                    depthMsg =
                        candidateDepth;
                }
            }
        }

        if (!rgbMsg ||
            !depthMsg)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));

            continue;
        }

        /*
         * 第二階段：
         * 重新檢查 queue front，再正式 commit pop。
         */
        {
            std::scoped_lock lock(
                rgbMutex_,
                depthMutex_);

            if (rgbBuf_.empty() ||
                depthBuf_.empty())
            {
                continue;
            }

            if (rgbBuf_.front() != rgbMsg ||
                depthBuf_.front() != depthMsg)
            {
                continue;
            }

            rgbBuf_.pop();
            depthBuf_.pop();
        }

        cv::Mat intensity =
            GetIntensityImage(
                rgbMsg);

        cv::Mat depth =
            GetDepthImage(
                depthMsg);


        if (intensity.empty() ||
            depth.empty())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Empty RGB-D input after conversion: "
                "intensity_empty=%d depth_empty=%d",
                static_cast<int>(
                    intensity.empty()),
                static_cast<int>(
                    depth.empty()));

            continue;
        }

        if (intensity.size() !=
            depth.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "RGB-D image size mismatch: "
                "intensity=%dx%d depth=%dx%d",
                intensity.cols,
                intensity.rows,
                depth.cols,
                depth.rows);

            continue;
        }

        if (g_use_rgbd_mask)
        {
            if (g_mask_rgbd.size() != intensity.size())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "RGB-D mask/image size mismatch: "
                    "mask=%dx%d image=%dx%d",
                    g_mask_rgbd.cols,
                    g_mask_rgbd.rows,
                    intensity.cols,
                    intensity.rows);

                continue;
            }

            /*
            * 白色區域遮掉：
            * intensity 設為黑色，避免 ORB 擷取機體特徵。
            */
            intensity.setTo(
                0,
                g_mask_rgbd);

            /*
            * 同一區域的 depth 設為 0，表示無有效深度。
            */
            depth.setTo(
                0,
                g_mask_rgbd);
        }

        const double syncDiff =
            std::abs(
                rgbStamp -
                depthStamp);

        double imageDt = 0.0;

        if (lastTrackedStamp_ > 0.0)
        {
            imageDt =
                rgbStamp -
                lastTrackedStamp_;
        }

        static std::uint64_t debugCounter = 0;

        if ((debugCounter++ % 30) == 0)
        {
            std::cout
                << std::fixed
                << std::setprecision(9)
                << "[RGBD SYNC]"
                << " rgb=" << rgbStamp
                << " depth=" << depthStamp
                << " abs_diff=" << syncDiff
                << " image_dt=" << imageDt
                << " intensity="
                << intensity.cols
                << "x"
                << intensity.rows
                << " depth="
                << depth.cols
                << "x"
                << depth.rows
                << " depth_type="
                << depth.type()
                << std::endl;
        }

        /*
         * ORB-SLAM3 會依 YAML 中：
         *
         * RGBD.DepthMapFactor: 1000.0
         *
         * 將 16UC1 毫米深度轉為公尺。
         */
        m_SLAM->TrackRGBD(
            intensity,
            depth,
            rgbStamp);

        lastTrackedStamp_ =
            rgbStamp;

        ++trackedCount_;

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }
}
