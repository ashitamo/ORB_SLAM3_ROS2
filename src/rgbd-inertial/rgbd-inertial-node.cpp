#include "rgbd-inertial-node.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>

using std::placeholders::_1;

namespace
{

constexpr std::size_t kImageQueueLimit = 10;
constexpr std::size_t kImuQueueLimit = 2000;

const char* kMaskPath =
    "/home/lab606/orbslam3_ros2_ws/"
    "src/ORB_SLAM3_ROS2/config/mask_left.png";

}  // namespace


RgbdInertialNode::RgbdInertialNode(
    ORB_SLAM3::System* pSLAM)
: Node("ORB_SLAM3_ROS2"),
  SLAM_(pSLAM)
{
    imu_time_offset_sec_ =
        this->declare_parameter<double>(
            "imu_time_offset_sec",
            0.0);

    RCLCPP_INFO(
        this->get_logger(),
        "IMU timestamp offset: %.6f seconds",
        imu_time_offset_sec_);

    if (SLAM_ == nullptr)
    {
        throw std::invalid_argument(
            "RgbdInertialNode received a null ORB-SLAM3 System pointer");
    }

    /*
     * 載入 infra1 對應的左眼 mask。
     *
     * 0   = 保留
     * 255 = 遮掉
     */
    maskRgbd_ =
        cv::imread(
            kMaskPath,
            cv::IMREAD_GRAYSCALE);

    if (maskRgbd_.empty())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "RGB-D-Inertial mask loading failed: %s. "
            "Running without mask.",
            kMaskPath);
    }
    else if (maskRgbd_.size() != cv::Size(848, 480))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "RGB-D-Inertial mask must be 848x480, "
            "but received %dx%d",
            maskRgbd_.cols,
            maskRgbd_.rows);

        maskRgbd_.release();
    }
    else
    {
        cv::threshold(
            maskRgbd_,
            maskRgbd_,
            127,
            255,
            cv::THRESH_BINARY);

        useMask_ = true;

        const int maskedPixels =
            cv::countNonZero(maskRgbd_);

        const int totalPixels =
            maskRgbd_.rows *
            maskRgbd_.cols;

        const double maskedRatio =
            static_cast<double>(maskedPixels) /
            static_cast<double>(totalPixels);

        RCLCPP_INFO(
            this->get_logger(),
            "RGB-D-Inertial mask loaded: %s, "
            "masked=%d/%d (%.2f%%)",
            kMaskPath,
            maskedPixels,
            totalPixels,
            maskedRatio * 100.0);
    }

    /*
     * RealSense image publisher 為 RELIABLE。
     */
    auto imageQos =
        rclcpp::QoS(
            rclcpp::KeepLast(10));

    imageQos.reliable();
    imageQos.durability_volatile();

    subRgb_ =
        this->create_subscription<ImageMsg>(
            "camera/rgb",
            imageQos,
            std::bind(
                &RgbdInertialNode::GrabRGB,
                this,
                _1));

    subDepth_ =
        this->create_subscription<ImageMsg>(
            "camera/depth",
            imageQos,
            std::bind(
                &RgbdInertialNode::GrabDepth,
                this,
                _1));

    /*
     * IMU 使用 SensorDataQoS：
     * BEST_EFFORT、VOLATILE、小 queue。
     */
    subImu_ =
        this->create_subscription<ImuMsg>(
            "/imu/data",
            rclcpp::SensorDataQoS(),
            std::bind(
                &RgbdInertialNode::GrabImu,
                this,
                _1));

    syncThread_ =
        std::thread(
            &RgbdInertialNode::SyncRGBDWithImu,
            this);

    RCLCPP_INFO(
        this->get_logger(),
        "RGB-D-Inertial subscriptions and "
        "synchronization thread started");
}

void RgbdInertialNode::Stop()
{
    stopRequested_.store(true);

    if (syncThread_.joinable())
    {
        syncThread_.join();
    }
}

RgbdInertialNode::~RgbdInertialNode()
{
    Stop();
}

bool RgbdInertialNode::IsFiniteImuMsg(
    const ImuMsg::SharedPtr& msg) const
{
    if (!msg)
    {
        return false;
    }

    const double ax =
        msg->linear_acceleration.x;

    const double ay =
        msg->linear_acceleration.y;

    const double az =
        msg->linear_acceleration.z;

    const double gx =
        msg->angular_velocity.x;

    const double gy =
        msg->angular_velocity.y;

    const double gz =
        msg->angular_velocity.z;

    if (!std::isfinite(ax) ||
        !std::isfinite(ay) ||
        !std::isfinite(az) ||
        !std::isfinite(gx) ||
        !std::isfinite(gy) ||
        !std::isfinite(gz))
    {
        return false;
    }

    const double accelerationNorm =
        std::sqrt(
            ax * ax +
            ay * ay +
            az * az);

    const double angularVelocityNorm =
        std::sqrt(
            gx * gx +
            gy * gy +
            gz * gz);

    /*
     * 這不是感測器實際量測範圍，
     * 只是排除明顯損壞的資料。
     */
    if (accelerationNorm > 100.0 ||
        angularVelocityNorm > 20.0)
    {
        return false;
    }

    return true;
}

ORB_SLAM3::IMU::Point
RgbdInertialNode::ConvertImuMsg(
    const ImuMsg::SharedPtr& msg) const
{
    const double rawTimestamp =
        Utility::StampToSec(
            msg->header.stamp);

    const double adjustedTimestamp =
        rawTimestamp + imu_time_offset_sec_;

    const cv::Point3f acceleration(
        static_cast<float>(
            msg->linear_acceleration.x),
        static_cast<float>(
            msg->linear_acceleration.y),
        static_cast<float>(
            msg->linear_acceleration.z));

    const cv::Point3f angularVelocity(
        static_cast<float>(
            msg->angular_velocity.x),
        static_cast<float>(
            msg->angular_velocity.y),
        static_cast<float>(
            msg->angular_velocity.z));

    return ORB_SLAM3::IMU::Point(
        acceleration,
        angularVelocity,
        adjustedTimestamp);
}

void RgbdInertialNode::GrabRGB(
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

    while (rgbBuf_.size() >=
           kImageQueueLimit)
    {
        const double droppedStamp =
            Utility::StampToSec(
                rgbBuf_.front()
                    ->header.stamp);

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

void RgbdInertialNode::GrabDepth(
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
                << " previous="
                << lastDepthStamp_
                << " current=" << stamp
                << " dt=" << dt
                << std::endl;
        }
    }

    lastDepthStamp_ = stamp;

    while (depthBuf_.size() >=
           kImageQueueLimit)
    {
        const double droppedStamp =
            Utility::StampToSec(
                depthBuf_.front()
                    ->header.stamp);

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

void RgbdInertialNode::GrabImu(
    const ImuMsg::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    if (!IsFiniteImuMsg(msg))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Rejected invalid IMU: "
            "acc=[%.9f %.9f %.9f], "
            "gyro=[%.9f %.9f %.9f]",
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z,
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

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
            "Rejected IMU with invalid timestamp");
        return;
    }

    ++imuReceivedCount_;

    std::lock_guard<std::mutex> lock(
        imuMutex_);

    if (lastImuStamp_ > 0.0)
    {
        const double dt =
            stamp - lastImuStamp_;

        if (dt <= 0.0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejected non-monotonic IMU timestamp: "
                "current=%.9f previous=%.9f dt=%.9f",
                stamp,
                lastImuStamp_,
                dt);

            return;
        }

        if (dt > 0.05)
        {
            std::cerr
                << std::fixed
                << std::setprecision(9)
                << "[IMU INPUT GAP]"
                << " previous=" << lastImuStamp_
                << " current=" << stamp
                << " dt=" << dt
                << std::endl;
        }
    }

    lastImuStamp_ = stamp;

    while (imuBuf_.size() >=
           kImuQueueLimit)
    {
        const double droppedStamp =
            Utility::StampToSec(
                imuBuf_.front()
                    ->header.stamp);

        std::cerr
            << std::fixed
            << std::setprecision(9)
            << "[IMU QUEUE] Overflow, dropping IMU: "
            << droppedStamp
            << std::endl;

        imuBuf_.pop();
    }

    imuBuf_.push(msg);
}

cv::Mat RgbdInertialNode::GetIntensityImage(
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
                "Unexpected intensity type=%d encoding=%s",
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
            "Intensity cv_bridge exception: %s",
            e.what());

        return cv::Mat();
    }
}

cv::Mat RgbdInertialNode::GetDepthImage(
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
                "Unexpected depth type=%d encoding=%s",
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

void RgbdInertialNode::SyncRGBDWithImu()
{
    constexpr double maxRgbdTimeDiff = 0.005;
    constexpr double maxImuInterval = 0.1;
    constexpr std::size_t minImuMeasurements = 2;

    while (rclcpp::ok() &&
           !stopRequested_.load())
    {
        ImageMsg::SharedPtr rgbMsg;
        ImageMsg::SharedPtr depthMsg;

        double rgbStamp = 0.0;
        double depthStamp = 0.0;

        /*
         * 第一階段：配對 RGB 與 depth。
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

                const double rgbdDiff =
                    std::abs(
                        rgbStamp -
                        depthStamp);

                if (rgbdDiff >
                    maxRgbdTimeDiff)
                {
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
                        << "[RGBD PAIR REJECTED]"
                        << " rgb=" << rgbStamp
                        << " depth=" << depthStamp
                        << " abs_diff=" << rgbdDiff
                        << " action="
                        << (rgbStamp < depthStamp
                                ? "drop_rgb"
                                : "drop_depth")
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
         * 第二階段：確認 IMU 已經跨過影像時間。
         *
         * 最新 IMU 必須 >= image timestamp，否則繼續等待。
         */
        bool imuHasReachedImage = false;
        bool imageOlderThanAvailableImu = false;

        {
            std::lock_guard<std::mutex> lock(
                imuMutex_);

            if (!imuBuf_.empty())
            {
                const double oldestImuStamp =
                    Utility::StampToSec(
                        imuBuf_.front()
                            ->header.stamp)
                    + imu_time_offset_sec_;

                const double latestImuStamp =
                    Utility::StampToSec(
                        imuBuf_.back()
                            ->header.stamp)
                    + imu_time_offset_sec_;

                imuHasReachedImage =
                    latestImuStamp >= rgbStamp;

                /*
                 * 第一張影像尚無上一區間 IMU。
                 * 如果影像比 queue 最舊 IMU 更早，
                 * 已不可能補齊前段資料。
                 */
                imageOlderThanAvailableImu =
                    lastTrackedImageStamp_ < 0.0 &&
                    oldestImuStamp > rgbStamp;
            }
        }

        if (imageOlderThanAvailableImu)
        {
            std::cerr
                << std::fixed
                << std::setprecision(9)
                << "[RGBD-IMU SYNC] Dropping image older "
                << "than oldest available IMU: image="
                << rgbStamp
                << std::endl;

            std::scoped_lock lock(
                rgbMutex_,
                depthMutex_);

            if (!rgbBuf_.empty() &&
                rgbBuf_.front() == rgbMsg)
            {
                rgbBuf_.pop();
            }

            if (!depthBuf_.empty() &&
                depthBuf_.front() == depthMsg)
            {
                depthBuf_.pop();
            }

            continue;
        }

        if (!imuHasReachedImage)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));

            continue;
        }

        /*
         * 第三階段：先用 IMU queue 的複本建立 vector。
         *
         * 此時不正式 pop，必須等完整驗證通過。
         */
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        std::vector<ImuMsg::SharedPtr> imuMsgsToConsume;

        {
            std::lock_guard<std::mutex> lock(
                imuMutex_);

            auto temporaryImuBuffer =
                imuBuf_;

            while (!temporaryImuBuffer.empty())
            {
                const auto imuMsg =
                    temporaryImuBuffer.front();

                const double imuStamp =
                    Utility::StampToSec(
                        imuMsg->header.stamp)
                    + imu_time_offset_sec_;

                if (imuStamp > rgbStamp)
                {
                    break;
                }

                temporaryImuBuffer.pop();

                if (!IsFiniteImuMsg(imuMsg))
                {
                    continue;
                }

                imuMsgsToConsume.push_back(
                    imuMsg);

                /*
                 * 避免 lastConsumedImu_ 和 queue 中資料
                 * 出現相同 timestamp。
                 */
                if (vImuMeas.empty() ||
                    std::abs(
                        vImuMeas.back().t -
                        imuStamp) > 1e-9)
                {
                    vImuMeas.push_back(
                        ConvertImuMsg(
                            imuMsg));
                }
            }
        }

        /*
         * IMU 已經跨過影像時間，但這張影像仍無法取得
         * 至少兩筆量測，代表它已不可能再補齊。
         */
        if (vImuMeas.size() <
            minImuMeasurements)
        {
            std::cerr
                << std::fixed
                << std::setprecision(9)
                << "[RGBD-IMU SYNC] Dropping image with "
                << "insufficient IMU measurements: image="
                << rgbStamp
                << " imu_count="
                << vImuMeas.size()
                << std::endl;

            std::scoped_lock lock(
                rgbMutex_,
                depthMutex_);

            if (!rgbBuf_.empty() &&
                rgbBuf_.front() == rgbMsg)
            {
                rgbBuf_.pop();
            }

            if (!depthBuf_.empty() &&
                depthBuf_.front() == depthMsg)
            {
                depthBuf_.pop();
            }

            continue;
        }

        /*
         * 第四階段：檢查 IMU timestamp。
         */
        bool validImuVector = true;

        for (std::size_t i = 1;
             i < vImuMeas.size();
             ++i)
        {
            const double imuDt =
                vImuMeas[i].t -
                vImuMeas[i - 1].t;

            if (!std::isfinite(imuDt) ||
                imuDt <= 0.0 ||
                imuDt > maxImuInterval)
            {
                std::cerr
                    << std::fixed
                    << std::setprecision(9)
                    << "[RGBD-IMU SYNC] Invalid IMU dt:"
                    << " index=" << i
                    << " previous="
                    << vImuMeas[i - 1].t
                    << " current="
                    << vImuMeas[i].t
                    << " dt=" << imuDt
                    << std::endl;

                validImuVector = false;
                break;
            }
        }

        if (!validImuVector)
        {
            /*
             * IMU vector 壞掉時，影像也不能永久卡在 queue front。
             */
            std::scoped_lock lock(
                rgbMutex_,
                depthMutex_);

            if (!rgbBuf_.empty() &&
                rgbBuf_.front() == rgbMsg)
            {
                rgbBuf_.pop();
            }

            if (!depthBuf_.empty() &&
                depthBuf_.front() == depthMsg)
            {
                depthBuf_.pop();
            }

            continue;
        }

        /*
         * 第五階段：正式 commit。
         *
         * 先確認 RGB 與 depth front 尚未被 callback overflow 改變。
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

        /*
         * 正式消耗已經用於這張影像的 IMU。
         */
        {
            std::lock_guard<std::mutex> lock(
                imuMutex_);

            bool imuQueueChanged = false;

            for (const auto& expectedMsg :
                 imuMsgsToConsume)
            {
                if (imuBuf_.empty() ||
                    imuBuf_.front() != expectedMsg)
                {
                    imuQueueChanged = true;
                    break;
                }

                imuBuf_.pop();
            }

            if (imuQueueChanged)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "IMU queue changed unexpectedly during commit");

                continue;
            }
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
                "Empty RGB-D input after conversion");

            continue;
        }

        if (intensity.size() !=
            depth.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "RGB-D size mismatch: "
                "intensity=%dx%d depth=%dx%d",
                intensity.cols,
                intensity.rows,
                depth.cols,
                depth.rows);

            continue;
        }

        if (useMask_)
        {
            if (maskRgbd_.size() !=
                intensity.size())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Mask/image size mismatch: "
                    "mask=%dx%d image=%dx%d",
                    maskRgbd_.cols,
                    maskRgbd_.rows,
                    intensity.cols,
                    intensity.rows);

                continue;
            }

            intensity.setTo(
                0,
                maskRgbd_);

            depth.setTo(
                0,
                maskRgbd_);
        }

        const double rgbdDiff =
            std::abs(
                rgbStamp -
                depthStamp);

        double imageDt = 0.0;

        if (lastTrackedImageStamp_ > 0.0)
        {
            imageDt =
                rgbStamp -
                lastTrackedImageStamp_;
        }

        static std::uint64_t debugCounter = 0;

        if ((debugCounter++ % 30) == 0)
        {
            std::cout
                << std::fixed
                << std::setprecision(9)
                << "[RGBD-IMU SYNC]"
                << " image=" << rgbStamp
                << " image_dt=" << imageDt
                << " rgbd_diff=" << rgbdDiff
                << " imu_offset="
                << imu_time_offset_sec_
                << " imu_count="
                << vImuMeas.size()
                << " imu_first="
                << vImuMeas.front().t
                << " imu_last="
                << vImuMeas.back().t
                << " image_minus_imu_last="
                << rgbStamp -
                vImuMeas.back().t
                << std::endl;
        }

        /*
         * ORB-SLAM3 原生支援：
         *
         * TrackRGBD(image, depth, timestamp, imuVector)
         */
        SLAM_->TrackRGBD(
            intensity,
            depth,
            rgbStamp,
            vImuMeas);

        lastTrackedImageStamp_ =
            rgbStamp;

        ++trackedCount_;

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }
}