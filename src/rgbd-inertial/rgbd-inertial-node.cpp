#include "rgbd-inertial-node.hpp"
#include "Tracking.h"

#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <thread>

using std::placeholders::_1;

namespace
{

constexpr std::size_t kImageQueueLimit = 10;
constexpr std::size_t kImuQueueLimit = 2000;
constexpr unsigned char kPngSignature[] = {
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};

const char* kMaskPath = ORB_SLAM3_ROS2_CONFIG_DIR "/mask_left.png";
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

    publishPose_ =
        this->declare_parameter<bool>(
            "publish_pose",
            true);

    publishOdometry_ =
        this->declare_parameter<bool>(
            "publish_odometry",
            true);
    
    publishTf_ =
        this->declare_parameter<bool>(
            "publish_tf",
            true);

    mapFrameId_ =
        this->declare_parameter<std::string>(
            "map_frame_id",
            "map");

    cameraFrameId_ =
        this->declare_parameter<std::string>(
            "camera_frame_id",
            "camera_infra1_optical_frame");

    if (publishPose_)
    {
        posePublisher_ =
            this->create_publisher<PoseStampedMsg>(
                "/orbslam3/pose",
                rclcpp::QoS(
                    rclcpp::KeepLast(10))
                    .reliable()
                    .durability_volatile());

        RCLCPP_INFO(
            this->get_logger(),
            "Pose publisher enabled: topic=/orbslam3/pose frame=%s child=%s",
            mapFrameId_.c_str(),
            cameraFrameId_.c_str());
    }
    if (publishOdometry_)
    {
        odometryPublisher_ =
            this->create_publisher<OdometryMsg>(
                "/orbslam3/map_odometry",
                rclcpp::QoS(
                    rclcpp::KeepLast(10))
                    .reliable()
                    .durability_volatile());

        RCLCPP_INFO(
            this->get_logger(),
            "Odometry publisher enabled: "
            "topic=/orbslam3/map_odometry "
            "frame=%s child=%s",
            mapFrameId_.c_str(),
            cameraFrameId_.c_str());
    }
    if (publishTf_)
    {
        transformBroadcaster_ =
            std::make_unique<
                tf2_ros::TransformBroadcaster>(
                    *this);

        RCLCPP_INFO(
            this->get_logger(),
            "TF broadcaster enabled: %s -> %s",
            mapFrameId_.c_str(),
            cameraFrameId_.c_str());
    }

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

    const std::string resolvedDepthTopic =
        this->get_node_topics_interface()
            ->resolve_topic_name("camera/depth");
    bool useCompressedDepth =
        resolvedDepthTopic.size() >= std::string("/compressedDepth").size() &&
        resolvedDepthTopic.compare(
            resolvedDepthTopic.size() - std::string("/compressedDepth").size(),
            std::string("/compressedDepth").size(),
            "/compressedDepth") == 0;
    if (!useCompressedDepth)
    {
        for (const auto& endpoint :
             this->get_publishers_info_by_topic(resolvedDepthTopic))
        {
            if (endpoint.topic_type() == "sensor_msgs/msg/CompressedImage")
            {
                useCompressedDepth = true;
                break;
            }
        }
    }

    if (useCompressedDepth)
    {
        subCompressedDepth_ =
            this->create_subscription<CompressedImageMsg>(
                "camera/depth",
                imageQos,
                std::bind(
                    &RgbdInertialNode::GrabCompressedDepth,
                    this,
                    _1));
        RCLCPP_INFO(
            this->get_logger(),
            "Depth input auto-detected: sensor_msgs/CompressedImage on %s",
            resolvedDepthTopic.c_str());
    }
    else
    {
        subDepth_ =
            this->create_subscription<ImageMsg>(
                "camera/depth",
                imageQos,
                std::bind(
                    &RgbdInertialNode::GrabDepth,
                    this,
                    _1));
        RCLCPP_INFO(
            this->get_logger(),
            "Depth input auto-detected: sensor_msgs/Image on %s",
            resolvedDepthTopic.c_str());
    }

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

void RgbdInertialNode::PublishPose(
    const Sophus::SE3f& Tcw,
    const builtin_interfaces::msg::Time& imageStamp)
{
    if (!publishPose_ ||
        !posePublisher_)
    {
        return;
    }

    /*
     * ORB-SLAM3 回傳的是 Tcw：
     * world/map -> camera
     *
     * ROS PoseStamped 需要 camera pose in map，
     * 因此取反得到 Twc。
     */
    const Sophus::SE3f Twc =
        Tcw.inverse();

    const Eigen::Vector3f translation =
        Twc.translation();

    Eigen::Quaternionf quaternion =
        Twc.unit_quaternion();

    quaternion.normalize();

    if (!translation.allFinite() ||
        !std::isfinite(quaternion.x()) ||
        !std::isfinite(quaternion.y()) ||
        !std::isfinite(quaternion.z()) ||
        !std::isfinite(quaternion.w()))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Rejected non-finite SLAM pose");

        return;
    }

    PoseStampedMsg poseMsg;

    poseMsg.header.stamp =
        imageStamp;

    poseMsg.header.frame_id =
        mapFrameId_;

    poseMsg.pose.position.x =
        static_cast<double>(
            translation.x());

    poseMsg.pose.position.y =
        static_cast<double>(
            translation.y());

    poseMsg.pose.position.z =
        static_cast<double>(
            translation.z());

    poseMsg.pose.orientation.x =
        static_cast<double>(
            quaternion.x());

    poseMsg.pose.orientation.y =
        static_cast<double>(
            quaternion.y());

    poseMsg.pose.orientation.z =
        static_cast<double>(
            quaternion.z());

    poseMsg.pose.orientation.w =
        static_cast<double>(
            quaternion.w());

    posePublisher_->publish(
        poseMsg);
}

void RgbdInertialNode::PublishOdometry(
    const Sophus::SE3f& Tcw,
    const builtin_interfaces::msg::Time& imageStamp)
{
    if (!publishOdometry_ ||
        !odometryPublisher_)
    {
        return;
    }

    /*
     * ORB-SLAM3 回傳 Tcw：
     * map/world -> camera
     *
     * Odometry 要表達 camera 在 map 中的 pose，
     * 因此使用 Twc。
     */
    const Sophus::SE3f Twc =
        Tcw.inverse();

    const Eigen::Vector3f translation =
        Twc.translation();

    Eigen::Quaternionf quaternion =
        Twc.unit_quaternion();

    quaternion.normalize();

    if (!translation.allFinite() ||
        !std::isfinite(quaternion.x()) ||
        !std::isfinite(quaternion.y()) ||
        !std::isfinite(quaternion.z()) ||
        !std::isfinite(quaternion.w()))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Rejected non-finite SLAM odometry");

        return;
    }

    OdometryMsg odometryMsg;

    /*
     * 與 PoseStamped 使用完全相同的
     * infra1 image timestamp。
     */
    odometryMsg.header.stamp =
        imageStamp;

    odometryMsg.header.frame_id =
        mapFrameId_;

    odometryMsg.child_frame_id =
        cameraFrameId_;

    odometryMsg.pose.pose.position.x =
        static_cast<double>(
            translation.x());

    odometryMsg.pose.pose.position.y =
        static_cast<double>(
            translation.y());

    odometryMsg.pose.pose.position.z =
        static_cast<double>(
            translation.z());

    odometryMsg.pose.pose.orientation.x =
        static_cast<double>(
            quaternion.x());

    odometryMsg.pose.pose.orientation.y =
        static_cast<double>(
            quaternion.y());

    odometryMsg.pose.pose.orientation.z =
        static_cast<double>(
            quaternion.z());

    odometryMsg.pose.pose.orientation.w =
        static_cast<double>(
            quaternion.w());

    /*
     * 第一版尚未計算線速度、角速度與 covariance。
     * ROS message 預設值為 0，因此先不填。
     *
     * 注意：0 covariance 在語意上不代表真正已知為零，
     * 之後接 robot_localization 前必須重新設計。
     */

    odometryPublisher_->publish(
        odometryMsg);
}

void RgbdInertialNode::PublishTransform(
    const Sophus::SE3f& Tcw,
    const builtin_interfaces::msg::Time& imageStamp)
{
    if (!publishTf_ ||
        !transformBroadcaster_)
    {
        return;
    }

    /*
     * ORB-SLAM3 回傳 Tcw：
     * map/world -> camera
     *
     * TF map -> camera 要使用
     * camera 在 map 中的姿態 Twc。
     */
    const Sophus::SE3f Twc =
        Tcw.inverse();

    const Eigen::Vector3f translation =
        Twc.translation();

    Eigen::Quaternionf quaternion =
        Twc.unit_quaternion();

    quaternion.normalize();

    if (!translation.allFinite() ||
        !std::isfinite(quaternion.x()) ||
        !std::isfinite(quaternion.y()) ||
        !std::isfinite(quaternion.z()) ||
        !std::isfinite(quaternion.w()))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Rejected non-finite SLAM transform");

        return;
    }

    TransformStampedMsg transformMsg;

    transformMsg.header.stamp =
        imageStamp;

    transformMsg.header.frame_id =
        mapFrameId_;

    transformMsg.child_frame_id =
        cameraFrameId_;

    transformMsg.transform.translation.x =
        static_cast<double>(
            translation.x());

    transformMsg.transform.translation.y =
        static_cast<double>(
            translation.y());

    transformMsg.transform.translation.z =
        static_cast<double>(
            translation.z());

    transformMsg.transform.rotation.x =
        static_cast<double>(
            quaternion.x());

    transformMsg.transform.rotation.y =
        static_cast<double>(
            quaternion.y());

    transformMsg.transform.rotation.z =
        static_cast<double>(
            quaternion.z());

    transformMsg.transform.rotation.w =
        static_cast<double>(
            quaternion.w());

    transformBroadcaster_->sendTransform(
        transformMsg);
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

void RgbdInertialNode::GrabCompressedDepth(
    const CompressedImageMsg::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    if (msg->format.find("compressedDepth") == std::string::npos ||
        msg->format.find("rvl") != std::string::npos ||
        msg->format.rfind("32FC1", 0) == 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Unsupported compressed depth format: %s; expected 16UC1 PNG compressedDepth",
            msg->format.c_str());
        return;
    }

    const auto pngBegin =
        std::search(
            msg->data.begin(),
            msg->data.end(),
            std::begin(kPngSignature),
            std::end(kPngSignature));
    if (pngBegin == msg->data.end())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "compressedDepth message has no PNG payload");
        return;
    }

    const std::vector<unsigned char> pngData(
        pngBegin,
        msg->data.end());
    const cv::Mat depth =
        cv::imdecode(
            pngData,
            cv::IMREAD_UNCHANGED);
    if (depth.empty() || depth.type() != CV_16UC1)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to decode 16UC1 compressedDepth PNG");
        return;
    }

    auto rawMessage =
        cv_bridge::CvImage(
            msg->header,
            sensor_msgs::image_encodings::TYPE_16UC1,
            depth).toImageMsg();
    GrabDepth(rawMessage);
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
        const Sophus::SE3f Tcw =
            SLAM_->TrackRGBD(
                intensity,
                depth,
                rgbStamp,
                vImuMeas);

        const int trackingState =
            SLAM_->GetTrackingState();

        if (trackingState ==
            ORB_SLAM3::Tracking::OK)
        {
            hasTracked_ = true;
            PublishPose(
                Tcw,
                rgbMsg->header.stamp);

            PublishTransform(
                Tcw,
                rgbMsg->header.stamp);
        }

        lastTrackedImageStamp_ =
            rgbStamp;

        ++trackedCount_;

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }
}
