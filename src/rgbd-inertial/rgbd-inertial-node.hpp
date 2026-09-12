#ifndef RGBD_INERTIAL_NODE_HPP_
#define RGBD_INERTIAL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

#include "cv_bridge/cv_bridge.h"

#include "System.h"
#include "utility.hpp"

#include <opencv2/core/core.hpp>


#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using OdometryMsg = nav_msgs::msg::Odometry;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;


class RgbdInertialNode : public rclcpp::Node
{
public:
    explicit RgbdInertialNode(
        ORB_SLAM3::System* pSLAM);

    ~RgbdInertialNode() override;

    void Stop();

private:
    /*
     * ROS callbacks
     */
    void GrabRGB(
        const ImageMsg::SharedPtr msg);

    void GrabDepth(
        const ImageMsg::SharedPtr msg);

    void GrabImu(
        const ImuMsg::SharedPtr msg);

    void PublishPose(
        const Sophus::SE3f& Tcw,
        const builtin_interfaces::msg::Time& imageStamp);

    void PublishOdometry(
        const Sophus::SE3f& Tcw,
        const builtin_interfaces::msg::Time& imageStamp);

    void PublishTransform(
        const Sophus::SE3f& Tcw,
        const builtin_interfaces::msg::Time& imageStamp);

    /*
     * RGB-D-IMU 同步執行緒
     */
    void SyncRGBDWithImu();

    /*
     * ROS image → cv::Mat
     */
    cv::Mat GetIntensityImage(
        const ImageMsg::SharedPtr& msg);

    cv::Mat GetDepthImage(
        const ImageMsg::SharedPtr& msg);

    /*
     * IMU 資料檢查與轉換
     */
    bool IsFiniteImuMsg(
        const ImuMsg::SharedPtr& msg) const;

    ORB_SLAM3::IMU::Point ConvertImuMsg(
        const ImuMsg::SharedPtr& msg) const;

private:
    ORB_SLAM3::System* SLAM_ = nullptr;

    double imu_time_offset_sec_ = 0.0;
    /*
     * ROS subscriptions
     */
    rclcpp::Subscription<ImageMsg>::SharedPtr subRgb_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subDepth_;
    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;

    rclcpp::Publisher<PoseStampedMsg>::SharedPtr posePublisher_;
    rclcpp::Publisher<OdometryMsg>::SharedPtr odometryPublisher_;

    std::unique_ptr<tf2_ros::TransformBroadcaster>transformBroadcaster_;

    /*
     * Input queues
     */
    std::queue<ImageMsg::SharedPtr> rgbBuf_;
    std::queue<ImageMsg::SharedPtr> depthBuf_;
    std::queue<ImuMsg::SharedPtr> imuBuf_;
    

    /*
     * Queue mutexes
     */
    std::mutex rgbMutex_;
    std::mutex depthMutex_;
    std::mutex imuMutex_;

    /*
     * Synchronization worker
     */
    std::thread syncThread_;
    std::atomic<bool> stopRequested_{false};

    /*
     * Input timestamp diagnostics
     */
    double lastRgbStamp_ = -1.0;
    double lastDepthStamp_ = -1.0;
    double lastImuStamp_ = -1.0;
    double lastTrackedImageStamp_ = -1.0;

    /*
     * Counters
     */
    std::uint64_t rgbReceivedCount_ = 0;
    std::uint64_t depthReceivedCount_ = 0;
    std::uint64_t imuReceivedCount_ = 0;
    std::uint64_t trackedCount_ = 0;

    /*
     * RGB-D mask
     *
     * 0   = 保留
     * 255 = 遮掉
     */
    cv::Mat maskRgbd_;
    bool useMask_ = false;

    bool publishPose_ = true;
    bool publishOdometry_ = true;
    bool publishTf_ = true;

    std::string mapFrameId_ = "map";
    std::string cameraFrameId_ = "camera_infra1_optical_frame";
};

#endif  // RGBD_INERTIAL_NODE_HPP_
