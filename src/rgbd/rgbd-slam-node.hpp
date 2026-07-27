#ifndef RGBD_SLAM_NODE_HPP_
#define RGBD_SLAM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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

using ImageMsg = sensor_msgs::msg::Image;

class RgbdSlamNode : public rclcpp::Node
{
public:
    explicit RgbdSlamNode(ORB_SLAM3::System* pSLAM);
    ~RgbdSlamNode() override;

private:
    void GrabRGB(const ImageMsg::SharedPtr msg);
    void GrabDepth(const ImageMsg::SharedPtr msg);

    void SyncRGBD();

    cv::Mat GetIntensityImage(
        const ImageMsg::SharedPtr& msg);

    cv::Mat GetDepthImage(
        const ImageMsg::SharedPtr& msg);

private:
    ORB_SLAM3::System* m_SLAM = nullptr;



    rclcpp::Subscription<ImageMsg>::SharedPtr subRgb_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subDepth_;

    std::queue<ImageMsg::SharedPtr> rgbBuf_;
    std::queue<ImageMsg::SharedPtr> depthBuf_;

    std::mutex rgbMutex_;
    std::mutex depthMutex_;

    std::thread* syncThread_ = nullptr;
    std::atomic<bool> stopRequested_{false};

    double lastRgbStamp_ = -1.0;
    double lastDepthStamp_ = -1.0;
    double lastTrackedStamp_ = -1.0;

    std::uint64_t rgbReceivedCount_ = 0;
    std::uint64_t depthReceivedCount_ = 0;
    std::uint64_t trackedCount_ = 0;
};

#endif  // RGBD_SLAM_NODE_HPP_