#include "rgbd-inertial-node.hpp"

#include "System.h"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <memory>
#include <string>
#include <exception>

namespace
{

bool ParseBool(const std::string& value)
{
    std::string normalized = value;

    std::transform(
        normalized.begin(),
        normalized.end(),
        normalized.begin(),
        [](unsigned char c)
        {
            return static_cast<char>(
                std::tolower(c));
        });

    return normalized == "true" ||
           normalized == "1" ||
           normalized == "yes" ||
           normalized == "on";
}

void PrintUsage(const char* programName)
{
    std::cerr
        << "Usage:\n"
        << "  " << programName
        << " vocabulary_file settings_file visualization\n\n"
        << "Example:\n"
        << "  " << programName
        << " /path/to/ORBvoc.txt"
        << " /path/to/D405_rgbd_inertial.yaml"
        << " false"
        << std::endl;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    const std::vector<std::string> arguments = rclcpp::remove_ros_arguments(argc, argv);

    if (arguments.size() != 6)
    {
        PrintUsage(arguments.empty() ? argv[0] : arguments[0].c_str());
        rclcpp::shutdown();
        return 1;
    }

    const std::string vocabularyPath = arguments[1];
    const std::string settingsPath = arguments[2];
    const bool visualization = ParseBool(arguments[3]);
    const std::string cameraTrajectoryPath = arguments[4];
    const std::string keyFrameTrajectoryPath = arguments[5];

    std::cout << "Starting ORB-SLAM3 RGB-D-Inertial" << std::endl;
    std::cout << "Vocabulary: " << vocabularyPath << std::endl;
    std::cout << "Settings: " << settingsPath << std::endl;
    std::cout << "Visualization: " << std::boolalpha << visualization << std::endl;
    std::cout << "Camera trajectory: " << cameraTrajectoryPath << std::endl;
    std::cout << "Keyframe trajectory: " << keyFrameTrajectoryPath << std::endl;

    ORB_SLAM3::System SLAM(
        vocabularyPath,
        settingsPath,
        ORB_SLAM3::System::IMU_RGBD,
        visualization
    );

    // SLAM.ActivateLocalizationMode();

    auto node = std::make_shared<RgbdInertialNode>(&SLAM);

    rclcpp::spin(node);

    std::cout << "Stopping RGB-D-Inertial synchronization thread..." << std::endl;
    node->Stop();

    std::cout << "Shutting down ORB-SLAM3..." << std::endl;
    SLAM.Shutdown();

    try
    {
        SLAM.SaveTrajectoryTUM(cameraTrajectoryPath);
        SLAM.SaveKeyFrameTrajectoryTUM(keyFrameTrajectoryPath);

        RCLCPP_INFO(node->get_logger(), "Camera trajectory saved to: %s", cameraTrajectoryPath.c_str());
        RCLCPP_INFO(node->get_logger(), "Keyframe trajectory saved to: %s", keyFrameTrajectoryPath.c_str());
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to save trajectory: %s", error.what());
    }

    std::cout << "Destroying ROS2 node..." << std::endl;
    node.reset();

    std::cout << "ORB-SLAM3 shutdown completed." << std::endl;
    rclcpp::shutdown();

    return 0;
}
