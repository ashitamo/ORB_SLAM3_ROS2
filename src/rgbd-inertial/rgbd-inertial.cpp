#include "rgbd-inertial-node.hpp"

#include "System.h"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <memory>
#include <string>

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

    /*
     * ros2 run 會先處理 --ros-args，但 argv 中仍可能包含
     * ROS 相關參數，因此使用 remove_ros_arguments() 取得程式參數。
     */
    const std::vector<std::string> arguments =
        rclcpp::remove_ros_arguments(
            argc,
            argv);

    if (arguments.size() != 4)
    {
        PrintUsage(
            arguments.empty()
                ? argv[0]
                : arguments[0].c_str());

        rclcpp::shutdown();
        return 1;
    }

    const std::string vocabularyPath =
        arguments[1];

    const std::string settingsPath =
        arguments[2];

    const bool visualization =
        ParseBool(
            arguments[3]);

    std::cout
        << "Starting ORB-SLAM3 RGB-D-Inertial"
        << std::endl;

    std::cout
        << "Vocabulary: "
        << vocabularyPath
        << std::endl;

    std::cout
        << "Settings: "
        << settingsPath
        << std::endl;

    std::cout
        << "Visualization: "
        << std::boolalpha
        << visualization
        << std::endl;

    /*
     * 關鍵：必須使用 IMU_RGBD，不能使用 RGBD。
     */
    ORB_SLAM3::System SLAM(
        vocabularyPath,
        settingsPath,
        ORB_SLAM3::System::IMU_RGBD,
        visualization);

    // SLAM.ActivateLocalizationMode();

    auto node =
        std::make_shared<RgbdInertialNode>(
            &SLAM);

    rclcpp::spin(node);

    std::cout
        << "Stopping RGB-D-Inertial synchronization thread..."
        << std::endl;

    node->Stop();

    std::cout
        << "Destroying ROS2 node..."
        << std::endl;

    node.reset();

    std::cout
        << "Shutting down ORB-SLAM3..."
        << std::endl;

    SLAM.Shutdown();

    std::cout
        << "ORB-SLAM3 shutdown completed."
        << std::endl;

    rclcpp::shutdown();

    return 0;
}
