#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <filesystem>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <unistd.h>

class AtlasSettings
{
public:
    std::string path;

    explicit AtlasSettings(const std::string& source) : path(source)
    {
        rclcpp::Node parameters("ORB_SLAM3_ROS2");
        const auto atlas = parameters.declare_parameter<std::string>("atlas_path", "");
        const auto save = parameters.declare_parameter<std::string>("atlas_save_path", "");
        const bool overwrite = parameters.declare_parameter<bool>("atlas_overwrite", true);
        if (atlas.empty() && save.empty()) return;
        cv::FileStorage settings(source, cv::FileStorage::READ);
        if (!settings.isOpened()) throw std::runtime_error("Cannot open settings: " + source);
        const std::string loadName = settings["System.LoadAtlasFromFile"].empty()
            ? "" : static_cast<std::string>(settings["System.LoadAtlasFromFile"]);
        const bool locating = !loadName.empty();
        if (atlas.empty()) throw std::runtime_error("atlas_save_path requires atlas_path");
        auto atlasFile = std::filesystem::absolute(atlas).lexically_normal();
        if (atlasFile.extension() != ".osa") atlasFile += ".osa";
        if (locating && !std::filesystem::is_regular_file(atlasFile))
            throw std::runtime_error("Atlas not found: " + atlasFile.string());
        auto saveFile = atlasFile;
        if (!save.empty()) saveFile = std::filesystem::absolute(save).lexically_normal();
        else if (locating) saveFile = atlasFile.parent_path() / (atlasFile.stem().string() + "_locating.osa");
        if (saveFile.extension() != ".osa") saveFile += ".osa";
        if (!overwrite && std::filesystem::exists(saveFile))
            throw std::runtime_error("Atlas output exists: " + saveFile.string() + "; use atlas_overwrite:=true to replace it");
        std::filesystem::create_directories(saveFile.parent_path());
        auto stemForCore = [](std::filesystem::path filename) {
            filename.replace_extension();
            return std::filesystem::relative(filename, std::filesystem::current_path()).generic_string();
        };
        std::ifstream input(source);
        std::string content, line;
        const std::regex atlasKey(R"(^\s*System\.(LoadAtlasFromFile|SaveAtlasToFile)\s*:)");
        while (std::getline(input, line))
            if (!std::regex_search(line, atlasKey)) content += line + "\n";
        auto yamlValue = [](const std::string& value) {
            cv::FileStorage writer(".yaml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
            writer << "value" << value;
            const auto serialized = writer.releaseAndGetString();
            return serialized.substr(serialized.find("value:") + 6);
        };
        content += "System.LoadAtlasFromFile: " + yamlValue(locating ? stemForCore(atlasFile) : "");
        content += "System.SaveAtlasToFile: " + yamlValue(stemForCore(saveFile));
        std::string pattern = (std::filesystem::temp_directory_path() / "orbslam-settings-XXXXXX").string();
        std::vector<char> buffer(pattern.begin(), pattern.end());
        buffer.push_back('\0');
        const int descriptor = mkstemp(buffer.data());
        if (descriptor < 0) throw std::runtime_error("Cannot create temporary settings");
        close(descriptor);
        temporary_ = buffer.data();
        std::ofstream output(temporary_);
        output << content;
        output.close();
        if (!output) throw std::runtime_error("Cannot write temporary settings");
        path = temporary_;
        RCLCPP_INFO(parameters.get_logger(), "Atlas mode=%s, input=%s, output=%s",
            locating ? "locating" : "mapping", locating ? atlasFile.c_str() : "none", saveFile.c_str());
    }

    ~AtlasSettings()
    {
        if (!temporary_.empty()) std::remove(temporary_.c_str());
    }

private:
    std::string temporary_;
};
