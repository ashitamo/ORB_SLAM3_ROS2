#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>

#include <cmath>
#include <iomanip>
#include <thread>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <atomic>
#include <cstdint>



using std::placeholders::_1;
namespace
{
bool IsFiniteImuMsg(const ImuMsg::SharedPtr &msg)
{
    if (!msg)
        return false;

    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;
    const double az = msg->linear_acceleration.z;

    const double gx = msg->angular_velocity.x;
    const double gy = msg->angular_velocity.y;
    const double gz = msg->angular_velocity.z;

    if (!std::isfinite(ax) ||
        !std::isfinite(ay) ||
        !std::isfinite(az) ||
        !std::isfinite(gx) ||
        !std::isfinite(gy) ||
        !std::isfinite(gz))
    {
        return false;
    }

    /*
     * 基本安全範圍，不是 IMU 的實際量測上限。
     * 主要用來擋住明顯損壞的資料。
     */
    const double accNorm =
        std::sqrt(ax * ax + ay * ay + az * az);

    const double gyroNorm =
        std::sqrt(gx * gx + gy * gy + gz * gz);

    if (accNorm > 100.0 || gyroNorm > 20.0)
        return false;

    return true;
}
}

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
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

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    subImu_ = this->create_subscription<ImuMsg>(
        "/imu/data",
        rclcpp::SensorDataQoS(),
        std::bind(&StereoInertialNode::GrabImu, this, _1));

    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    image_qos.reliable();
    image_qos.durability_volatile();

    subImgLeft_ = this->create_subscription<ImageMsg>(
        "/camera/camera/infra1/image_rect_raw",
        image_qos,
        std::bind(&StereoInertialNode::GrabImageLeft, this, _1));

    subImgRight_ = this->create_subscription<ImageMsg>(
        "/camera/camera/infra2/image_rect_raw",
        image_qos,
        std::bind(&StereoInertialNode::GrabImageRight, this, _1));

    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    if (syncThread_)
    {
        if (syncThread_->joinable())
            syncThread_->join();

        delete syncThread_;
        syncThread_ = nullptr;
    }

    SLAM_->Shutdown();
    SLAM_->SaveKeyFrameTrajectoryTUM(
        "KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(
    const ImuMsg::SharedPtr msg)
{
    if (!msg)
        return;

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

    const double newTime =
        Utility::StampToSec(msg->header.stamp);

    if (!std::isfinite(newTime) || newTime <= 0.0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Rejected IMU with invalid timestamp: %.9f",
            newTime);

        return;
    }

    std::lock_guard<std::mutex> lock(bufMutex_);

    if (!imuBuf_.empty())
    {
        const double lastTime =
            Utility::StampToSec(
                imuBuf_.back()->header.stamp);

        if (newTime <= lastTime)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejected non-monotonic IMU timestamp: "
                "new=%.9f last=%.9f dt=%.9f",
                newTime,
                lastTime,
                newTime - lastTime);

            return;
        }
    }

    while (imuBuf_.size() >= 2000)
        imuBuf_.pop();

    imuBuf_.push(msg);
}


void StereoInertialNode::GrabImageLeft(
    const ImageMsg::SharedPtr msgLeft)
{
    if (!msgLeft)
        return;
    ++leftReceivedCount_;
    const double stamp =
        Utility::StampToSec(msgLeft->header.stamp);

    std::lock_guard<std::mutex> lock(bufMutexLeft_);

    if (lastLeftStamp_ > 0.0)
    {
        const double dt = stamp - lastLeftStamp_;

        if (dt > 0.05)
        {
            std::cerr
                << std::fixed
                << std::setprecision(9)
                << "[LEFT INPUT GAP]"
                << " previous=" << lastLeftStamp_
                << " current=" << stamp
                << " dt=" << dt
                << std::endl;
        }
    }

    lastLeftStamp_ = stamp;

    while (imgLeftBuf_.size() >= 10)
    {
        const double droppedTime =
            Utility::StampToSec(
                imgLeftBuf_.front()->header.stamp);

        std::cerr
            << std::fixed
            << std::setprecision(9)
            << "[LEFT QUEUE] Overflow, dropping image: "
            << droppedTime
            << std::endl;

        imgLeftBuf_.pop();
    }

    imgLeftBuf_.push(msgLeft);
}

void StereoInertialNode::GrabImageRight(
    const ImageMsg::SharedPtr msgRight)
{
    if (!msgRight)
        return;
    ++rightReceivedCount_;
    const double stamp =
        Utility::StampToSec(msgRight->header.stamp);

    std::lock_guard<std::mutex> lock(bufMutexRight_);

    if (lastRightStamp_ > 0.0)
    {
        const double dt = stamp - lastRightStamp_;

        if (dt > 0.05)
        {
            std::cerr
                << std::fixed
                << std::setprecision(9)
                << "[RIGHT INPUT GAP]"
                << " previous=" << lastRightStamp_
                << " current=" << stamp
                << " dt=" << dt
                << std::endl;
        }
    }

    lastRightStamp_ = stamp;

    while (imgRightBuf_.size() >= 10)
    {
        const double droppedTime =
            Utility::StampToSec(
                imgRightBuf_.front()->header.stamp);

        std::cerr
            << std::fixed
            << std::setprecision(9)
            << "[RIGHT QUEUE] Overflow, dropping image: "
            << droppedTime
            << std::endl;

        imgRightBuf_.pop();
    }

    imgRightBuf_.push(msgRight);
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    if (!msg)
    {
        RCLCPP_ERROR(this->get_logger(), "Received null image message");
        return cv::Mat();
    }

    try
    {
        cv_bridge::CvImageConstPtr cv_ptr =
            cv_bridge::toCvShare(
                msg,
                sensor_msgs::image_encodings::MONO8);

        if (!cv_ptr || cv_ptr->image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge returned empty image");
            return cv::Mat();
        }

        if (cv_ptr->image.type() != CV_8UC1)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unexpected image type: %d",
                cv_ptr->image.type());
            return cv::Mat();
        }

        return cv_ptr->image.clone();
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "cv_bridge exception: %s",
            e.what());
        return cv::Mat();
    }
}

void StereoInertialNode::SyncWithImu()
{
    /*
     * Kalibr 定義：
     * t_imu = t_cam + timeshift_cam_imu
     *
     * 先前 Kalibr 結果：
     * cam0 = 0.08815577028178322 s
     */
    constexpr double camImuTimeShiftSec = 0.0;//0.08815577028178322;

    std::cout
        << std::fixed
        << std::setprecision(9)
        << "[TIME SHIFT] Camera timestamp shift = "
        << camImuTimeShiftSec
        << " s"
        << std::endl;

    const double maxStereoTimeDiff = 0.01;
    const std::size_t minImuMeasurements = 2;

    while (rclcpp::ok())
    {
        ImageMsg::SharedPtr leftMsg;
        ImageMsg::SharedPtr rightMsg;

        double tImLeft = 0.0;
        double tImRight = 0.0;

        /*
         * 第一階段：
         * 找到一組左右時間相符的影像。
         * 此處先不要 pop。
         */
        {
            std::scoped_lock lock(
                bufMutexLeft_,
                bufMutexRight_);

            if (!imgLeftBuf_.empty() &&
                !imgRightBuf_.empty())
            {
                const auto candidateLeft =
                    imgLeftBuf_.front();

                const auto candidateRight =
                    imgRightBuf_.front();

                const double tImLeftRaw =
                    Utility::StampToSec(
                        candidateLeft->header.stamp);

                const double tImRightRaw =
                    Utility::StampToSec(
                        candidateRight->header.stamp);

                tImLeft =
                    tImLeftRaw + camImuTimeShiftSec;

                tImRight =
                    tImRightRaw + camImuTimeShiftSec;

                const double stereoTimeDiff =
                    std::abs(tImLeftRaw - tImRightRaw);

                if (stereoTimeDiff > maxStereoTimeDiff)
                {
                    if (tImLeftRaw < tImRightRaw)
                    {
                        imgLeftBuf_.pop();
                    }
                    else
                    {
                        imgRightBuf_.pop();
                    }

                    std::cout
                        << std::fixed
                        << std::setprecision(9)
                        << "stereo resync: "
                        << "left_raw=" << tImLeftRaw
                        << ", right_raw=" << tImRightRaw
                        << ", abs_diff=" << stereoTimeDiff
                        << ", left_received=" << leftReceivedCount_
                        << ", right_received=" << rightReceivedCount_
                        << std::endl;
                }
                else
                {
                    leftMsg = candidateLeft;
                    rightMsg = candidateRight;
                }
            }
        }

        if (!leftMsg || !rightMsg)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));
            continue;
        }

        /*
         * 第二階段：
         * 確認 IMU queue 已經跨過目前影像時間。
         *
         * 不只檢查最新 IMU，也檢查是否有影像時間以前的 IMU。
         */
        bool imuReady = false;
        bool imageTooOldForImu = false;

        {
            std::lock_guard<std::mutex> lock(bufMutex_);

            if (!imuBuf_.empty())
            {
                const double oldestImuTime =
                    Utility::StampToSec(
                        imuBuf_.front()->header.stamp);

                const double latestImuTime =
                    Utility::StampToSec(
                        imuBuf_.back()->header.stamp);

                /*
                 * latest >= image：
                 * 表示 IMU 已經追到影像時間。
                 *
                 * oldest <= image：
                 * 表示至少有一筆影像之前的 IMU。
                 *
                 * 如果有 lastImuMsg_，則它也可作為影像前的邊界。
                 */
                const bool hasOlderImu =
                    oldestImuTime <= tImLeft;

                imuReady =
                    hasOlderImu &&
                    latestImuTime >= tImLeft;

                imageTooOldForImu =
                    oldestImuTime > tImLeft;
            }
        }

        if (imageTooOldForImu)
        {
            std::cerr
                << std::fixed
                << std::setprecision(9)
                << "[SYNC] Dropping image because it is older "
                << "than the oldest available IMU: image="
                << tImLeft
                << std::endl;

            /*
             * 這張影像已不可能補齊 IMU，只能安全丟棄。
             */
            {
                std::scoped_lock lock(
                    bufMutexLeft_,
                    bufMutexRight_);

                if (!imgLeftBuf_.empty() &&
                    imgLeftBuf_.front() == leftMsg)
                {
                    imgLeftBuf_.pop();
                }

                if (!imgRightBuf_.empty() &&
                    imgRightBuf_.front() == rightMsg)
                {
                    imgRightBuf_.pop();
                }
            }

            continue;
        }

        if (!imuReady)
        {
            /*
             * IMU 尚未追上，不可 pop 影像。
             */
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));
            continue;
        }

        /*
         * 第三階段：
         * 建立這一張影像所需的 IMU vector。
         *
         * 先放入上一張影像區間最後一筆 IMU，
         * 再加入目前影像時間以前的所有新 IMU。
         */
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        std::vector<ImuMsg::SharedPtr> imuMsgsToConsume;
        {
            std::lock_guard<std::mutex> lock(bufMutex_);
            auto tempImuBuf = imuBuf_;

            while (!tempImuBuf.empty())
            {
                const auto imuMsg = tempImuBuf.front();

                const double t =
                    Utility::StampToSec(
                        imuMsg->header.stamp);

                if (t > tImLeft)
                {
                    break;
                }

                tempImuBuf.pop();

                if (!IsFiniteImuMsg(imuMsg))
                {
                    continue;
                }

                imuMsgsToConsume.push_back(imuMsg);

                if (vImuMeas.empty() ||
                    std::abs(vImuMeas.back().t - t) > 1e-9)
                {
                    cv::Point3f acc(
                        imuMsg->linear_acceleration.x,
                        imuMsg->linear_acceleration.y,
                        imuMsg->linear_acceleration.z);

                    cv::Point3f gyr(
                        imuMsg->angular_velocity.x,
                        imuMsg->angular_velocity.y,
                        imuMsg->angular_velocity.z);

                    vImuMeas.emplace_back(acc, gyr, t);
                }
            }
        }

        /*
        * 檢查 IMU vector 是否正常。
        */
        bool validImuVector = true;

        for (std::size_t i = 0; i < vImuMeas.size(); ++i)
        {
            const auto &p = vImuMeas[i];

            if (!p.a.allFinite() ||
                !p.w.allFinite() ||
                !std::isfinite(p.t))
            {
                std::cerr
                    << "[SYNC] Non-finite IMU value at index "
                    << i
                    << std::endl;

                validImuVector = false;
                break;
            }

            if (i > 0)
            {
                const double imuDt =
                    p.t - vImuMeas[i - 1].t;

                if (!std::isfinite(imuDt) ||
                    imuDt <= 0.0 ||
                    imuDt > 0.1)
                {
                    std::cerr
                        << std::fixed
                        << std::setprecision(9)
                        << "[SYNC] Invalid IMU dt at index "
                        << i
                        << ": previous=" << vImuMeas[i - 1].t
                        << " current=" << p.t
                        << " dt=" << imuDt
                        << std::endl;

                    validImuVector = false;
                    break;
                }
            }
        }

        if (!validImuVector)
        {
            std::cerr
                << "[SYNC] Dropping image due to invalid IMU vector"
                << std::endl;

            /*
            * 丟影像，但不要消耗真正的 IMU queue。
            */
            {
                std::scoped_lock lock(
                    bufMutexLeft_,
                    bufMutexRight_);

                if (!imgLeftBuf_.empty() &&
                    imgLeftBuf_.front() == leftMsg)
                {
                    imgLeftBuf_.pop();
                }

                if (!imgRightBuf_.empty() &&
                    imgRightBuf_.front() == rightMsg)
                {
                    imgRightBuf_.pop();
                }
            }

            continue;
        }

        /*
        * 少於兩筆就不能做有效積分。
        * 這時直接丟影像，不要等待同一張影像。
        */
        if (vImuMeas.size() < minImuMeasurements)
        {
            std::cerr
                << std::fixed
                << std::setprecision(9)
                << "[SYNC] Dropping image due to insufficient IMU: "
                << "image=" << tImLeft
                << " count=" << vImuMeas.size()
                << std::endl;

            {
                std::scoped_lock lock(
                    bufMutexLeft_,
                    bufMutexRight_);

                if (!imgLeftBuf_.empty() &&
                    imgLeftBuf_.front() == leftMsg)
                {
                    imgLeftBuf_.pop();
                }

                if (!imgRightBuf_.empty() &&
                    imgRightBuf_.front() == rightMsg)
                {
                    imgRightBuf_.pop();
                }
            }

            continue;
        }

        /*
        * 到這裡才正式從真正的 IMU queue 移除資料。
        */
        {
            std::lock_guard<std::mutex> lock(bufMutex_);

            bool commitOk = true;

            for (const auto &expectedMsg : imuMsgsToConsume)
            {
                if (imuBuf_.empty() ||
                    imuBuf_.front() != expectedMsg)
                {
                    std::cerr
                        << "[SYNC] IMU queue changed unexpectedly"
                        << std::endl;

                    commitOk = false;
                    break;
                }

                imuBuf_.pop();
            }

            if (!commitOk)
            {
                continue;
            }
        }

        /*
         * 確認 IMU vector 合法後，現在才 pop 影像。
         */
        {
            std::scoped_lock lock(
                bufMutexLeft_,
                bufMutexRight_);

            if (imgLeftBuf_.empty() ||
                imgRightBuf_.empty())
            {
                continue;
            }

            if (imgLeftBuf_.front() != leftMsg ||
                imgRightBuf_.front() != rightMsg)
            {
                continue;
            }

            imgLeftBuf_.pop();
            imgRightBuf_.pop();
        }

        cv::Mat imLeft = GetImage(leftMsg);
        cv::Mat imRight = GetImage(rightMsg);

        if (imLeft.empty() || imRight.empty())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Empty image after conversion: "
                "left_empty=%d right_empty=%d",
                static_cast<int>(imLeft.empty()),
                static_cast<int>(imRight.empty()));

            continue;
        }

        if (imLeft.size() != imRight.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Stereo image size mismatch: "
                "left=%dx%d right=%dx%d",
                imLeft.cols,
                imLeft.rows,
                imRight.cols,
                imRight.rows);

            continue;
        }

        if (bClahe_)
        {
            clahe_->apply(imLeft, imLeft);
            clahe_->apply(imRight, imRight);
        }

        if (doRectify_)
        {
            cv::Mat rectLeft;
            cv::Mat rectRight;

            cv::remap(
                imLeft,
                rectLeft,
                M1l_,
                M2l_,
                cv::INTER_LINEAR);

            cv::remap(
                imRight,
                rectRight,
                M1r_,
                M2r_,
                cv::INTER_LINEAR);

            imLeft = rectLeft;
            imRight = rectRight;
        }

        if (imLeft.empty() || imRight.empty())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Image became empty after preprocessing");
            continue;
        }

        /*
         * 顯示實際送進 ORB-SLAM3 的影像時間間隔。
         */
        double imageDt = 0.0;

        if (lastTrackedImageTime_ > 0.0)
        {
            imageDt = tImLeft - lastTrackedImageTime_;
        }

        static std::size_t syncDebugCounter = 0;

        if ((syncDebugCounter++ % 30) == 0)
        {
            std::cout
                << std::fixed
                << std::setprecision(9)
                << "[SYNC]"
                << " image=" << tImLeft
                << " image_dt=" << imageDt
                << " imu_count=" << vImuMeas.size()
                << " first=" << vImuMeas.front().t
                << " last=" << vImuMeas.back().t
                << " image_minus_last="
                << tImLeft - vImuMeas.back().t
                << std::endl;
        }

        /*
         * 最後一道保護。
         */
        if (vImuMeas.size() < minImuMeasurements)
        {
            std::cerr
                << "[SYNC] Refusing TrackStereo because "
                << "the IMU vector is too small"
                << std::endl;
            continue;
        }

        static double previousLastImuTime = -1.0;
        if (!vImuMeas.empty())
        {
            if (previousLastImuTime > 0.0)
            {
                const double imuBoundaryGap =
                    vImuMeas.front().t - previousLastImuTime;

                if (std::abs(imuBoundaryGap) > 0.02)
                {
                    std::cerr
                        << std::fixed
                        << std::setprecision(9)
                        << "[SYNC] IMU boundary gap="
                        << imuBoundaryGap
                        << " previous_last="
                        << previousLastImuTime
                        << " current_first="
                        << vImuMeas.front().t
                        << std::endl;
                }
            }

            previousLastImuTime = vImuMeas.back().t;
        }

        // double maxAccNorm = 0.0;
        // double maxGyroNorm = 0.0;

        // for (const auto &imu : vImuMeas)
        // {
        //     const double accNorm =
        //         static_cast<double>(imu.a.norm());

        //     const double gyroNorm =
        //         static_cast<double>(imu.w.norm());

        //     maxAccNorm = std::max(maxAccNorm, accNorm);
        //     maxGyroNorm = std::max(maxGyroNorm, gyroNorm);
        // }

        // std::cout
        //     << " max_acc=" << maxAccNorm
        //     << " max_gyro=" << maxGyroNorm;

        SLAM_->TrackStereo(
            imLeft,
            imRight,
            tImLeft,
            vImuMeas);

        lastTrackedImageTime_ = tImLeft;

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }
}