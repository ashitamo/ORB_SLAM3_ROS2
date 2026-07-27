# D405 + Xsens + ORB-SLAM3 RGB-D-Inertial

> 目前穩定基準：**D405 Infra1 + Depth + Xsens IMU**
>
> 本文件以目前可運行版本、近期測試 log 與既有專案整理為基礎。  
> 為避免把推測寫成既定事實，本文使用以下標記：
>
> - ✅ 已由目前程式、執行輸出或測試確認
> - ⚠️ 合理推測，但尚未完成針對性驗證
> - 🧪 已做過實驗，但結論仍受測試條件限制
> - 📌 尚待完成
> - 🔎 需要重新檢查目前原始碼後才能確認

---

## 1. 專案概述

本專案在 Ubuntu 22.04 與 ROS 2 Humble 上，整合：

- Intel RealSense D405
- Xsens IMU
- ORB-SLAM3
- 自訂 ROS 2 wrapper

目前實際送入 ORB-SLAM3 的資料為：

```text
影像：/camera/camera/infra1/image_rect_raw
深度：/camera/camera/depth/image_rect_raw
IMU： /imu/data
```

雖然 ORB-SLAM3 的介面名稱是 `RGB-D-Inertial`，目前 `camera/rgb` 實際 remap 到 `infra1`。因此目前版本應稱為：

> **Infra–Depth–Inertial ORB-SLAM3**

而不是已完成的彩色 RGB-D-Inertial 系統。

### 目前狀態

✅ 可從空 Atlas 建圖  
✅ 可完成 IMU 初始化  
✅ 可完成 VIBA 1、VIBA 2  
✅ 已恢復完整 IMU prediction 與 inertial optimization  
✅ 中低速移動下可穩定追蹤  
✅ 停止後速度與位置可重新收斂  
✅ 已能記錄完整執行 log  
📌 尚未完成彩色影像版本  
📌 尚未提供 trajectory 檔案輸出  
📌 尚未重做最終 Kalibr  
📌 尚未完成 ROS 2 pose / odometry / TF 的正式輸出與驗證  
📌 尚未完成 ground truth 定量精度評估

---

## 2. 軟硬體環境

### 2.1 已知環境

```text
OS：Ubuntu 22.04
ROS 2：Humble
Camera：Intel RealSense D405
IMU：Xsens
ORB-SLAM3 sensor mode：RGB-D-Inertial
```

### 2.2 目前執行參數

```text
影像解析度：848 × 480
Camera FPS：30 Hz
IMU frequency：100 Hz
IMU period：0.01 s
imu_time_offset_sec：0.0 s
ORB features per image：1500
Initial FAST threshold：20
Minimum FAST threshold：7
```

### 2.3 目前影像曝光設定

一般基準設定：

```text
enable_auto_exposure = false
exposure = 6000
gain = 64
```

暗環境、中低速候選設定：

```text
enable_auto_exposure = false
exposure = 16000
gain = 16
```

目前測試顯示：

- `6000 / 64` 對一般移動較平衡。
- `16000 / 16` 在暗環境與中低速下可得到良好的特徵匹配。
- `16000 / 16` 尚未充分驗證高速旋轉，長曝光可能增加 motion blur。
- `4000 / 64` 在目前場景偏暗。
- `4000 / 80` 可補亮度，但提高 gain 也會增加影像雜訊。

---

## 3. 資料夾結構

以下是目前對話與執行指令中已確認的主要路徑。實際封版前應再用 `tree` 或 `find` 核對一次。

```text
/home/lab606/
├── slam_ws/
│   └── src/
│       └── ORB_SLAM3/
│           ├── Vocabulary/
│           │   └── ORBvoc.txt
│           ├── Examples/
│           │   └── RGB-D-Inertial/
│           │       └── D405_rgbd_inertial_test.yaml
│           ├── include/
│           ├── src/
│           │   ├── Tracking.cc
│           │   ├── LocalMapping.cc
│           │   ├── Optimizer.cc
│           │   └── ...
│           ├── Thirdparty/
│           └── ...
│
├── orbslam3_ros2_ws/
│   ├── src/
│   │   └── ORB_SLAM3_ROS2/
│   │       ├── src/
│   │       │   └── rgbd-inertial/
│   │       │       ├── rgbd-inertial-node.cpp
│   │       │       └── ...
│   │       ├── include/
│   │       ├── config/
│   │       │   └── mask_left.png
│   │       └── ...
│   ├── build/
│   ├── install/
│   └── log/
│
└── orbslam3_logs/
    └── rgbd_imu_YYYYMMDD_HHMMSS.log
```

### 建議封版時保存

```text
ORB_SLAM3 commit hash
ORB_SLAM3_ROS2 commit hash
D405_rgbd_inertial_test.yaml
mask_left.png
目前啟動指令
曝光參數
Xsens 輸出設定
範例 log
Kalibr 輸出檔
README.md
```

---

## 4. 安裝與編譯

> ⚠️ 本節目前是「現有機器上的重建流程」，不是已完整驗證的 clean-room 安裝指南。  
> Pangolin、OpenCV、Eigen、Sophus、RealSense ROS 與 Xsens driver 的精確版本仍應在封版時補上。

### 4.1 ROS 2 環境

```bash
source /opt/ros/humble/setup.bash
```

### 4.2 ORB-SLAM3 core

目前 ORB-SLAM3 core 位於：

```text
/home/lab606/slam_ws/src/ORB_SLAM3
```

✅ 目前已可正常編譯並產生 ORB-SLAM3 library。  
📌 尚需在正式 README 補上從全新環境開始的完整 dependency 安裝與 core build 指令。

建議封版前記錄：

```bash
cd /home/lab606/slam_ws/src/ORB_SLAM3
git rev-parse HEAD
git status
```

### 4.3 ROS 2 wrapper

```bash
cd ~/orbslam3_ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select orbslam3 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source ~/orbslam3_ros2_ws/install/setup.bash
```

### 4.4 驗證執行檔

```bash
ros2 pkg executables orbslam3
```

預期至少能看到：

```text
orbslam3 rgbd-inertial
```

---

## 5. 使用方式

## 5.1 啟動相機與 IMU

📌 RealSense 與 Xsens 的完整 launch 指令目前沒有在本文件中確認，正式封版前必須補上。

啟動後至少應存在：

```text
/camera/camera/infra1/image_rect_raw
/camera/camera/depth/image_rect_raw
/imu/data
```

檢查 topic：

```bash
ros2 topic list | grep -E "infra1|depth|imu"
```

檢查頻率：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /imu/data
```

目前預期：

```text
infra1：約 30 Hz
depth：約 30 Hz
imu：約 100 Hz
```

檢查一次 IMU：

```bash
ros2 topic echo /imu/data --once
```

---

## 5.2 固定曝光

### 一般使用

```bash
ros2 param set /camera/camera \
  depth_module.enable_auto_exposure false

ros2 param set /camera/camera \
  depth_module.exposure 6000

ros2 param set /camera/camera \
  depth_module.gain 64
```

### 暗環境、中低速使用

```bash
ros2 param set /camera/camera \
  depth_module.enable_auto_exposure false

ros2 param set /camera/camera \
  depth_module.exposure 16000

ros2 param set /camera/camera \
  depth_module.gain 16
```

確認設定：

```bash
ros2 param get /camera/camera depth_module.enable_auto_exposure
ros2 param get /camera/camera depth_module.exposure
ros2 param get /camera/camera depth_module.gain
```

---

## 5.3 啟動 ORB-SLAM3

```bash
source /opt/ros/humble/setup.bash
source ~/orbslam3_ros2_ws/install/setup.bash

ros2 run orbslam3 rgbd-inertial \
  /home/lab606/slam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/lab606/slam_ws/src/ORB_SLAM3/Examples/RGB-D-Inertial/D405_rgbd_inertial_test.yaml \
  true \
  --ros-args \
  -p imu_time_offset_sec:=0.0 \
  -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
  -r camera/depth:=/camera/camera/depth/image_rect_raw
```

說明：

```text
true
```

代表開啟 Viewer。

目前 `camera/rgb` remap 到 `infra1`，這是刻意的，不代表已使用彩色影像。

---

## 5.4 啟動並記錄 log

```bash
mkdir -p ~/orbslam3_logs

LOG=~/orbslam3_logs/rgbd_imu_$(date +%Y%m%d_%H%M%S).log

ros2 run orbslam3 rgbd-inertial \
  /home/lab606/slam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/lab606/slam_ws/src/ORB_SLAM3/Examples/RGB-D-Inertial/D405_rgbd_inertial_test.yaml \
  true \
  --ros-args \
  -p imu_time_offset_sec:=0.0 \
  -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
  -r camera/depth:=/camera/camera/depth/image_rect_raw \
  2>&1 | tee "$LOG"

echo "Log saved to: $LOG"
```

按 `Ctrl+C` 結束。

---

## 5.5 初始化操作方式

啟動後不要只維持完全靜止，也不要立即快速甩動。

建議流程：

1. 先靜止約 1～2 秒。
2. 做適量 XYZ 平移。
3. 加入適量 roll、pitch、yaw。
4. 保持中等速度與清楚畫面。
5. 等待 `end VIBA 1`。
6. 等待 `end VIBA 2`。
7. 再開始正式使用或測試。

太慢可能出現：

```text
Not enough motion for initializing
```

太快可能造成：

```text
Fail to track local map!
```

---

## 5.6 執行中監看項目

目前自訂 log 會輸出：

```text
[TRACK DIAG]
[IMU STATE]
[TLM OPT MODE]
[RGB INPUT GAP]
[DEPTH INPUT GAP]
[RGBD PAIR REJECTED]
[RGBD-IMU SYNC]
start VIBA 1
end VIBA 1
start VIBA 2
end VIBA 2
```

經驗判斷：

```text
inliers > 300
通常追蹤良好

inliers 150～300
仍可使用，但品質已下降

inliers < 100
進入高風險區域

連續 Fail to track local map!
應降低速度、增加光線或停止等待恢復
```

以上是目前專案的經驗值，不是 ORB-SLAM3 官方保證門檻。

---

## 6. 目前穩定基準設定

```text
Sensor mode：
RGB-D-Inertial

Image：
/camera/camera/infra1/image_rect_raw

Depth：
/camera/camera/depth/image_rect_raw

IMU：
/imu/data

Resolution：
848 × 480

Camera FPS：
30 Hz

IMU frequency：
100 Hz

IMU period：
0.01 s

IMU timestamp offset：
0.0 s

IMU prediction：
enabled

Inertial optimization：
enabled

Atlas：
測試時優先從頭建圖

Mask：
mask_left.png
```

核心開關應維持：

```cpp
kEnableImuPrediction = true;
kEnableInertialOptimization = true;
```

先前的關閉組合只用於問題定位，不是正式使用設定。

---

## 7. 過去完成的工作

## 7.1 由 Stereo-Inertial 轉向目前的 Infra–Depth–Inertial

早期文件主要記錄 D405 左右 infrared 的 Stereo-Inertial 整合。

目前實際穩定基準已改為：

```text
infra1 + depth + external IMU
```

因此舊 Stereo-Inertial 文件中的下列內容不能直接視為目前 RGB-D-Inertial branch 已具備：

- 左右影像配對邏輯
- Stereo resync
- 左右 frame drop
- Stereo baseline 處理
- Stereo wrapper 的 transactional queue 實作

🔎 若要在目前 README 宣稱這些機制已完整移植，必須重新檢查現行 `rgbd-inertial` 原始碼。

---

## 7.2 Camera–IMU 外參方向修正

✅ 先前 `IMU.T_b_c1` 的方向或矩陣定義曾有問題。  
✅ 已修正到目前可完成初始化、VIBA 與正常追蹤的狀態。

這代表外參方向錯誤已不再是目前的首要問題。

但：

📌 最終精度仍需透過重新執行 Kalibr 驗證。  
📌 不應將「目前能跑」解讀為「外參已達最終精度」。

---

## 7.3 IMU integration period 修正

目前使用：

```cpp
mImuPer = 1.0 / static_cast<double>(mImuFreq);
```

在 100 Hz 下：

```text
mImuPer = 0.01 s
```

✅ 已確認 runtime log 顯示：

```text
[IMU CONFIG] frequency=100 period=0.01
```

---

## 7.4 IMU timestamp offset 參數化

ROS 2 wrapper 增加：

```text
imu_time_offset_sec
```

定義：

```text
t_used = t_raw + imu_time_offset_sec
```

測試過：

```text
-10 ms
0 ms
+10 ms
```

結果：

- 三組都沒有顯示決定性改善。
- 固定 ±10 ms 不是目前主要問題。
- 目前保留 `0.0 s`。

早期舊 Kalibr 曾給出約 ±88 ms 等級的 offset；在目前資料鏈測試中會造成明顯不穩，因此不能直接沿用。

---

## 7.5 RGB-D–IMU 同步與診斷

目前 wrapper 已具備或已觀察到的功能：

✅ 等待 IMU 資料覆蓋影像時間  
✅ 檢查每張影像的 IMU measurement 數量  
✅ 記錄影像與最後一筆 IMU 的時間差  
✅ 記錄 RGB input gap  
✅ 記錄 depth input gap  
✅ 記錄 RGB-D pair rejected  
✅ 記錄 image interval 與 IMU count

正常情況通常為：

```text
image_dt：約 0.0333 s
imu_count：3 或 4
image_minus_imu_last：約 0～10 ms
```

🔎 是否完整保留前一影像邊界的 IMU sample、是否對邊界做正確插值，仍應在目前 source 中做一次正式 code review。

---

## 7.6 IMU prediction / optimizer A/B

測試過：

```text
Prediction = true
Optimization = true

Prediction = false
Optimization = true

Prediction = false
Optimization = false
```

結果顯示：

- 關閉 prediction 不能消除旋轉期間的暫態速度。
- 從空 Atlas 測試時仍可能出現相同行為。
- 問題不能單獨歸因於 Atlas、prediction 或 ±10 ms offset。

正式版本已恢復：

```text
Prediction = true
Optimization = true
```

---

## 7.7 Atlas 隔離測試

早期測試可能將失敗後新增的 KeyFrame 存入新的 Atlas，造成後續測試污染。

後續改成：

```text
Initialization of Atlas from scratch
```

確認：

- 從空 Atlas 仍可正常初始化。
- 載入 Atlas 不是系統運作的必要條件。
- Atlas 不是過去所有問題的唯一根因。

目前建議：

- 原始 Atlas 保持唯讀。
- 診斷測試不要覆寫穩定 Atlas。
- 不同測試使用不同輸出檔名。
- Atlas save/load 應獨立驗證。

---

## 7.8 曝光與 gain A/B

已比較：

```text
6000 / 64
4000 / 64
4000 / 80
16000 / 16
```

確認：

- 畫面亮度確實影響 ORB feature matching。
- 短曝光不一定比較好；畫面太暗也會減少有效特徵。
- 高 gain 可增加亮度，但會增加影像雜訊。
- 長曝光、低 gain 在暗環境中低速表現良好。
- 高速下仍需考慮 motion blur。

---

## 7.9 靜止收斂測試

✅ 完成 VIBA 後，設備停止並靜止時：

- `gyroNorm` 可回到很低。
- velocity 可下降到接近零。
- position disturbance 可回到毫米級。
- 系統不會必然持續不可恢復地飄走。

因此目前較準確的判斷是：

> 系統在旋轉或移動期間可能出現暫態速度與位移，但只要視覺追蹤仍有效，停止後通常能重新收斂。

---

## 8. 已解決的關鍵問題

| 問題 | 狀態 | 結果 |
|---|---:|---|
| `IMU.T_b_c1` 方向／矩陣定義錯誤 | ✅ 已修正 | 系統可正常初始化與融合 |
| IMU period 設定不正確 | ✅ 已修正 | 100 Hz 對應 0.01 s |
| 無法調整 Camera–IMU offset | ✅ 已完成 | ROS 2 parameter 可設定 |
| ±10 ms 是否為主因 | ✅ 已排除為主因 | 目前使用 0 ms |
| Prediction disabled 是否為主因 | ✅ 已排除 | 恢復 prediction=true |
| Atlas 是否為唯一根因 | ✅ 已排除 | 空 Atlas 仍可重現並可正常運作 |
| 無法觀察 tracking / IMU 狀態 | ✅ 已改善 | 增加 TRACK DIAG、IMU STATE 等 log |
| 不知道曝光是否影響 tracking | ✅ 已完成初步 A/B | 確認亮度、gain、motion blur 有交互影響 |
| 靜止後是否持續發散 | ✅ 已驗證 | 可重新收斂 |
| log 無法保存 | ✅ 已完成 | 使用 `tee` 保存完整輸出 |

---

## 9. 目前已知問題與原因推測

## 9.1 尚未完成彩色 RGB 版本

目前：

```text
camera/rgb -> infra1
```

📌 尚未正式切換並驗證 color image topic。

需要確認：

- color topic 名稱
- color camera_info
- color FPS 與解析度
- color 與 depth 的 timestamp 關係
- depth 是否需要 align 到 color
- mask 是否需重做
- color 對 ORB inliers 的影響
- CPU 與頻寬負載

### 重要決策

如果彩色影像只是資料收集用途，而 SLAM 仍使用 infra：

```text
SLAM：infra1 + depth + IMU
資料集：額外同步收 color
```

這通常比直接替換穩定 SLAM 輸入風險低。

如果最終 SLAM 一定要使用 color，則必須建立獨立 branch 與設定檔，不要直接覆蓋目前穩定 infra 版本。

---

## 9.2 尚未輸出 trajectory

目前 Pangolin Viewer 可顯示軌跡，但 ROS 2 wrapper 尚未完成可直接使用的 trajectory 輸出流程。

ORB-SLAM3 core 本身具備 trajectory save 類型的功能，但目前 wrapper 尚未接出或尚未驗證：

```text
SaveTrajectoryTUM
SaveKeyFrameTrajectoryTUM
SaveTrajectoryEuRoC
```

📌 建議加入：

```text
Ctrl+C 時自動儲存 trajectory
ROS 2 service 手動觸發儲存
輸出 TUM 格式
輸出 keyframe trajectory
輸出路徑與檔名參數
```

如果要接機器人系統，還應加入：

```text
/orbslam3/pose
/orbslam3/odom
TF: map -> camera_link
TF: map -> base_link
```

---

## 9.3 Kalibr 尚未重做

目前 calibration 可讓系統運作，但不是最終版本。

重新校正時必須使用：

```text
最終影像 topic
最終解析度
最終 FPS
最終 IMU topic
最終 IMU frequency
最終相機與 IMU 固定安裝
實際 driver timestamp pipeline
```

### 執行順序

如果最終 SLAM 使用 color：

```text
先完成 color branch
→ 再做最終 Kalibr
```

如果 SLAM 繼續使用 infra，而 color 只做資料收集：

```text
infra1 + IMU 做最終 Kalibr
→ color 另外保存 camera_info 與必要外參
```

---

## 9.4 RGB / depth 偶發掉幀或配對失敗

仍會出現：

```text
[RGB INPUT GAP]
[DEPTH INPUT GAP]
[RGBD PAIR REJECTED]
```

常見 `0.0667 s` 表示約掉 1 幀；更長間隔代表連續掉幀或 callback 延遲。

### 可能原因

⚠️ RealSense driver 本身掉幀  
⚠️ USB 頻寬或裝置問題  
⚠️ ROS 2 QoS 不合適  
⚠️ callback / executor 處理不及  
⚠️ Viewer、ORB-SLAM3 與大量 log 造成 CPU 負載  
⚠️ RGB 與 depth publisher 時序不同  
⚠️ wrapper queue / pairing policy 過度嚴格

### 目前不能直接下結論

只看 online wrapper log，不能判斷是：

```text
資料源已經缺幀
```

還是：

```text
資料有發布，但 wrapper 沒及時處理
```

需要 rosbag 離線驗證。

---

## 9.5 快速移動時 local map tracking 下降

快速旋轉或移動時，可能看到：

```text
inliers 下降
Fail to track local map!
```

### 可能原因

⚠️ motion blur  
⚠️ 相鄰幀視角差異過大  
⚠️ 暗環境造成特徵對比不足  
⚠️ 高 gain 造成影像雜訊  
⚠️ RGB / depth gap 進一步放大追蹤困難  
⚠️ 場景紋理不足或大量重複紋理

目前資料支持：

> 移動速度是明顯影響因素；光線與曝光設定會放大或減弱這個問題。

---

## 9.6 初始化期間移動太慢

如果初始化期間移動太小或激勵不足，可能出現：

```text
Not enough motion for initializing
Reset map because local mapper set the bad imu flag
```

這不是單純畫面太暗，而是 IMU initialization 缺少足夠運動激勵。

---

## 9.7 `avgA_last` 初始值異常

啟動時曾重複看到：

```text
[STEREO INIT] Unreasonable average acceleration
last=[非常大的數值]
```

這表示第一次使用前的 previous acceleration state 可能沒有正確初始化。

目前已有異常檢查或防護，因此後續通常能恢復，但根因尚未完全修正。

📌 應改成明確狀態：

```cpp
Eigen::Vector3f mLastInitAvgAcc = Eigen::Vector3f::Zero();
bool mbHasLastInitAvgAcc = false;
```

第一次使用時只初始化，不計算差值：

```cpp
if (!mbHasLastInitAvgAcc)
{
    mLastInitAvgAcc = avgA_current;
    mbHasLastInitAvgAcc = true;
    return;
}
```

reset active map 時也要清除：

```cpp
mbHasLastInitAvgAcc = false;
mLastInitAvgAcc.setZero();
```

---

## 9.8 IMU preintegration 邊界仍需正式檢查

目前每張 30 Hz 影像通常取得 3～4 筆 100 Hz IMU。

但仍需確認：

- 每個 image interval 是否包含正確的前後邊界 sample
- `lastImuMsg_` 是否正確延續到下一 frame
- 是否有 sample 重複或遺失
- preintegration 的 `dT` 是否約等於 image interval
- image gap 發生時是否累積到足夠 IMU
- reset 後是否清除 stale IMU state

建議新增：

```text
frame_dt
imu_first_time
imu_last_time
imu_count
preintegrated_dT
abs(frame_dt - preintegrated_dT)
```

---

## 9.9 尚未完成定量精度評估

目前只有：

- Viewer 視覺觀察
- inliers
- static convergence
- velocity / bias log
- 是否 lost

尚未有：

```text
ATE
RPE
loop closure error
repeatability
long-term drift
ground truth comparison
```

因此目前的「可以使用」代表：

> 可作為研究與整合用穩定原型。

不代表：

> 已完成可量化保證的定位產品。

---

## 10. 接下來要做的事

## P0：封存目前穩定版本

建議版本名稱：

```text
d405_infra_depth_imu_v0.1
```

保存並提交：

```bash
cd /home/lab606/slam_ws/src/ORB_SLAM3
git status
git rev-parse HEAD
git add .
git commit -m "Stable D405 infra-depth-inertial baseline"
git tag -a d405_infra_depth_imu_v0.1 \
  -m "Stable D405 infra-depth-inertial baseline"
```

ROS 2 wrapper 也要獨立 commit/tag：

```bash
cd /home/lab606/orbslam3_ros2_ws/src/ORB_SLAM3_ROS2
git status
git rev-parse HEAD
```

在 commit 前應避免把：

```text
build/
install/
log/
大型 Atlas
大型 rosbag
```

加入 Git。

---

## P1：加入 trajectory 與 live pose 輸出

優先實作：

```text
1. SaveTrajectoryTUM
2. SaveKeyFrameTrajectoryTUM
3. Ctrl+C 自動儲存
4. ROS 2 service 手動儲存
5. /orbslam3/pose
6. /orbslam3/odom
7. map -> camera_link TF
8. map -> base_link TF
```

建議 trajectory 輸出資料夾：

```text
~/orbslam3_trajectories/
```

檔名：

```text
camera_trajectory_YYYYMMDD_HHMMSS.txt
keyframe_trajectory_YYYYMMDD_HHMMSS.txt
```

---

## P1：錄製 rosbag，定位掉幀發生位置

錄製：

```text
infra1 或 color
depth
camera_info
imu
```

比較：

```text
bag 內 timestamp 是否已缺幀
online wrapper 是否額外丟幀
關閉 Viewer 是否改善
降低 debug log 是否改善
不同 QoS 是否改善
不同 executor 是否改善
```

這是處理 `[RGB INPUT GAP]`、`[DEPTH INPUT GAP]` 與 `[RGBD PAIR REJECTED]` 的必要步驟。

---

## P1：決定最終 SLAM 影像來源

需要明確選擇：

### 方案 A：infra 作為 SLAM 輸入

```text
infra1 + depth + IMU
```

優點：

- 已有穩定基準。
- 風險最低。
- 彩色影像可額外同步收集。

### 方案 B：color 作為 SLAM 輸入

```text
color + depth + IMU
```

需要重新驗證：

- intrinsics
- depth alignment
- timestamp
- mask
- exposure
- inliers
- motion blur
- CPU / USB bandwidth

建議建立新設定檔：

```text
D405_color_depth_inertial.yaml
```

不要覆寫：

```text
D405_rgbd_inertial_test.yaml
```

---

## P1：重做最終 Kalibr

在最終 image topic 決定後進行：

```text
Camera intrinsics
Camera–IMU extrinsic
Camera–IMU time offset
IMU noise density
IMU random walk
```

要求：

```text
相機與 IMU 安裝完全固定
使用目前 ROS 2 driver 錄製
使用最終 FPS / frequency
使用最終 topic 與 timestamp pipeline
```

舊 calibration 可以做初始值，但不能視為最終真值。

---

## P2：修正初始化 state

修正：

```text
avgA_last
mbHasLastInitAvgAcc
active map reset
IMU preintegration reset
lastImuMsg_ reset
```

目的：

- 移除啟動時巨大垃圾值。
- 避免 reset 後初始化狀態殘留。
- 降低 reset loop 的可能性。

---

## P2：檢查 preintegration 時間一致性

每 frame 比較：

```text
image_dt
preintegrated_dT
```

正常應接近：

```text
abs(image_dt - preintegrated_dT) ≈ 0
```

允許誤差需依實際插值與 timestamp 精度判定，不應先硬寫一個未驗證門檻。

---

## P2：定量精度測試

trajectory export 完成後：

```text
靜止 1 分鐘
閉環回到起點
固定路徑重複 5 次
不同曝光重複測試
一般速度與高速測試
Atlas save/load 測試
evo APE
evo RPE
```

---

## 11. 建議開發順序

```text
封存目前 infra 穩定版
    ↓
加入 trajectory / pose / odom / TF
    ↓
錄 rosbag 定位 RGB-depth gap
    ↓
決定 SLAM 使用 infra 或 color
    ↓
用最終資料鏈重做 Kalibr
    ↓
修正 avgA_last 與 reset state
    ↓
檢查 preintegration dT
    ↓
執行 ATE / RPE 與長時間測試
    ↓
驗證 Atlas save/load
```

### 為什麼 trajectory 應先做

trajectory 輸出完成後，後續才能客觀比較：

- 舊 Kalibr 與新 Kalibr
- infra 與 color
- 不同曝光
- 不同同步策略
- 不同初始化方式

否則只能依 Viewer 與 log 做主觀判斷。

---

## 12. 目前是否可以直接使用

### 可以開始使用的範圍

✅ 受控環境中低速建圖  
✅ ORB-SLAM3 功能測試  
✅ 回環與追蹤穩定性測試  
✅ rosbag 資料收集  
✅ trajectory 輸出功能開發  
✅ 其他 ROS 2 模組的前期整合

### 尚未建議直接部署的範圍

📌 需要可量化精度保證的定位  
📌 長時間無人值守運行  
📌 高速或劇烈旋轉  
📌 直接接 Nav2 作為唯一定位源  
📌 依賴尚未驗證的 Atlas save/load  
📌 需要彩色 SLAM 但仍沿用 infra calibration

目前版本最準確的定位是：

> **可用的研究原型與穩定基準版，而不是已完成驗證的部署版定位模組。**

---

## 13. 使用前檢查清單

```text
[ ] ROS 2 Humble 已 source
[ ] wrapper workspace 已 source
[ ] infra1 約 30 Hz
[ ] depth 約 30 Hz
[ ] IMU 約 100 Hz
[ ] auto exposure 已關閉
[ ] exposure / gain 已確認
[ ] 使用正確 YAML
[ ] imu_time_offset_sec = 0.0
[ ] 從空 Atlas 或載入已知唯讀 Atlas
[ ] log 路徑已建立
[ ] 初始化時有足夠平移與旋轉
[ ] 已看到 end VIBA 1
[ ] 已看到 end VIBA 2
[ ] inliers 沒有長時間低於 100
[ ] 沒有連續 Fail to track local map!
[ ] 測試結束後已保存 log
```

---

## 14. 待補資訊

正式發布此 README 前，還需要補：

```text
[ ] RealSense 完整 launch 指令
[ ] Xsens 完整 launch 指令
[ ] RealSense ROS driver 版本
[ ] Xsens driver 版本
[ ] ORB_SLAM3 commit hash
[ ] ORB_SLAM3_ROS2 commit hash
[ ] Pangolin 版本
[ ] OpenCV 版本
[ ] Eigen 版本
[ ] Sophus 版本
[ ] 乾淨 Ubuntu 的完整安裝步驟
[ ] color topic 與 camera_info
[ ] 最終 Kalibr 檔案
[ ] trajectory 輸出格式
[ ] pose / odom / TF frame 定義
[ ] Atlas save/load 行為
```

---

## 15. 簡要總結

目前已完成的核心成果：

```text
D405 infra1 + depth + Xsens IMU
可在 ROS 2 Humble 上執行 ORB-SLAM3 RGB-D-Inertial
已修正外參方向與 IMU period
已加入 timestamp offset 與同步診斷
已排除 Atlas、prediction、±10 ms offset 為單一根因
已完成曝光 A/B
已確認靜止後可重新收斂
```

目前最重要的剩餘工作：

```text
trajectory / pose / odom / TF
RGB-depth 掉幀定位
最終影像來源決策
重新 Kalibr
初始化 state 修正
定量精度評估
```
