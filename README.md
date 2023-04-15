# 0. はじめに

haya_imu_rosは、9軸IMU/AHRS haya_imu v3の専用ROSパッケージです。主なフィーチャとして、通常出力モード、デモンストレーションモード、キャリブレーションモード、6軸回転クォータニオン、9軸回転クォータニオン、オイラー角を同時にパブリッシュ(Max1000Hz)することと、RVIZにてのデモンストレーションが可能となります。

# 1. 対向環境

- Ubuntu 20.04 18.04 16.04 推奨
- ROS noetic melodic kinetic 推奨

# 2. 使用手順

## 2.1 haya_imu_rosのインストール

$cd ~/catkin_ws/src

$git clone https://github.com/soarbear/haya_imu_ros.git

$cd ~/catkin_ws

$catkin_make --only-pkg-with-deps haya_imu_ros

## 2.2 USBでの接続

haya_imuを対向装置へUSBで接続できた場合、赤LEDが常時点灯することを確認できます。

## 2.3 デバイス名の固定

- デバイス名を固定する場合、

$chmod +x ~/catkin_ws/src/haya_imu_ros/script/create_rules.sh

$~/catkin_ws/src/haya_imu_ros/script/create_rules.sh

$udevadm control --reload-rules && udevadm trigger

- また、固定したデバイス名を解除する場合、

$chmod +x ~/catkin_ws/src/haya_imu_ros/script/delete_rules.sh

$~/catkin_ws/src/haya_imu_ros/script/delete_rules.sh

$udevadm control --reload-rules && udevadm trigger

## 2.4 パラメータの確認

params.yamlに載ってあるパラメータの値を確認して、必要に応じて変更します。

## 2.5 ROSローンチ

- 通常出力モード、キャリブレーションモードの場合、

$roslaunch haya_imu_ros haya_imu.launch

- デモンストレーションモードの場合、

$roslaunch haya_imu_ros haya_imu_demo.launch

![alt text](https://github.com/soarbear/haya_imu_ros/blob/main/image/demo_fusion.jpg)

## 2.6 Topic、tfの確認

- imu_data(Message: haya_imu_ros/ImuData)、通常出力モード、キャリブレーションモード用 

- tf(Message: geometry_msgs/TransformStamped)、デモンストレーションモード用

- Topicのデータを確認する例

$rostopic echo imu_data

- Topicの出力レートを確認する例

$rostopic hz -w 100 imu_data

# 3. リリース

- v3.2 April 2023 新規リリース

# 4. ライセンス

- 本ROSパッケージ(haya_imu_ros)に対して、BSD-3-Clauseが適用されます。
