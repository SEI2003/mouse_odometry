# mouse_odometry

2つの光学マウスを用いて、ROS 2 の `nav_msgs/msg/Odometry` として2次元オドメトリを出力するパッケージです。

マウスの移動量を `/dev/input` から直接読み取り、2台の平均移動量から並進速度を計算します。
また、2つのマウスを左右方向に一定距離離して設置することで、左右の移動量差からyaw角速度も推定します。

## Features

* 2つのマウスから移動量を取得
* `/dev/input/by-id/` からマウスデバイスを自動検出
* `nav_msgs/msg/Odometry` を `/mouse_odom` に配信
* マウス間距離からyaw角を推定
* ROSサービスで位置・姿勢をリセット可能
* `input` グループにユーザーを追加することで `sudo` なしで実行可能

## Environment

* Ubuntu 22.04
* ROS 2 Humble
* C++17
* Linux input event device

## ROS Interfaces

### Published Topics

| Topic         | Type                    | Description    |
| ------------- | ----------------------- | -------------- |
| `/mouse_odom` | `nav_msgs/msg/Odometry` | マウスから計算したオドメトリ |

### Services

| Service             | Type                 | Description    |
| ------------------- | -------------------- | -------------- |
| `/reset_mouse_odom` | `std_srvs/srv/Empty` | 位置とyaw角を0にリセット |

## Coordinate Definition

このノードでは、以下の座標系を前提としています。

```text
ROS座標系:
  x : 前方向
  y : 左方向
  z : 上方向

yaw:
  反時計回りが正
```

マウスの入力は以下のように扱います。

```text
REL_Y : ロボットの前後方向
REL_X : ロボットの左右方向
```

2つのマウスを左右方向に離して取り付けた場合、yaw変化量は以下で計算します。

```text
delta_yaw = (right_forward - left_forward) / mouse_separation_m
```

デフォルトのマウス間距離は `0.135 m` です。

## Device Layout

推奨配置は以下です。

```text
ロボット前方
    ↑

[左マウス] ---- 13.5 cm ---- [右マウス]

    ↓
ロボット後方
```

デフォルトでは、以下の前提になっています。

```text
mouse1_is_left = true
```

つまり、

```text
mouse1 : 左側
mouse2 : 右側
```

として扱います。

左右が逆の場合は、起動時に次のパラメータを変更してください。

```bash
-p mouse1_is_left:=false
```

## Installation

ワークスペースを作成します。

```bash
mkdir -p ~/mouse_ws/src
cd ~/mouse_ws/src
```

このリポジトリを配置します。

```bash
git clone <your_repository_url>
```

ビルドします。

```bash
cd ~/mouse_ws
colcon build --packages-select mouse_odometry --symlink-install
source install/setup.bash
```

## Device Permission

マウスは `/dev/input/eventXX` として認識されます。
通常ユーザーのままだと、以下のように `Permission denied` が出る場合があります。

```text
open failed: Permission denied
```

一時的に動かすだけなら、対象デバイスに読み取り権限を付与します。

```bash
sudo chmod a+r /dev/input/event16
sudo chmod a+r /dev/input/event18
```

ただし、`event16` や `event18` はUSBの抜き差しや再起動で変わることがあります。

恒久的に `sudo` なしで扱う場合は、ユーザーを `input` グループに追加します。

```bash
sudo usermod -aG input $USER
```

設定を反映するために、一度ログアウトして再ログインします。
またはPCを再起動します。

反映されているか確認します。

```bash
groups
```

出力に `input` が含まれていればOKです。

```text
user25 adm cdrom sudo dip plugdev input lpadmin sambashare
```

その後は、`sudo` なしでノードを起動できます。

```bash
ros2 run mouse_odometry mouse_odom_node
```

## Mouse Device Check

接続されている入力デバイスを確認します。

```bash
ls -l /dev/input/by-id/
```

例：

```text
usb-Logitech_USB_Optical_Mouse-event-mouse -> ../event18
usb-Logitech_USB_Receiver-if01-event-mouse -> ../event16
usb-Logitech_USB_Receiver-if01-event-kbd -> ../event15
```

このノードは `/dev/input/by-id/` 内の `event-mouse` で終わるデバイスだけを自動検出します。
そのため、`event-kbd` のようなキーボードデバイスは除外されます。

## Usage

通常起動：

```bash
ros2 run mouse_odometry mouse_odom_node
```

マウス間距離を指定して起動：

```bash
ros2 run mouse_odometry mouse_odom_node --ros-args \
  -p mouse_separation_m:=0.135
```

yawの符号が逆の場合：

```bash
ros2 run mouse_odometry mouse_odom_node --ros-args \
  -p yaw_sign:=-1.0
```

マウス1とマウス2の左右が逆の場合：

```bash
ros2 run mouse_odometry mouse_odom_node --ros-args \
  -p mouse1_is_left:=false
```

手動でデバイスを指定する場合：

```bash
ros2 run mouse_odometry mouse_odom_node --ros-args \
  -p auto_detect_devices:=false \
  -p device_path_1:=/dev/input/by-id/usb-Logitech_USB_Optical_Mouse-event-mouse \
  -p device_path_2:=/dev/input/by-id/usb-Logitech_USB_Receiver-if01-event-mouse
```

## Parameters

| Parameter             |           Default | Description                    |
| --------------------- | ----------------: | ------------------------------ |
| `auto_detect_devices` |            `true` | `/dev/input/by-id/` からマウスを自動検出 |
| `device_path_1`       |              `""` | マウス1のデバイスパス                    |
| `device_path_2`       |              `""` | マウス2のデバイスパス                    |
| `cpi`                 |          `1000.0` | マウスのCPI                        |
| `x_scale`             |             `1.0` | 左右方向のスケール補正                    |
| `y_scale`             |            `0.92` | 前後方向のスケール補正                    |
| `mouse_separation_m`  |           `0.135` | 左右マウス間の距離[m]                   |
| `mouse1_is_left`      |            `true` | `true` の場合、マウス1を左側として扱う        |
| `yaw_sign`            |             `1.0` | yaw方向の符号補正                     |
| `publish_rate`        |            `10.0` | `/mouse_odom` の配信周期[Hz]        |
| `frame_id`            |      `mouse_odom` | Odometryの親フレーム                 |
| `child_frame_id`      | `mouse_base_link` | Odometryの子フレーム                 |
| `grab_device`         |            `true` | 入力デバイスを排他的に取得する                |

## Topic Check

トピックが出ているか確認します。

```bash
ros2 topic list | grep mouse
```

Odometryの中身を確認します。

```bash
ros2 topic echo /mouse_odom
```

配信周期を確認します。

```bash
ros2 topic hz /mouse_odom
```

## Reset Odometry

位置とyaw角をリセットします。

```bash
ros2 service call /reset_mouse_odom std_srvs/srv/Empty "{}"
```

## Calibration

### 1. 前進方向の確認

ロボットをまっすぐ前に動かします。

期待される値：

```text
linear.x > 0
linear.y ≒ 0
angular.z ≒ 0
```

もし `linear.x` が負になる場合は、`y_scale` の符号を反転します。

```bash
-p y_scale:=-0.92
```

### 2. 左右方向の確認

ロボットを真横に動かします。

期待される値：

```text
linear.y が変化
linear.x ≒ 0
angular.z ≒ 0
```

左右の符号が逆の場合は、`x_scale` の符号を反転します。

```bash
-p x_scale:=-1.0
```

### 3. yaw方向の確認

ロボットをその場で反時計回りに回転させます。

期待される値：

```text
angular.z > 0
```

もし `angular.z` が負になる場合は、`yaw_sign` を反転します。

```bash
-p yaw_sign:=-1.0
```

### 4. マウス間距離の補正

yaw角が実際より大きく出る場合は、`mouse_separation_m` を大きくします。

```bash
-p mouse_separation_m:=0.145
```

yaw角が実際より小さく出る場合は、`mouse_separation_m` を小さくします。

```bash
-p mouse_separation_m:=0.125
```

## Example Output

`/mouse_odom` の例です。

```yaml
header:
  frame_id: mouse_odom
child_frame_id: mouse_base_link
pose:
  pose:
    position:
      x: -0.129
      y: 0.077
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.351
      w: 0.936
twist:
  twist:
    linear:
      x: 0.000
      y: 0.000
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
```

## Notes

このノードは光学マウスの移動量をそのまま積算するため、床面の材質やマウスの接地状態に影響されます。

精度を上げるには、以下の点に注意してください。

* マウスを床面に安定して接地させる
* 2つのマウスの向きをそろえる
* マウス間距離を正確に測る
* 滑りやすい床面を避ける
* 実測距離に合わせて `x_scale` と `y_scale` を調整する
* 実測角度に合わせて `mouse_separation_m` または `yaw_sign` を調整する

## License

Apache-2.0
