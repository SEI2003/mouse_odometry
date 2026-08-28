# mouse_odometry

2台のPMW3901オプティカルフローセンサの微小移動量から、車体中心の平面運動 `Δx / Δy / Δyaw` を推定し、ROS 2 `nav_msgs/msg/Odometry` を `/mouse_odom` に出力するパッケージです。

## 現在の入力境界

このリポジトリを調査した時点では、PMW3901、MCU、Serial/UART、SPIのデータ取得コードや通信仕様は存在せず、Linux `/dev/input/event*` からUSBマウスを読むコードだけが存在していました。センサの実通信仕様を推測しないため、本パッケージはUART/SPIドライバを実装していません。

代わりに、入力取得と運動推定を次のように分離しています。

```text
既存または今後のPMW3901ドライバ / MCUブリッジ
  -> /pmw3901/left/flow, /pmw3901/right/flow
  -> countの軸別校正・取付yaw補正
  -> 2センサ最小二乗推定
  -> /mouse_odom
```

入力型は `mouse_odometry/msg/Pmw3901Flow` です。

```text
std_msgs/Header header                 # 積算区間の終了時刻
int32 delta_x_count                    # PMW3901センサX軸のraw差分
int32 delta_y_count                    # PMW3901センサY軸のraw差分
builtin_interfaces/Duration integration_time
bool valid                             # 取得側が判断した観測の有効性
bool quality_available                 # qualityを取得できたか
uint16 quality                         # SQUAL等（取得できる場合のみ）
```

`delta_*_count` は、直前のメッセージからの累積値ではなく、`integration_time` の区間だけで生じた差分です。`header.stamp` はその区間の終了時刻にしてください。2センサの積算区間を揃える責任は入力アダプタ側にもあります。header stampがゼロの場合はROS受信時刻を使いますが、正確な同期とreset後の古いキューデータ除去には送信元timestampが必要です。

PMW3901のX/Y軸や符号はモジュール・取付方法・既存ファームウェアに依存するため、このREADMEでは決め打ちしません。実際の出力仕様を確認して校正値と取付角を設定してください。

## 座標系とセンサ配置

ROS標準の車体座標系を使います。

```text
+x: 車両前方
+y: 車両左方向
+z: 上方向
+yaw: 上から見て反時計回り
```

デフォルト位置は次のとおりです。

```text
left:  (x, y, z) = (0.220, +0.055, 0.282) m
right: (x, y, z) = (0.220, -0.055, 0.282) m
```

`left_sensor_z` と `right_sensor_z` は車体のodom基準点から見た取付位置です。一方、`sensor_height_from_ground` はセンサと実際の路面との距離を記録する校正条件で、同じ値とは限りません。現在の推定式はzを使わず、`sensor_height_from_ground` による推測的な自動スケール補正式も実装していません。

## countから車体座標の移動量へ

USBマウスのCPI式は使用しません。各センサ・各軸を独立に実測校正します。

```text
dx_sensor = delta_x_count * x_meter_per_count
dy_sensor = delta_y_count * y_meter_per_count
```

負の `meter_per_count` も設定できるため、確認済みの軸符号を表現できます。その後、取付yaw `alpha` で車体座標へ回転します。

```text
dx_body_sensor = cos(alpha) * dx_sensor - sin(alpha) * dy_sensor
dy_body_sensor = sin(alpha) * dx_sensor + cos(alpha) * dy_sensor
```

デフォルトの `1.0e-4 m/count` はビルド・接続確認用の仮値であり、正確な値ではありません。実機利用前に4軸すべてを校正してください。

## 平面運動推定

位置 `(x_i, y_i)` の各センサ観測を、微小剛体運動モデル

```text
dx_i = dx - y_i * dtheta
dy_i = dy + x_i * dtheta
```

に当てはめます。実装は対称配置専用式ではなく、次の一般行列をEigenの列ピボット付きQR分解で最小二乗します。

```text
z = A u

z = [left_dx, left_dy, right_dx, right_dy]^T
u = [dx, dy, dtheta]^T

A = [1  0  -left_y ]
    [0  1   left_x ]
    [1  0  -right_y]
    [0  1   right_x]

u = argmin ||A u - z||^2
residual = ||A u - z||
```

行列rankが3未満になるセンサ配置では起動を拒否します。デフォルト配置では、純粋yawにより両センサへ生じる同方向の `dy = 0.220 * dtheta` もモデル内で除去されます。

受理した増分は中間yawでodom座標へ積算します。

```text
yaw_mid = yaw + dtheta / 2
odom_x += dx * cos(yaw_mid) - dy * sin(yaw_mid)
odom_y += dx * sin(yaw_mid) + dy * cos(yaw_mid)
yaw = normalize(yaw + dtheta)
```

速度 `vx=dx/dt`, `vy=dy/dt`, `wz=dtheta/dt` はchild frame（車体座標）基準で格納します。

## 有効性、同期、外れ値

両方の新しい観測が揃った場合だけ3DoF推定します。片方が停止・invalidの場合、その値をゼロ移動とは解釈せず、積算とOdometry publishを停止してWARNを出します。片側だけからyawを推定するfallbackはありません。

観測は次の場合に棄却されます。

- `valid == false`
- qualityが利用可能で `quality < minimum_quality`
- `integration_time <= 0`
- ROS受信から `sensor_timeout_sec` を超過
- 左右timestamp差が `max_sensor_time_difference_sec` を超過
- 左右積算時間差が `max_sensor_interval_difference_sec` を超過
- 最小二乗残差が `max_motion_residual` を超過
- `abs(vx)`, `abs(vy)`, `abs(wz)` が各速度上限を超過

qualityが通信に存在しない場合は `quality_available=false` とし、値を捏造しないでください。その観測にはquality filterを適用しません。

## ROSインターフェース

Published topics:

| Topic | Type | 内容 |
| --- | --- | --- |
| `/mouse_odom` | `nav_msgs/msg/Odometry` | 受理済み観測だけを積算したodom。TFはpublishしない |
| `/mouse_odom/debug` | `mouse_odometry/msg/Pmw3901Debug` | raw count、変換後[m]、推定増分、residual、quality、valid、棄却理由 |

Subscribed topics:

| Default topic | Type |
| --- | --- |
| `/pmw3901/left/flow` | `mouse_odometry/msg/Pmw3901Flow` |
| `/pmw3901/right/flow` | `mouse_odometry/msg/Pmw3901Flow` |

Service:

| Service | Type | 内容 |
| --- | --- | --- |
| `/reset_mouse_odom` | `std_srvs/srv/Empty` | mutex内でx/y/yawと未処理の左右deltaをクリア |

reset時刻以前のsource timestampを持つ遅延メッセージも棄却します。TFは意図的にpublishしません。

`/mouse_odom` のデフォルトframeは `mouse_odom`、child frameは `mouse_base_link` です。poseは親frame、twistはchild frame基準です。共分散は現時点では固定の暫定値です。residualやqualityから動的共分散へ発展させる境界は一箇所にまとめていますが、未校正のモデルは追加していません。

## Parameters

| Parameter | Default | 内容 |
| --- | ---: | --- |
| `left_sensor_x/y/z` | `0.220 / 0.055 / 0.282` | 左センサ位置[m] |
| `right_sensor_x/y/z` | `0.220 / -0.055 / 0.282` | 右センサ位置[m] |
| `left_sensor_yaw` | `0.0` | 左センサ軸から車体軸への回転[rad] |
| `right_sensor_yaw` | `0.0` | 右センサ軸から車体軸への回転[rad] |
| `left_x_meter_per_count` | `1.0e-4` | 左Xの仮換算値[m/count] |
| `left_y_meter_per_count` | `1.0e-4` | 左Yの仮換算値[m/count] |
| `right_x_meter_per_count` | `1.0e-4` | 右Xの仮換算値[m/count] |
| `right_y_meter_per_count` | `1.0e-4` | 右Yの仮換算値[m/count] |
| `sensor_height_from_ground` | `0.0` | 校正時の対地高さ[m]。情報保持のみ |
| `minimum_quality` | `0` | qualityが存在する場合の下限 |
| `sensor_timeout_sec` | `0.1` | ROS受信watchdog[s] |
| `max_sensor_time_difference_sec` | `0.02` | 左右終了timestampの最大差[s] |
| `max_sensor_interval_difference_sec` | `0.02` | 左右積算時間の最大差[s] |
| `max_linear_speed` | `5.0` | 車体+x速度上限[m/s] |
| `max_lateral_speed` | `5.0` | 車体+y速度上限[m/s] |
| `max_angular_speed` | `10.0` | yaw角速度上限[rad/s] |
| `max_motion_residual` | `0.01` | 4観測の最小二乗残差上限[m] |
| `left_flow_topic` | `/pmw3901/left/flow` | 左入力topic |
| `right_flow_topic` | `/pmw3901/right/flow` | 右入力topic |
| `frame_id` | `mouse_odom` | Odometry親frame |
| `child_frame_id` | `mouse_base_link` | Odometry子frame |

速度・residual・quality閾値も、実測データから調整する必要があります。

## ビルドと起動

ROS 2 Humbleの例:

```bash
cd ~/mouse_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mouse_odometry --symlink-install
source install/setup.bash
ros2 run mouse_odometry mouse_odom_node --ros-args \
  -p left_x_meter_per_count:=<calibrated_value> \
  -p left_y_meter_per_count:=<calibrated_value> \
  -p right_x_meter_per_count:=<calibrated_value> \
  -p right_y_meter_per_count:=<calibrated_value> \
  -p sensor_height_from_ground:=<calibration_height>
```

これだけではPMW3901ハードウェアから値は届きません。実機の既存MCU/ドライバ仕様に従う入力アダプタが、左右の `Pmw3901Flow` をpublishする必要があります。

確認:

```bash
ros2 topic echo /mouse_odom
ros2 topic echo /mouse_odom/debug
ros2 service call /reset_mouse_odom std_srvs/srv/Empty "{}"
```

## 実機キャリブレーション

校正中は路面、照明、高さ、速度を実運用条件に固定し、raw countを `/mouse_odom/debug` または入力側ログで記録します。

1. X方向: yawを変えず、正確に例えば1.0 m前進します。左右各センサについて `実距離 / 累積X count` を計算し、各 `x_meter_per_count` に設定します。符号もこの試験で確定します。
2. Y方向: 可能ならyawを変えず、既知距離だけ横移動します。左右各々の `実距離 / 累積Y count` を設定します。車体を正確に横移動できない場合、純粋なY校正は困難なので、治具やセンサ単体の直線ステージを使用してください。斜め移動をYだけの校正値へ混ぜないでください。
3. 取付角: 車体軸に沿った直線移動で他方の軸へ現れる成分を確認し、`left_sensor_yaw` と `right_sensor_yaw` を調整します。軸の取り違えはyaw角だけでなく入力アダプタの実仕様と照合します。
4. yaw: 90/180/360 degなど既知角度だけ回し、推定yawを比較します。yawだけを合わせるため実測したセンサ間距離を恣意的に変更せず、センサ位置、左右各軸scale、取付角、flowの追跡状態を切り分けます。
5. 高さ依存: 同じ校正高さを `sensor_height_from_ground` に記録します。高さを変える場合は再校正してください。距離センサを将来追加するまでは自動補正されません。

## 数値テスト

`test/test_planar_motion_estimator.cpp` に次を実装しています。

| Test | 真値 `(dx, dy, dtheta)` | 理論観測 `(left_dx,left_dy,right_dx,right_dy)` | 期待結果 |
| --- | --- | --- | --- |
| A 直進 | `(0.100, 0, 0)` | `(0.100,0,0.100,0)` | `(0.100,0,0)` |
| B 横滑り | `(0, 0.100, 0)` | `(0,0.100,0,0.100)` | `(0,0.100,0)` |
| C 純粋yaw | `(0,0,0.100)` | `(-0.0055,0.022,+0.0055,0.022)` | `(0,0,0.100)` |
| D 複合 | `(0.080,0.020,0.050)` | `(0.07725,0.031,0.08275,0.031)` | 元の真値 |

実行:

```bash
cd ~/mouse_ws
colcon test --packages-select mouse_odometry
colcon test-result --verbose
```

## 前提と限界

PMW3901はオプティカルフローセンサであり、raw countは絶対的な実距離ではありません。精度は路面模様、センサと路面の距離、照明、振動、pitch/roll、個体差、速度、tracking lossに依存します。現在の2D odometryは平坦路面かつセンサ高さがほぼ一定という前提です。

未解決なのは実機固有のPMW3901入力アダプタと通信仕様、4軸の実測scale、センサ軸の符号・取付角、qualityの実際の範囲、運用環境に適した閾値です。これらはリポジトリ内に根拠となるファームウェアや仕様がなかったため推測していません。
