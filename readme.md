# mouse_odometry

2台のPMW3901オプティカルフローセンサの微小移動量から、車体中心の平面運動 `Δx / Δy / Δyaw` を推定し、ROS 2 `nav_msgs/msg/Odometry` を `/mouse_odom` に出力するパッケージです。

## 入力境界

入力取得と運動推定は次のように分離しています。`pmw3901_serial_bridge` がXIAO ESP32-C3のbinary packet v1をdecodeし、既存の`mouse_odom_node`へ左右のraw flowを渡します。平面運動推定、health判定、最小二乗、Odometry積算はBridgeに重複実装していません。

```text
PMW3901 x2 -> XIAO ESP32-C3 -> USB Serial binary v1
  -> pmw3901_serial_bridge
  -> /pmw3901/left/flow, /pmw3901/right/flow
  -> countの軸別校正・取付yaw補正
  -> 2センサ最小二乗推定
  -> /mouse_odom
```

入力型は `mouse_odometry/msg/Pmw3901Flow` です。

```text
std_msgs/Header header                 # 積算区間の終了時刻
uint32 cycle_id                        # ESP32 sampling cycle ID
int32 delta_x_count                    # PMW3901センサX軸のraw差分
int32 delta_y_count                    # PMW3901センサY軸のraw差分
builtin_interfaces/Duration integration_time
bool valid                             # 取得側が判断した観測の有効性
bool quality_available                 # qualityを取得できたか
uint16 quality                         # SQUAL等（取得できる場合のみ）
```

`delta_*_count` は、直前のメッセージからの累積値ではなく、`integration_time` の区間だけで生じた差分です。`header.stamp` の定義は **measurement end timestamp（積算区間終了時刻）** であり、measurement start、MCU送信時刻、ROS受信時刻ではありません。2センサの積算区間を揃える責任は入力アダプタ側にもあります。

他の入力アダプタでmeasurement endを取得できない場合だけ、header stampをゼロにしてください。`pmw3901_serial_bridge`が正常同期している場合は、ESP measurement timestampをROS時間へ変換した非ゼロstampを必ず設定します。

PMW3901のX/Y軸や符号はモジュール・取付方法・既存ファームウェアに依存するため、このREADMEでは決め打ちしません。実際の出力仕様を確認して校正値と取付角を設定してください。

## PMW3901 Serial Bridge

### Serialとpacket v1

```bash
ros2 run mouse_odometry pmw3901_serial_bridge --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p serial_baud:=115200
```

外部Serial libraryは使用せず、Ubuntu 22.04のPOSIX `termios`でdeviceをraw、8N1、non-blockingに設定します。ROS wall timerが既定2 ms周期で`poll()`/`read()`し、node executorをblockしません。対応baudは9600、19200、38400、57600、115200と、OSが定義する場合の230400、460800、921600です。USB CDCではbaud設定が物理転送速度を直接決めない場合があります。

wire packetは固定33 bytes、全multibyte fieldがlittle endianです。

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | header `0xAA55`（wire `55 AA`） |
| 2 | 1 | version `1` |
| 3 | 4 | `cycle_id` |
| 7 | 8 | ESP monotonic `timestamp_us` |
| 15 | 4 | `measurement_interval_us` |
| 19 / 21 / 23 / 24 | 2 / 2 / 1 / 1 | left dx / dy / SQUAL / status |
| 25 / 27 / 29 / 30 | 2 / 2 / 1 / 1 | right dx / dy / SQUAL / status |
| 31 | 2 | CRC |

CRCはCRC-16/CCITT-FALSE（poly `0x1021`、init `0xFFFF`、RefIn/RefOut false、XorOut `0x0000`、`"123456789" -> 0x29B1`）です。offset 0〜30の31 bytesを計算し、末尾へlittle endianで格納します。CRC不一致packetはpublishしません。

### Byte stream parserと再同期

1回の`read()`とpacket境界は無関係です。内部bufferへ任意長のbyte列を追加し、`55 AA`を検索して33 bytesが揃うまで保持します。versionとCRCが正常なら`decodePacketV1()`で明示的にlittle endian decodeします。version/CRC失敗時はbuffer全体を捨てず、現在のheader候補から1 byteだけ進めて次のheaderを再検索します。garbage、packet分割、複数packet同時受信、破損packet直後の正常packetに対応します。

### ESP timestampからROS timestampへの変換

ESPの`timestamp_us`は`esp_timer_get_time()`由来の起動後monotonic時刻で、Unix/ROS時刻ではありません。CRC正常な初期packetからoffset候補を集めます。

```text
offset_i = ros_receive_time_i - esp_timestamp_i
offset = minimum(offset_i)
measurement_ros_time = esp_timestamp + offset
```

既定では最初の20 packetについて候補を集め、そのminimumを固定offsetとします。USB伝送遅延は正方向へ加わるため、最小候補を選ぶことで初回1 packet固定よりjitterの影響を抑えます。同期完了まではflowをpublishしません。以後の左右`Pmw3901Flow.header.stamp`には毎回のPC受信時刻ではなく、この変換済みmeasurement-end時刻を共通設定します。debugの`transport_delay_estimate_sec = receive_time - converted_measurement_time`はminimum候補を基準にした追加遅延で、絶対的なone-way USB delayではありません。

`use_sim_time=true`でROS clockがまだ0の場合は同期せずpublishを抑止します。ROS clockが後退した場合は同期epochを作り直します。ESP timestampが前回より小さくなった場合はESP再起動としてclock offsetをresetし、そのpacketから新しいoffsetを確立します。cycle IDだけの`0xFFFFFFFF -> 0`はunsigned差分で連続と判断し、再起動にしません。

### Drop、status、integration time

連続する正常packetの`uint32_t cycle_id`差が2以上なら`difference - 1`をdrop数へ加算します。重複cycleと大きな逆行cycleはOdometryの二重積算を防ぐためpublishしません。

```text
quality_available = status & 0x20
valid = (status & 0x01) && !(status & (0x08 | 0x10 | 0x40))
```

`SPI_ERROR`、`INIT_ERROR`、`OBSERVATION_ERROR`のいずれかがあれば、誤って`VALID`も立っていても安全側でfalseにします。SQUAL byteは`uint16 quality`へ拡張します。`measurement_interval_us`は秒とnanosecondへ分解し、左右に同じ`builtin_interfaces/Duration`を設定します。`measurement_interval_us=0` packetは、clock同期済みでpublish対象になった場合も左右`valid=false`とするため、既存validationでは積算されません。既定設定では最初の20 packetをclock同期に使い、その間はflow自体をpublishしません。

packet v1では左右を同じcycleとして扱い、同じ`cycle_id`、measurement-end timestamp、integration timeを設定します。cycle IDはBridge内のdrop・重複・逆行検出に加え、`mouse_odom_node`のpair validationでも一致を必須とします。平面運動推定ロジック自体は変更しません。ただしESPではSPI逐次読出しのため、実際には微小なread時刻差があります。v1 packetにはそのoffsetが含まれていません。

### Reconnectとdebug

open失敗、`poll()` hangup、read errorではnodeを終了せずfd、parserの未処理byte、session/clock同期状態を閉じ、既定1秒間隔で再openします。再接続後は正常packetからoffset候補を集め直し、既定20 packetでclock offsetを再構築します。

`/pmw3901/serial_debug`（`Pmw3901SerialDebug`）には接続状態、parser/CRC/version/resync/drop/duplicate統計、reconnect/ESP reboot数、現在のcycle/timestamp/interval/status、clock offset、同期状態、最終ROS受信時刻、変換済みmeasurement時刻、transport delay estimateを既定1 Hzでpublishします。

| Parameter | Default | 内容 |
| --- | ---: | --- |
| `serial_port` | `/dev/ttyACM0` | XIAO USB Serial device |
| `serial_baud` | `115200` | termios baud設定 |
| `reconnect_interval_sec` | `1.0` | 再open周期 |
| `io_poll_period_ms` | `2` | non-blocking I/O timer周期 |
| `debug_publish_period_sec` | `1.0` | Serial debug publish周期 |
| `clock_sync_sample_count` | `20` | minimum offset決定に使う初期packet数 |
| `left_flow_topic` | `/pmw3901/left/flow` | 左publish topic |
| `right_flow_topic` | `/pmw3901/right/flow` | 右publish topic |
| `left_frame_id` | `pmw3901_left` | 左message frame |
| `right_frame_id` | `pmw3901_right` | 右message frame |

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
motion_residual = ||A u - z||
```

行列rankが3未満になるセンサ配置では起動を拒否します。デフォルト配置では、純粋yawにより両センサへ生じる同方向の `dy = 0.220 * dtheta` もモデル内で除去されます。

### 2センサ構成の観測限界

`motion_residual = ||Au-z||` は「観測がこの剛体運動モデルで説明できるか」の指標であり、完全なsensor healthではありません。今回の4観測・3未知数の配置では、左右Xの差が並進とyawの組み合わせとして正確に説明される場合があります。

例えば `left_dx=0.101, right_dx=0.100, left_dy=right_dy=0` は、実際には左X scaleが1%ずれた直進であっても、推定器にはyawを含む剛体運動として見え、`motion_residual` はほぼゼロになります。片側X tracking errorでも同様です。したがって、次は誤った判断です。

```text
motion_residualが小さい = 左右PMW3901が正常
```

2センサの観測だけでは、この種の異常と実際のyawを理論的に一意に識別できません。SQUAL、timeout、measurement timestamp、integration time、raw delta sanity checkを独立に使い、将来はIMU gyro zとの比較も行う必要があります。今回、IMU融合やIMU dependencyは追加していません。

受理した増分は中間yawでodom座標へ積算します。

```text
yaw_mid = yaw + dtheta / 2
odom_x += dx * cos(yaw_mid) - dy * sin(yaw_mid)
odom_y += dx * sin(yaw_mid) + dy * cos(yaw_mid)
yaw = normalize(yaw + dtheta)
```

pose積算には推定deltaそのものを使います。速度の `dt` だけはROS callback間隔ではなく、左右が整合したmeasurement integration timeの平均

```text
dt_motion = (left.integration_time + right.integration_time) / 2
```

を使います。`vx=dx/dt_motion`, `vy=dy/dt_motion`, `wz=dtheta/dt_motion` はchild frame（車体座標）基準です。

## 有効性、同期、外れ値

両方の新しい観測が揃った場合だけ3DoF推定します。片方が停止・invalidの場合、その値をゼロ移動とは解釈せず、積算とOdometry publishを停止してWARNを出します。片側だけからyawを推定するfallbackはありません。

観測は次の場合に棄却されます。

- `valid == false`
- timestamp表現が不正
- qualityが利用可能で `quality < minimum_quality`
- `integration_time <= 0`
- ROS受信から `sensor_timeout_sec` を超過
- `abs(raw_dx/raw_dy)` が設定上限を超過
- scale・yaw変換後の値が非有限
- 左右timestamp差が `max_sensor_time_difference_sec` を超過
- 左右`cycle_id`が不一致
- 左右積算時間差が `max_sensor_interval_difference_sec` を超過
- 最小二乗残差が `max_motion_residual` を超過
- `abs(vx)`, `abs(vy)`, `abs(wz)` が各速度上限を超過

raw値はmessage上int32なのでNaNにはなりませんが、整数範囲チェックと変換後の有限値チェックを別々に実施します。raw上限のデフォルトは未校正の暫定値です。

qualityが通信に存在しない場合は `quality_available=false` とし、値を捏造しないでください。その観測にはquality filterを適用しません。SQUALが利用可能なら、最小二乗を行う前のtracking validity判定として使います。片側low qualityを `delta=0` として推定へ入れることはありません。

内部の `SensorValidity` はtimestamp、integration time、timeout、quality、変換値、raw範囲、入力側validを独立に保持します。debugのreject reasonは次のenumです。

| 値 | 名前 | 意味 |
| ---: | --- | --- |
| 0 | `REJECT_NONE` | 棄却なし |
| 1 | `REJECT_TIMEOUT` | 受信timeout |
| 2 | `REJECT_LOW_QUALITY` | SQUAL等が下限未満 |
| 3 | `REJECT_INVALID_TIMESTAMP` | timestamp表現が不正 |
| 4 | `REJECT_TIME_MISMATCH` | 左右measurement end差が過大 |
| 5 | `REJECT_INVALID_INTEGRATION_TIME` | 積算時間が非正または不正 |
| 6 | `REJECT_INTEGRATION_TIME_MISMATCH` | 左右積算時間差が過大 |
| 7 | `REJECT_NONFINITE_VALUE` | 変換値または推定値が非有限 |
| 8 | `REJECT_RAW_DELTA_OUTLIER` | raw count上限超過 |
| 9 | `REJECT_VELOCITY_OUTLIER` | vx/vy/wz上限超過 |
| 10 | `REJECT_RESIDUAL_OUTLIER` | motion residual上限超過 |
| 11 | `REJECT_SOURCE_INVALID` | 入力アダプタのvalidがfalse |
| 12 | `REJECT_CYCLE_MISMATCH` | 左右cycle IDが不一致 |

## ROSインターフェース

Published topics:

| Topic | Type | 内容 |
| --- | --- | --- |
| `/mouse_odom` | `nav_msgs/msg/Odometry` | 受理済み観測だけを積算したodom。TFはpublishしない |
| `/mouse_odom/debug` | `mouse_odometry/msg/Pmw3901Debug` | cycle ID/match、raw/変換値、quality、左右・pair validity/reason、積算時間差、timestamp差、推定delta/速度、motion residual |

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
| `max_abs_raw_delta_x` | `32767` | センサXのraw差分上限。未校正の暫定値 |
| `max_abs_raw_delta_y` | `32767` | センサYのraw差分上限。未校正の暫定値 |
| `left_flow_topic` | `/pmw3901/left/flow` | 左入力topic |
| `right_flow_topic` | `/pmw3901/right/flow` | 右入力topic |
| `frame_id` | `mouse_odom` | Odometry親frame |
| `child_frame_id` | `mouse_base_link` | Odometry子frame |

raw・速度・motion residual・quality閾値は、実測データから調整する必要があります。

## Packet v2候補

v1は左右raw flowを同じcycleへまとめるため既存推定器へ安全に接続できます。将来v2には`session_id`、`sensor_read_offset_us`、左右個別measurement timestamp/integration time、shutter、overflow/saturation availability、firmware build IDを追加すると、再起動判定、逐次読出し補正、tracking診断を改善できます。version dispatchは`decodePacketV1()`から分離してあるため、既存v1を保ったまま追加できます。

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

BridgeとOdometry nodeを同時に起動する場合:

```bash
ros2 launch mouse_odometry pmw3901_odometry.launch.py \
  serial_port:=/dev/ttyACM0 serial_baud:=115200
```

実機接続とtopic確認:

```bash
ls -l /dev/ttyACM*
ros2 run mouse_odometry pmw3901_serial_bridge --ros-args \
  -p serial_port:=/dev/ttyACM0 -p serial_baud:=115200
ros2 topic echo /pmw3901/left/flow
ros2 topic echo /pmw3901/right/flow
ros2 topic echo /pmw3901/serial_debug
ros2 topic echo /mouse_odom
ros2 topic echo /mouse_odom/debug
ros2 service call /reset_mouse_odom std_srvs/srv/Empty "{}"
```

左右topicでは`cycle_id`、raw delta、quality/availability、valid、integration time、header stampを確認します。正常時は左右cycle IDが一致し、header stampがESP起動後数秒ではなく現在のROS時間系になっている必要があります。device権限がない場合は、運用環境に合わせてdialout groupまたはudev ruleを設定してください。

## 実機キャリブレーション

校正中は路面、照明、高さ、速度を実運用条件に固定し、raw countを `/mouse_odom/debug` または入力側ログで記録します。

1. X方向: yawを変えず、正確に例えば1.0 m前進します。左右各センサについて `実距離 / 累積X count` を計算し、各 `x_meter_per_count` に設定します。符号もこの試験で確定します。
2. Y方向: 可能ならyawを変えず、既知距離だけ横移動します。左右各々の `実距離 / 累積Y count` を設定します。車体を正確に横移動できない場合、純粋なY校正は困難なので、治具やセンサ単体の直線ステージを使用してください。斜め移動をYだけの校正値へ混ぜないでください。
3. 取付角: 車体軸に沿った直線移動で他方の軸へ現れる成分を確認し、`left_sensor_yaw` と `right_sensor_yaw` を調整します。軸の取り違えはyaw角だけでなく入力アダプタの実仕様と照合します。
4. yaw: 90/180/360 degなど既知角度だけ回し、推定yawを比較します。yawだけを合わせるため実測したセンサ間距離を恣意的に変更せず、センサ位置、左右各軸scale、取付角、flowの追跡状態を切り分けます。
5. 高さ依存: 同じ校正高さを `sensor_height_from_ground` に記録します。高さを変える場合は再校正してください。距離センサを将来追加するまでは自動補正されません。

## 数値テスト

`test/test_planar_motion_estimator.cpp`、`test/test_flow_validation.cpp`、`test/test_pmw3901_serial_protocol.cpp`に数値・validation・transport testを実装しています。

| Test | 真値 `(dx, dy, dtheta)` | 理論観測 `(left_dx,left_dy,right_dx,right_dy)` | 期待結果 |
| --- | --- | --- | --- |
| A 直進 | `(0.100, 0, 0)` | `(0.100,0,0.100,0)` | `(0.100,0,0)` |
| B 横滑り | `(0, 0.100, 0)` | `(0,0.100,0,0.100)` | `(0,0.100,0)` |
| C 純粋yaw | `(0,0,0.100)` | `(-0.0055,0.022,+0.0055,0.022)` | `(0,0,0.100)` |
| D 複合 | `(0.080,0.020,0.050)` | `(0.07725,0.031,0.08275,0.031)` | 元の真値 |
| E 左X scale +1% | 本来直進 | `(0.101,0,0.100,0)` | false yaw、`motion_residual≈0`を確認 |
| F 片側X異常 | 不明 | `(0.110,0,0.100,0)` | false yaw、`motion_residual≈0`を確認 |
| G 積算時間不一致 | 左0.010 s、右0.020 s | timestampは一致 | pair棄却、積算不可 |
| H timestamp不一致 | 左右差0.1 s | deltaは正常 | pair棄却、積算不可 |
| I low SQUAL | 左qualityが下限未満 | 右は正常 | 左invalid、pair棄却 |
| J 取付yaw誤差 | 左だけ設定誤差1 deg | 本来直進 | x/y/yawとresidualへの混入を確認 |

E/Fは故障検出の成功テストではなく、**motion residualでは検出できない異常があることを仕様として固定するテスト**です。G〜Iはvalidation結果がinvalidになることを直接確認します。raw count上限超過の追加テストもあります。

Serial/packet testは、正常33-byte decode、負int16 little-endian、CRC破損、分割入力、2 packet連結、先頭garbage、破損後の再同期、cycle drop、uint32 wrap、ESP→ROS時刻変換、初期minimum-offset推定、ESP timestamp巻戻り、status error優先を検証します。flow validationでは左右cycle mismatchを棄却し、同一cycleを受理することも確認します。

実行:

```bash
cd ~/mouse_ws
colcon test --packages-select mouse_odometry
colcon test-result --verbose
```

## IMUを使用しないこと

`/mouse_odom`のx/y/yawおよびvx/vy/wzは、左右PMW3901だけから`PlanarMotionEstimator`で算出します。このパッケージはIMU topicをsubscribeせず、gyro補正、EKF、姿勢補正、scale補正、IMUによるhealth判定を行いません。`sensor_msgs`や`robot_localization`への依存もありません。

## 前提と限界

PMW3901はオプティカルフローセンサであり、raw countは絶対的な実距離ではありません。精度は路面模様、センサと路面の距離、照明、振動、pitch/roll、個体差、速度、tracking lossに依存します。現在の2D odometryは平坦路面かつセンサ高さがほぼ一定という前提です。

未解決なのは実機device名/権限、USB抜差し時のOS挙動、長時間のclock drift/transport jitter、4軸の実測scale、センサ軸の符号・取付角、qualityの実際の範囲、運用環境に適した閾値です。

現在検出できるのは、停止・通信遅延、cycle/window不一致、入力側invalid、low SQUAL、raw上限超過、モデルと矛盾する観測成分、推定速度超過です。一方、実際の剛体運動と同じ観測部分空間へ入る片側X scale/tracking error、左右に共通するscale誤差、SQUALにも現れない緩やかなbiasは2センサflowだけでは識別困難です。今回のOdometryへ別センサは一切融合していません。

Serial Bridgeとv1 wire formatは実装済みです。固定offset方式はESPとPC clockの周波数差を補正しないため、長時間運転ではdriftを実測し、必要なら線形offset+skew推定へ拡張してください。
