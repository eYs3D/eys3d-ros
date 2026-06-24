# eYs3D ステレオ深度カメラ用 ROS 2 ドライバ

[![ROS 2](https://img.shields.io/badge/ROS%202-Foxy%20%7C%20Humble%20%7C%20Jazzy-blue)](https://docs.ros.org/en/rolling/Releases.html)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)

**Language:** [English](../README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`eys3d_camera` は eYs3D ステレオ深度カメラ用の公式 ROS 2
ドライバです。カラー画像・深度画像・点群を標準 REP-103 frame tree
で配信します。ROS 2 Foxy、Humble、Jazzy に対応します。

### 対応カメラ

| 型番 | Product code | USB | 状態 |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | 量産 |
| **R77** | YX8072 | USB 2.0 | 量産 |
| **G62** | YX8081 | USB 2.0 | 量産 |

---

## インストール

```bash
sudo apt install ros-$ROS_DISTRO-diagnostic-updater
```

本パッケージを workspace の `src/` に置きビルド:

```bash
cd ~/ros2_ws
colcon build --packages-select eys3d_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

per-model launch はデフォルトで RViz を起動します。
`sudo apt install ros-$ROS_DISTRO-rviz2` で一度導入するか、
`rviz:=false` を付与して視覚化をスキップしてください。

---

## クイックスタート

各モデルにはデフォルト video mode を開く launch ショートカットが用意されています:

```bash
ros2 launch eys3d_camera G100P.launch.py     # G100+: L'+D 1280x720 interleave、SDK 30 fps/ストリーム
ros2 launch eys3d_camera R77.launch.py       # R77:   L'+D 1280x920 color + 640x460 depth @ 30 fps
ros2 launch eys3d_camera G62.launch.py       # G62:   L'+D 640x480 @ 25 fps
```

mode の切替は `mode_id:=<n>` で指定します。各モデルの全 mode 一覧は
`launch/video_modes/<MODEL>.yaml` を参照してください。

### 配信トピック

`camera_name=G100P_1` の例:

| Topic | 型 | 説明 |
|---|---|---|
| `/G100P_1/left_color` | `sensor_msgs/Image` | 左眼カラー画像。常に `rgb8`（YUYV と MJPEG ソースをインラインで RGB888 にデコード；モノクロセンサ機種は R=G=B として出力）。補正の有無は現在の video mode で決まります。|
| `/G100P_1/right_color` | `sensor_msgs/Image` | 右眼カラー画像；video mode が L\|R をワンエンドポイントに格納する場合のみ配信（`split_lr: true`）|
| `/G100P_1/depth_image` | `sensor_msgs/Image` (`16UC1`, mm, REP-118) | 深度(ミリメートル) |
| `/G100P_1/pointcloud` | `sensor_msgs/PointCloud2` | XYZ float32、ROS 基底軸(メートル);`colored_pointcloud:=true` のとき XYZRGB |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | 内部パラメータ、Image フレームごと(`header.stamp` 一致) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | ヘルスメトリクス、1 Hz |

### サブスクライバの QoS

画像 / 深度 / PointCloud2 トピックは `SensorDataQoS`
(BestEffort + KeepLast 5 + Volatile) で配信されます。サブスクライバ
は同じ QoS を宣言する必要があります:

```cpp
sub_ = create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS(), callback);
```

```python
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(Image, topic, callback, qos_profile=qos_profile_sensor_data)
```

新しい ros2cli の `ros2 topic hz` / `echo` は QoS を自動ネゴシエー
ションするため、**追加のフラグは不要**です。

### Interleave モード(G100+ のみ)

ステレオカメラには RGB 出力専用の第 3 のセンサーがありません。
「IR ドットが映り込まない color」と「高品質の depth」を同時に
取得するため、eYs3D は Interleave モードを採用しています:

- **偶数 frame —— IR オフ → `/left_color`**。
  IR ドットの映り込みがなく、クリーンな RGB 画像を出力します。
- **奇数 frame —— IR オン → `/depth_image`**。
  IR ドットがステレオマッチングに構造的特徴を与え、より高品質
  な深度が得られます。

ドライバは SDK ストリームの上流で偶数/奇数 frame を振り分けるため、
ストリームあたりの FPS は半分になります。

#### タイムスタンプへの影響

G100+ デフォルトの `mode_id=1` を例にすると:

```
sensor 60 fps  ──SDK interleave──>  /G100P_1/left_color   30 fps
                                 └  /G100P_1/depth_image  30 fps
```

color と depth は **隣接する 2 枚の sensor frame** から得られ、
stamp は **sensor frame 1 周期 ≈ 15.6 ms(1000 / 60)** ずれます。
同一ストリーム内では `Image` と対応する `CameraInfo` の stamp は
一致します。

color と depth の対応付けには `message_filters::ApproximateTime`
を使用し、`slop` を sensor frame 1 周期(例: 20 ms)に設定すれば
安定してペアリングできます。

#### 該当する mode ID

`launch/video_modes/G100P.yaml` において、**`mode_id` 1, 3, 5, 7–21**
が interleave です。それ以外のモードは color と depth を同時取得し、
stamp が一致します。

---

## 設定

### Video mode の切替

```bash
ros2 launch eys3d_camera G100P.launch.py mode_id:=7    # G100+: L'+D 640x480 interleave (SDK 30 fps)
ros2 launch eys3d_camera R77.launch.py   mode_id:=4    # R77:   D-only 640x460 @ 30 fps
ros2 launch eys3d_camera G62.launch.py   mode_id:=3    # G62:   L'+D 320x240 @ 30 fps
```

各カメラの完全な video mode 一覧は `launch/video_modes/<MODEL>.yaml`
にあります。ノード起動時に RCLCPP INFO log としても出力されますが、
per-model launch のデフォルト `log:=sdk` ではこの層が抑制されます。
表示するには `log:=all` を指定するか、YAML を直接開いてください。

### Launch パラメータ

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `camera_name` | `<MODEL>_1` | ROS namespace と frame-id の前置詞 |
| `mode_id` | `1` | video mode 一覧内のインデックス |
| `dev_serial_number` | `""` | シリアル番号の部分一致でバインド |
| `usb_port` | `""` | USB トポロジ（例 `2-3:1.0`）でバインド |
| `depth_minimum_mm` | `-1` | `/depth_image` と `/pointcloud` 双方に適用される近接カットオフ。これ未満のピクセルは 0 にクリップ。`-1` = モデル既定（G100+ 250、R77 200、G62 100）|
| `depth_maximum_mm` | `-1` | `/depth_image` と `/pointcloud` 双方に適用される遠方カットオフ。これ超過のピクセルは 0 にクリップ。`-1` = モデル既定（G100+ 1900、R77/G62 1500）|
| `colored_pointcloud` | `false` | 直近の左カラーフレームを参照して XYZRGB PointCloud2 を出力。Depth-only モードでは自動で XYZ にフォールバック |
| `spatial_filter` | `false` | 視差ドメインの edge-aware IIR 空間フィルタを有効化（bool 切替）|
| `temporal_filter` | `false` | 時間フィルタ、alpha ブレンド + persistence を有効化（bool 切替、実行中に調整可能）|
| `hole_filling` | `0` | Z ドメインの hole fill モード。`0` = OFF、`1` = fill_from_left、`2` = farthest_from_around、`3` = nearest_from_around（整数モード、bool ではない）|
| `filter_profile` | `default` | チューニングプロファイル名。`cfg/filter_profiles/<name>.yaml` に解決され、起動時にフィルタのチューニング値を読み込む |
| `ir_intensity` | `-1` | `-1` = モデル既定（G100+ / R77 = 3、G62 = 60）；`0` = OFF；正整数で FW 範囲（G100+ 0-9、R77 0-6、G62 0-96）に上書き |
| `log` | `sdk`（per-model）/ `all`（generic） | 端末出力レベル。`all` = RCLCPP + SDK の全出力；`sdk` = RCLCPP INFO/DEBUG を抑制し WARN/ERROR + SDK printf は維持；`close` = すべてを per-process ログファイルへリダイレクト（端末は無音）|
| `rviz` | `true` | RViz を自動起動 |

カラーが `left_color` と `right_color` に分割されるかは、選択した video mode（`launch/video_modes/<MODEL>.yaml` の `split_lr` フラグ）で決まり、launch 引数では制御しません。

### 画像コントロール

eYs3D の各モジュールは工場出荷時に `enable_auto_exposure` ON、
`enable_auto_white_balance` ON、`power_line_frequency` 60 Hz に
設定されています。ドライバはファームウェアの起動値をそのまま継承
し、**オペレータが明示的に上書きしたときだけカメラに書き戻します**。
そのため ROS 以外で調整した設定は再起動後も保持されます。IR だけは
例外で、起動直後はプロジェクタが OFF のため必ず適用されます。

実行中の動的調整(再起動不要):

```bash
ros2 param set /G100P_1/eys3d_camera ir_intensity         5
ros2 param set /G100P_1/eys3d_camera enable_auto_exposure      false   # マニュアル露光
ros2 param set /G100P_1/eys3d_camera exposure_time_step        -8
ros2 param set /G100P_1/eys3d_camera enable_auto_white_balance false
ros2 param set /G100P_1/eys3d_camera power_line_frequency 1       # 1 = 50 Hz、2 = 60 Hz
```

### 後処理フィルタ

3 種のオプションフィルタを内蔵しています。すべて既定で OFF、
互いに独立して動作するため任意の組み合わせが可能です。

| フィルタ | 切替方法 |
|---|---|
| `spatial_filter` | launch のみ |
| `temporal_filter` | launch + 実行中 |
| `hole_filling` | launch のみ(`0` = OFF;`2` = 推奨開始モード)|

チューニング値(`alpha` / `delta` / `magnitude` / `holes_fill` /
`persistence`)は `cfg/filter_profiles/<name>.yaml` にあります。
カスタムプロファイルを作成するには `default.yaml` をコピーして
フィールドを編集してください;欠落フィールドはノードのコンパイル時
デフォルトにフォールバックします。

```bash
# 既定プロファイルで spatial + temporal を有効化
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true temporal_filter:=true

# farthest_from_around hole filling を追加(推奨開始モード)
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true hole_filling:=2

# カスタムチューニングプロファイルへ切替
ros2 launch eys3d_camera G100P.launch.py \
    spatial_filter:=true filter_profile:=indoor
```

`temporal_filter` のパラメータは実行中に再設定可能です:

```bash
ros2 param set /G100P_1/eys3d_camera temporal_filter             true
ros2 param set /G100P_1/eys3d_camera temporal_filter_alpha       0.4
ros2 param set /G100P_1/eys3d_camera temporal_filter_persistence 3
```

### 実行時ストリーム制御

カメラ出力を一時停止または停止する 2 つのサービスがあります
(ROS node はそのまま走り続けます)。どちらも color と depth を
まとめて制御します。

```bash
# フレームの publish は停止しますが、カメラは USB 上で stream を
# 続けます。ドライバ CPU はほぼ 0 に落ち、resume 後の次フレームは
# すぐ到着します。短い中断用。
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: false}"

# USB パイプを完全に解放します。ROS node は生きたまま、topic も
# advertised のまま。resume は約 300 ms でカメラを再オープンします。
# USB 帯域を他のデバイスに譲りたいときに使います。
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: false}"
```

Standby 中は、自動再接続 watchdog (次節) は無信号を意図的なものと
判定し、切断としては扱いません。現在の制御状態は `/diagnostics` の
`stream_state` キー (`Active` / `Paused` / `Standby`) で公開されます。

### ホットプラグ自動復旧

ドライバは 1 Hz の watchdog を内蔵しています。color **と** depth の
両方が **3 秒間連続**でフレームを受信しない場合(起動直後は 10 秒に
緩和)、watchdog はデバイスを close し再接続ループに入り、2 秒ごとに
再オープンを試みます。カメラが再接続されると元の topic は自動的に
再開され、**`ros2 launch` の再起動は不要**です。

```
[ERROR] watchdog: no frames for 3 s — declaring camera disconnected
[INFO]  watchdog: reconnect succeeded after 2 attempt(s)
```

低速モード(例:R77 の 7 fps)は 10 秒の起動猶予で吸収されます。
最初の 1 フレームを観測した時点で、3 秒の定常判定しきい値に切り替わります。

### マルチカメラ

複数台接続時、ドライバは launch の `model` に対応する USB PID
(G100+ = `0x0181`、R77 = `0x0180`、G62 = `0x0183`)に一致するデバイスを
選択します。`model` に対応する PID のカメラが見つからない場合や、
`usb_port` / `dev_serial_number` が別モデルに解決された場合、ドライバ
は PID 不一致エラーで起動を拒否します。

同じモデルを 2 台識別する場合、launch は**必ず**シリアル番号
または USB トポロジパスで明示的にバインドしてください — 自動の
model-PID マッチでは同モデル 2 台を区別できません。どちらの
バインドも再起動・着脱順に左右されません:

```bash
ros2 launch eys3d_camera G100P.launch.py \
    camera_name:=front dev_serial_number:=8036259M200025

ros2 launch eys3d_camera G100P.launch.py \
    camera_name:=rear  usb_port:=2-3:1.0
```

`launch/examples/` に 3 つのマルチカメラサンプル launch を同梱
しています。それぞれ `usb_port` のプレースホルダがあるので、実機の
配線に合わせて書き換えてから起動してください:

```bash
ros2 launch eys3d_camera examples/dual_G62.launch.py
ros2 launch eys3d_camera examples/dual_G100P.launch.py
ros2 launch eys3d_camera examples/G100P_plus_R77.launch.py
```

同梱のマルチカメラサンプルは、USB 帯域の範囲内で 2 台が安定して
立ち上がるよう、あらかじめ軽めの `mode_id` を選んであります。
手動で `mode_id` を上げてカメラが立ち上がらない場合は、軽い mode
に戻すか、2 台を別々の USB ポートに分けてください。

どのヒントも候補にマッチしない場合、ノードは全デバイスの
`(PID, serial_number, /dev/videoN, usb_port)` をログに出力して終了
します —— 正しい値を launch にコピーしてください。

---

## Frame ID

ドライバは起動時に静的 TF ツリーを 1 回ブロードキャストします。
ルートは `<camera_name>_link`(ROS 基底軸: X 前、Y 左、Z 上)。
各ストリームにはセンサ frame と REP-103 `_optical_frame` があります。
PointCloud2 は `<camera_name>_points_frame` を使用し、すでに ROS
基底軸に変換済みです(追加の回転は不要)。

ロボット URDF に統合する場合は、既存 frame の下に `<camera_name>_link`
をマウントしてください。最小例(`parent_link`、joint 名、取付姿勢は
プラットフォームに合わせて調整してください):

```xml
<joint name="g100p_mount" type="fixed">
  <parent link="parent_link"/>
  <child  link="G100P_1_link"/>
  <origin xyz="0.10 0.00 0.05" rpy="0 0 0"/>
</joint>
<link name="G100P_1_link"/>
```

ドライバはカメラ TF ツリーを `TRANSIENT_LOCAL` durability で
`/tf_static` に 1 回配信します。後から購読を始めたノードも
キャッシュされた transform を即座に受け取れます。

---

## Composable Node

ドライバは `rclcpp` component として登録されており、
`ComposableNodeContainer` にロードして下流 component と同一プロセスで
メッセージ交換できます。`use_intra_process_comms:=true` を設定すると、
画像・深度・点群メッセージは同一コンテナ内のサブスクライバへポインタで
引き渡され、DDS を介しません。

```bash
ros2 launch eys3d_camera examples/G100P_composable.launch.py \
    use_intra_process_comms:=true
```

`launch/examples/G100P_composable.launch.py` を起点に、
`composable_node_descriptions` に自分の `ComposableNode` を追加し、
各エントリに `extra_arguments=[{'use_intra_process_comms': True}]`
を付けるとコンテナを共有できます。この launch は `params_file` 引数を
受け付け、ドライバ設定一式を 1 つの YAML にまとめて渡せます。

`/tf_static` は常に `TRANSIENT_LOCAL` durability で配信され、ノード
レベルの intra-process 設定にかかわらず動作します。

---

## 診断トピック `/diagnostics`

毎秒 1 件 `DiagnosticArray` を配信します。1 件あたり 5 つの
`DiagnosticStatus`(タスクごとに 1 つ)を含み、名前は
`"<hardware_id>: <task>"` 形式(`hardware_id` はカメラのシリアル
番号、モジュールが報告しない場合は `eys3d_camera`)。全体の健全性は
`device` タスクの `level` と `message` に要約されます:

| `level` | `message` | 意味 |
|---|---|---|
| `OK` | `streaming OK` | 各有効ストリームが期待 fps の 50 % 以上 |
| `WARN` | `one stream below 50% of expected fps` | 片方のストリームが遅い |
| `ERROR` | `no frames flowing on enabled streams` | 有効ストリームが全て閾値未満(または完全に無音) |
| `ERROR` | `camera disconnected; Linux device node not present` | USB 切断、ウォッチドッグがデバイス復帰時に自動再接続 |

タスクごとの `values` キーバリュー:

**`device`** — 接続 + 識別情報:

| Key | 説明 |
|---|---|
| `connection_state` | `streaming` または `disconnected` |
| `device_present` | `true` / `false`、V4L2 ノードの有無 |
| `reconnect_attempts` | 起動以降の累積再接続試行回数 |
| `usb_port` | open 時に解決した sysfs インタフェースパス(例 `2-3:1.0`)|
| `serial_number` | SDK が返したモジュールシリアル |
| `actual_fps` | `APC_OpenDevice2` の返した fps(interleave mode はストリーム単体で半分)|
| `stream_state` | `Active` / `Paused` / `Standby` — `pause` / `standby` サービスが制御する実行時状態 |

**`color`** と **`depth`** — ストリーム別スループット:

| Key | 説明 |
|---|---|
| `input_fps` | 直近 1 秒で SDK から受信したフレーム数。購読者の有無に依存せず、カメラ / USB 健全性の指標 |
| `publish_fps` | 直近 1 秒で実際にトピックへ発行したフレーム数。購読者なしなら 0、追従できていれば `input_fps` と一致、継続的に下回る場合はドライバ側が遅れている |
| `input_total` | open 以降に SDK から受信した累積フレーム数 |
| `publish_total` | open 以降に発行した累積フレーム数 |
| `input_dropped` | SDK 側の累積ドロップ数(シリアル番号の不連続から検出) |
| `decode_avg_ms` | (`color` のみ)直近 1 秒の color デコード平均時間。直近 1 秒内にデコードが発生した場合のみ出力 |
| `decode_max_ms` | (`color` のみ)これまで観測した最長 color デコード時間。一度でもデコードが発生した場合のみ出力 |

**`pointcloud`** — 投影 + 後処理カウンタ:

| Key | 説明 |
|---|---|
| `compute_status` | `active`(購読者あり)、`idle (no /pointcloud subscriber)`、`idle (never run ; no subscriber since start)`、`(disconnected — see device task)` |
| `publish_fps` | 直近 1 秒の点群発行数。購読者なしで 0 |
| `compute_avg_ms` | 直近 1 秒の点群計算平均時間(`active` のとき出力) |
| `compute_max_ms` | これまで観測した最長点群計算時間 |
| `publish_total` | 累積発行点群数 |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | 各後処理段の累積適用回数 |

**`thermal`**:

| Key | 説明 |
|---|---|
| `temperature_c` | チップ温度(°C)。非対応モデルは `n/a (not supported on this model)` |

`/diagnostics` メッセージは少なくとも 1 つの購読者が接続している
ときにのみ組み立てて発行されます。ホットパスの atomic カウンタは常
時動作するため、後から監視を接続しても open 以降の累積値を確認で
きます。

判読方法:

| `input_fps` | `publish_fps` | 意味 |
|---|---|---|
| ≈ 期待値 | 0 | 購読者なし、ドライバはアイドル |
| ≈ 期待値 | ≈ input | ドライバが追従できている |
| ≈ 期待値 | < input | 購読者あり、配信パスが遅れている(デコードが遅い / DDS 渋滞 / 購読側 QoS 不一致)|
| 0 | 0 | カメラ / USB がデータを送出していない — `device.connection_state` を確認 |

`/diagnostics` を購読(あるいは `rqt_robot_monitor` を実行)すれば
リアルタイム表示できます。

同梱モニタは全ストリームを自動購読し、SDK / Pub / Rx の各レートと
デコード / 計算所要時間を並べて表示します:

```bash
ros2 run eys3d_camera perf_monitor            # namespace 自動検出
ros2 run eys3d_camera perf_monitor --ns /G100P_1 --interval 0.5
```

---

## トラブルシューティング

| 症状 | 対処 |
|---|---|
| 自前のサブスクライバが画像 / 点群メッセージを受信できない | ドライバは `SensorDataQoS`(BestEffort + KeepLast 5 + Volatile) で配信します。サブスクライバ側は互換 QoS を宣言する必要があります — C++ は `rclcpp::SensorDataQoS()`、Python は `qos_profile_sensor_data`。詳細は[サブスクライバの QoS](#サブスクライバの-qos)節を参照 |
| RViz の Image パネルが空白 | Fixed Frame を `<camera_name>_link` に設定し、topic 名が現在の `camera_name` namespace と一致するか確認 |
| カメラ open が "device busy" で失敗 | 他プロセスが `/dev/videoN` を保持中。`lsof /dev/video*` で特定し、driver の残留であれば `pkill -9 -f camera_node` で終了 |
| `No device matches binding hints` | log に検出した全 eYs3D モジュールが列挙されます。正しい `dev_serial_number` または `usb_port` を launch 引数に転記 |
| `cv_bridge` が encoding 不一致を報告 | color topic は wire format に依らず `rgb8` で publish。サブスクライバは `bgr8` や `mono8` ではなく `rgb8` を要求してください |

### サブスクライバが受信する fps が driver の公開 fps より少ない

`perf_monitor` または `ros2 topic hz` で Rx < Pub と表示される一方で
`/diagnostics` が `dropped = 0` を示す場合、損失は driver ではなく
DDS 伝送層で起きています。

サポートされる video mode では、画像 1 フレームのサイズが kernel
UDP socket と IP fragment reassembly の単発バーストで吸収できる
容量を超えます(`rgb8` 1280×720 ≈ 2.7 MB、典型的な point cloud は
最大 ~11 MB)。同じ根本原因(kernel 側バッファのオーバーフロー)が
2 つの運用環境で異なる症状として現れます:

- **高性能な x86 ホスト**: driver が各フレームを密な UDP fragment
  burst として送り込み、受信ソケット / reassembly queue が消化し
  終わる前に溢れます。
- **リソース制約の強い ARM ホスト**: 同じ kernel buffer の消化が
  遅く、結果としてバッファが先に満杯になります。

本パッケージには FastRTPS を 32 MB の shared-memory segment へ
切り替える opt-in スクリプトが同梱されており、**同一ホスト**上の
サブスクライバについては UDP fragmentation を取り除けます。**別
ホスト**上のサブスクライバには適用できません — その場合は
[ROS 2 DDS tuning guide](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html)
に従って `net.core.rmem_max` を引き上げてください。SHM segment は
ROS 2 participant ごとに `/dev/shm` を約 32 MB 占有するため、
自動では適用されません。

ユーザーアカウントに対して有効化する手順: 次の 1 行を `~/.bashrc`
の `source <workspace>/install/setup.bash` **より後** に追記して
ください。`~/ros2_ws` は実際の workspace パスに置き換えます:

```bash
source ~/ros2_ws/install/eys3d_camera/share/eys3d_camera/config/enable_fastrtps_shm.bash
```

