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
sudo apt install ros-$ROS_DISTRO-diagnostic-updater \
                 ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-rviz2
```

機種別 launch は `urdf:=true` と `rviz:=true` がデフォルトのため、
`robot_state_publisher`、`xacro`、`rviz2` が無いと起動しません。
`urdf:=false rviz:=false` を渡せば不要です。

`eys3d_camera` と `eys3d_camera_interfaces`（セルフキャリブレーションの
action 定義）の 2 つのパッケージを workspace の `src/` に置きビルドします。
`--packages-up-to` により interfaces が先にビルドされます:

```bash
cd ~/ros2_ws
colcon build --packages-up-to eys3d_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


### デバイスの権限

ドライバは一般ユーザーとしてカメラを開きます。デバイスのオープンが
権限エラーで失敗する場合は、eSPDI SDK が USB デバイスにアクセスできる
よう、同梱の udev ルールをインストールしてください:

```bash
sudo cp install/eys3d_camera/share/eys3d_camera/udev/99-eys3d.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

その後カメラを挿し直します。このルールは eYs3D デバイス（USB vendor
`3438`）へのアクセスを許可します。あるいは、ユーザーを `video` グループ
に追加して再ログインしても構いません。

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
| `/G100P_1/left_color/image_raw` | `sensor_msgs/Image` | 左眼カラー画像。常に `rgb8`（YUYV と MJPEG ソースをインラインで RGB888 にデコード；モノクロセンサ機種は R=G=B として出力）。補正の有無は現在の video mode で決まります。|
| `/G100P_1/right_color/image_raw` | `sensor_msgs/Image` | 右眼カラー画像；video mode が L\|R をワンエンドポイントに格納する場合のみ配信（`split_lr: true`）|
| `/G100P_1/depth/image_raw` | `sensor_msgs/Image` (`16UC1`, mm, REP-118) | 深度(ミリメートル) |
| `/G100P_1/depth/points` | `sensor_msgs/PointCloud2` | XYZ float32、ROS 基底軸(メートル);`colored_pointcloud:=true` のとき XYZRGB |
| `/G100P_1/<stream>/camera_info` | `sensor_msgs/CameraInfo` | 内部パラメータ、Image フレームごと(`header.stamp` 一致) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | ヘルスメトリクス、1 Hz |

### CameraInfo と歪み係数

各 `camera_info` は、その topic で配信される画像そのものを記述します。

深度はすべてのモードで補正済みです。カラーはカタログ名にアポストロフィが
付くモード(`L'+D`、`L'+R'+D`)で補正済みです。それらの topic では `k` は
`p` の左 3×3、`d` はゼロ(`plumb_bob`)、`r` は単位行列になります。カメラが
すでに歪みを取り除いているため、戻すものが残っていません。

`L+R` と `L+R+D` モードはカラー topic に生のセンサー画像を配信します。
そこでの `k` と `d` は配信解像度における工場出荷時のレンズモデル、`r` は
補正回転で、`image_proc` がそのまま補正できます。係数の個数を仮定せず
`distortion_model` を読んでください。ドライバは `rational_polynomial` 8 個
または `plumb_bob` 5 個を、その個体に保存された校正値に従って報告します。

いずれの場合も `p` が投影行列であり、姿勢推定(AprilTag、PnP、SLAM)で
使うべき内部パラメータです。ステレオ対では右カメラの `p[3]` が
`-fx × ベースライン`(メートル)です。

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

- **偶数 frame —— IR オフ → `/left_color/image_raw`**。
  IR ドットの映り込みがなく、クリーンな RGB 画像を出力します。
- **奇数 frame —— IR オン → `/depth/image_raw`**。
  IR ドットがステレオマッチングに構造的特徴を与え、より高品質
  な深度が得られます。

ドライバは SDK ストリームの上流で偶数/奇数 frame を振り分けるため、
ストリームあたりの FPS は半分になります。

#### タイムスタンプへの影響

USB 3.0 での G100+ デフォルト `mode_id=1` を例にすると:

```
sensor 60 fps  ──SDK interleave──>  /G100P_1/left_color/image_raw  30 fps
                                 └  /G100P_1/depth/image_raw       30 fps
```

color と depth は **隣接する 2 枚の sensor frame** から得られ、
stamp は **sensor frame 1 周期 ≈ 15.6 ms(1000 / 60)** ずれます。
同一ストリーム内では `Image` と対応する `CameraInfo` の stamp は
一致します。

color と depth の対応付けには `message_filters::ApproximateTime`
を使用し、`slop` を sensor frame 1 周期(例: 20 ms)に設定すれば
安定してペアリングできます。

#### 該当する mode ID

`launch/video_modes/G100P.yaml` において、**`mode_id` 1, 3, 5, 7–21, 56, 57**
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
| `mode_id` | `-1` | video mode 一覧内のインデックス;`-1` = 自動(ネゴシエートされた USB 速度に対する signature デフォルト) |
| `dev_serial_number` | `""` | シリアル番号の部分一致でバインド |
| `usb_port` | `""` | USB トポロジ（例 `2-3:1.0`）でバインド |
| `depth_near_mm` | `-1` | `/depth/image_raw` と `/depth/points` 双方に適用される近接カットオフ。これ未満のピクセルは 0 にクリップ。`-1` = モデル既定（G100+ 250、R77 200、G62 100）|
| `depth_far_mm` | `-1` | `/depth/image_raw` と `/depth/points` 双方に適用される遠方カットオフ。これ超過のピクセルは 0 にクリップ。`-1` = モデル既定（G100+ 1900、R77/G62 1500）|
| `colored_pointcloud` | `false` | 直近の左カラーフレームを参照して XYZRGB PointCloud2 を出力。Depth-only モードでは自動で XYZ にフォールバック |
| `spatial_filter` | `false` | 視差ドメインの edge-aware IIR 空間フィルタを有効化（bool 切替）|
| `temporal_filter` | `false` | 時間フィルタ、alpha ブレンド + persistence を有効化（bool 切替、実行中に調整可能）|
| `hole_filling` | `0` | Z ドメインの hole fill モード。`0` = OFF、`1` = fill_from_left、`2` = farthest_from_around、`3` = nearest_from_around（整数モード、bool ではない）|
| `filter_profile` | `default` | チューニングプロファイル名。`cfg/filter_profiles/<name>.yaml` に解決され、起動時にフィルタのチューニング値を読み込む |
| `ir_value` | `-1` | `-1` = モード依存の既定値：深度ありのモードまたはモノクロ機（G62 / R77）ではモデル既定（G100+ / R77 = 3、G62 = 60）、カラーセンサーのカラー専用モードでは OFF；`0` = OFF；正整数で FW 範囲（G100+ 0-6、R77 0-6、G62 0-96）に上書き |
| `log` | `sdk`（per-model）/ `all`（generic） | 端末出力レベル。`all` = RCLCPP + SDK の全出力；`sdk` = RCLCPP INFO/DEBUG を抑制し WARN/ERROR + SDK printf は維持；`close` = すべてを per-process ログファイルへリダイレクト（端末は無音）|
| `urdf` | `true` | namespace 分離した `robot_state_publisher` で `<camera_name>/robot_description` にカメラモデル(mesh + 取付穴 frame)を配信。ロボット側の記述に既にカメラが含まれる場合は `false` |
| `rviz` | `true` | RViz を自動起動 |
| `selfcal_enable` | `false` | オプションのセルフキャリブレーションを有効化。`true` で `selfcal/run` アクションと `selfcal/commit` サービスが利用可能になる（「セルフキャリブレーション」節を参照）。launch 時のみ |

カラーが `left_color` と `right_color` に分割されるかは、選択した video mode（`launch/video_modes/<MODEL>.yaml` の `split_lr` フラグ）で決まり、launch 引数では制御しません。

### 画像コントロール

eYs3D の各モジュールは工場出荷時に `auto_exposure` ON、
`auto_white_balance` ON、`power_line_frequency` 60 Hz に
設定されています。ドライバはファームウェアの起動値をそのまま継承
し、**オペレータが明示的に上書きしたときだけカメラに書き戻します**。
そのため ROS 以外で調整した設定は再起動後も保持されます。IR だけは
例外で、起動直後はプロジェクタが OFF のため必ず適用されます。

実行中の動的調整(再起動不要):

```bash
ros2 param set /G100P_1/eys3d_camera ir_value             5
ros2 param set /G100P_1/eys3d_camera auto_exposure        false   # マニュアル露光
ros2 param set /G100P_1/eys3d_camera exposure_time_step   -8
ros2 param set /G100P_1/eys3d_camera auto_white_balance   false
ros2 param set /G100P_1/eys3d_camera power_line_frequency 1       # 1 = 50 Hz、2 = 60 Hz
```

`exposure_time_step` は **[-13, 3]** の符号付き整数（log2 露光レジスタ）を取り、`auto_exposure` が `false` のときにのみ適用されます。

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

カメラ出力を制御する 3 つのサービスがあります(ROS node はそのまま
走り続けます)。`pause` と `standby` は `std_srvs/srv/SetBool` を受け取り
color と depth をまとめて制御します。`hw_reset` は `std_srvs/srv/Empty`
を受け取り、USB 経由でカメラをハードウェアリセットします。

```bash
# フレームの publish は停止しますが、カメラは USB 上で stream を
# 続けます。ドライバ CPU はほぼ 0 に落ち、resume 後の次フレームは
# すぐ到着します。短い中断用。
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/pause   std_srvs/srv/SetBool "{data: false}"

# USB パイプを完全に解放します。ROS node は生きたまま、topic も
# advertised のまま。resume はカメラを再オープンし、機種により数秒かかります。
# USB 帯域を他のデバイスに譲りたいときに使います。
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: true}"
ros2 service call /G100P_1/standby std_srvs/srv/SetBool "{data: false}"

# USB 経由でカメラをリセット(デバイスを再列挙)します。node は
# ストリームを停止してリセットを発行し、その後 watchdog が自動で
# 再接続します。フレームは通常約 12 秒で復帰します。node を再起動
# せずに固まったカメラを復旧するために使います。
ros2 service call /G100P_1/hw_reset std_srvs/srv/Empty
```

Standby 中は、自動再接続 watchdog (次節) は無信号を意図的なものと
判定し、切断としては扱いません。現在の制御状態は `/diagnostics` の
`stream_state` キー (`Active` / `Paused` / `Standby`) で公開されます。

### ホットプラグ自動復旧

ドライバは 1 Hz の watchdog を内蔵し、ストリームごとに個別に監視します。
あるストリームが一度フレームを出したあと **3 秒間連続**で無音になると、
デバイスを close して再接続ループに入り、2 秒ごとに再オープンを試みます。
color が流れ続けたまま depth がファームウェア側で停止した場合も復旧します。
最初のフレームが届くまでのしきい値は 10 秒で、R77 の 7 fps のような低速
モードをカバーします。カメラが再接続されると元の topic は自動的に再開され、
**`ros2 launch` の再起動は不要**です。

```
[ERROR] watchdog: depth stream silent for 3 s; declaring camera disconnected
[INFO]  watchdog: reconnect succeeded after 2 attempt(s)
```

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
ros2 launch eys3d_camera dual_G62.launch.py
ros2 launch eys3d_camera dual_G100P.launch.py
ros2 launch eys3d_camera G100P_plus_R77.launch.py
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

各モデルには 3D mesh 付きの URDF/xacro 記述ファイルが付属します
(`urdf/eys3d_<MODEL>.urdf.xacro` + `meshes/<MODEL>.dae`)。`<name>_link`
は深度起点に位置します —— 左右イメージャの中央、光軸の高さ、カメラ
前面から Z' だけ内側(G100+ 6.75 mm、R77 4.8 mm、G62 3.1 mm)。ロボットの
既存 frame の下に、実際の取付姿勢で macro をインスタンス化してください:

```xml
<xacro:include filename="$(find eys3d_camera)/urdf/eys3d_G100P.urdf.xacro"/>
<xacro:eys3d_G100P name="G100P_1" parent="parent_link">
  <origin xyz="0.10 0.00 0.05" rpy="0 0 0"/>
</xacro:eys3d_G100P>
```

単体プレビュー(`name` の既定値は `<MODEL>_1` で、ドライバおよび付属の
rviz レイアウトと一致します):

```bash
ros2 launch eys3d_camera display_model.launch.py model:=G100P
```

3 モデルを並べてプレビュー（ハードウェア不要）:

```bash
ros2 launch eys3d_camera three_models.launch.py
```

各記述ファイルには筐体の取付穴 frame(`<name>_tripod_frame` および
`<name>_back/bottom_screw*_frame`、位置は工場 CAD 由来)も含まれ、
メカ設計との整合確認に使えます。

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
ros2 launch eys3d_camera G100P_composable.launch.py \
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
| `OK` | `streaming` | 設定された各ストリームが配信中 |
| `OK` | `streaming (paused — publish gated by operator)` | `pause` が有効 |
| `OK` | `standby (USB pipe closed by operator)` | `standby` が有効 |
| `ERROR` | `no frames flowing on any configured stream` | 設定された全ストリームが期待レートの半分未満 |
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

サマリは `streaming`、`input rate below 50% of expected`(WARN)、
`not configured (D-only mode)`、`standby` のいずれかです。

**`pointcloud`** — 投影 + 後処理カウンタ:

| Key | 説明 |
|---|---|
| `compute_status` | `active`(購読者あり)、`idle (no /depth/points subscriber)`、`idle (never run ; no subscriber since start)`、`(disconnected — see device task)` |
| `publish_fps` | 直近 1 秒の点群発行数。購読者なしで 0 |
| `compute_avg_ms` | 直近 1 秒の点群計算平均時間(`active` のとき出力) |
| `compute_max_ms` | これまで観測した最長点群計算時間 |
| `publish_total` | 累積発行点群数 |
| `spatial_filter_total` / `temporal_filter_total` / `hole_fill_total` | 各後処理段の累積適用回数 |

**`thermal`**:

| Key | 説明 |
|---|---|
| `temperature_c` | チップ温度(°C)。センサーを備えたモデルで読み取りに成功した場合のみ発行され、それ以外はキー自体が現れず、理由はタスクの summary に入る |

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

## セルフキャリブレーション

オプションのストリーム内セルフキャリブレーションは、ステレオペアを再整合し、
キャリブレーションがずれたモジュールで深度の充填率を回復します。既定で
ビルドに含まれ(CMake `EYS3D_WITH_SELFCAL`)、launch 時に
`selfcal_enable:=true` で有効化します。

```bash
ros2 launch eys3d_camera G100P.launch.py selfcal_enable:=true
```

実行時はカメラが**深度モードをストリーミングしており、作動距離にある通常の
テクスチャのあるシーンに向いている**必要があります —— キャリブレーターは
深度カバレッジを測るため、有効な深度が必要です。1 回の `selfcal/run` アクション
がセッション全体を実行し、内蔵のチューニングが自動的に適用されます。

```bash
# 1 回のセッションを実行(収束までブロック、約 20-30 秒、続けて短い再チェック。
# --feedback は phase / progress をストリーミング)。auto_commit_shift_px < 0 は
# 結果を有効なまま保持するが flash には決して書き込まない。>= 0 は cy シフトが
# その画素数に達すると、検証済みで改善された実行を自動書き込みする(下表を参照)。
ros2 action send_goal /G100P_1/selfcal/run \
  eys3d_camera_interfaces/action/SelfCal \
  "{auto_commit_shift_px: 0.25}" --feedback

# 保持した結果を flash に書き込む(自動書き込みが発火しなかった場合のみ必要)。
# レスポンスの `success` フィールドを確認 —— 保持結果がなければ false になります。
ros2 service call /G100P_1/selfcal/commit std_srvs/srv/Trigger
```

探索が終わると、今回はキャリブレーターの outcome **に加えてライブ A/B
再チェック**で自ら決着します —— 再チェックは同一シーンで「新しい整合 vs 実行前の
整合」の深度充填率を測るため、判定はシーン依存の推測ではなく実際の前後を
反映します:

- **改善を検証** —— 再チェックが確認した `SUCCESS` が有効(`applied: true`)。
  `auto_commit_shift_px` が設定され `cy_shift_px` が到達すれば、今回は flash に
  **書き込み**(`committed: true`)。そうでなければ**有効なまま保持**されますが
  揮発性です —— 電源再投入後も残すには `selfcal/commit` を呼び出します。
- **既に最適**(`NO_CHANGE`)—— 変更・保持・書き込みは行われません。正常で
  健全な結果です。
- **悪化 / 検証不能 / 失敗** —— 再チェックが悪化と判定または確認できない場合
  (例:実行中にカメラが動いた)、あるいは `INSUFFICIENT_INPUT` / `TIMEOUT` /
  `FAILED` —— カメラは実行前の整合に**ロールバック**されます(`reverted: true`)。

`cy_shift_px` は実測の垂直整合シフトで、ハードウェアから直接読み取られ、自動
書き込みのゲートが使う値です。刻みは **0.25 px** に固定され、補正は
**5.0 px** で上限されます。そのため常に `0.25, 0.50, … 5.00` のいずれか
(動かなければ `0`)です。
`auto_commit_shift_px` はしきい値で、任意の値を取れますが、動作が変わるのは
これらの刻み境界だけで、刻みの中間値は次の刻みへ切り上げられます(`0.3` は
`0.5` と同じ）:

| `auto_commit_shift_px` | 効果 |
| --- | --- |
| `-1`(既定)| 自動書き込みしない —— 確認後に手動で書き込む。ゴールでこのフィールドを省略した場合に使われる値 |
| `0.25` | 実移動(1 ステップ以上)があれば書き込む —— **推奨** |
| `0.50`, `0.75`, … `5.00` まで | より大きなシフトを要求 |
| `> 5.0` | 発火しない(シフトは上限を超えられない） |

### アクションの Feedback

ゴールを `--feedback` 付きで送った場合、実行中は継続的に
ストリーミングされます:

| フィールド | 意味 |
| --- | --- |
| `phase` | `INITIAL_SEARCH` / `REFINEMENT` / `RECHECK` / `COMPLETED` |
| `progress` | 探索の進捗、`0.0`–`1.0` |
| `processed_frames` | これまでにキャリブレーターへ渡した深度フレーム数 |
| `valid_ratio_latest` | 最新フレームの充填率(結果が出てから値が入る）|

### アクションの Result

セッションを開始できなかった場合、結論に至らなかった場合、棄却した変更を
戻せなかった場合は goal が abort します。意図した revert は succeed です。
いずれの場合も詳細は `outcome` が持ちます。

実行が決着したときに一度だけ返されます —— そのセッションが何を計測し、
カメラに何を残したかの記録です:

| フィールド | 意味 |
| --- | --- |
| `outcome` | `SUCCESS` / `NO_CHANGE` / `INSUFFICIENT_INPUT`(シーンの有効深度が不足)/ `TIMEOUT`(時間内に収束しなかった)/ `FAILED` |
| `cy_shift_px` | 実測の垂直 cy シフト(px）—— 自動書き込みのしきい値の基準 |
| `recheck_verdict` | A/B 再チェック:`improved` / `worse` / `inconclusive` / `skipped` |
| `recheck_ratio_before` / `recheck_ratio_after` | 再チェック時の「実行前 / 収束後」整合の充填率 |
| `applied` | 補正がレジスタで有効 |
| `reverted` | 実行前のキャリブレーションへロールバック済み |
| `committed` | ユーザーキャリブレーション領域に書き込み済み |
| `message` | 結果の人間可読な要約 |
| `correction_level`、`valid_ratio_first` / `_latest` / `_delta` | 診断用のみ(書き込みの判定には使わない）|

セッション全体はストリーム内で実行されます:深度は発行され続け、後処理フィルタも
動作し続けます —— 実行がストリームを再起動することはありません。探索中は
キャリブレーターが各整合を試すにつれて深度品質が目に見えて変動し、収束後に
落ち着きます。セッションは**中断できません**(cancel は拒否され、制御セッター
および `pause` / `standby` / `hw_reset` も拒否します)—— 短時間で、結果が悪ければ
自動的に復元するため、中断すべきものは何もありません。

`commit` は flash に書き込む唯一のステップです。工場出荷時のキャリブレーション
はバックアップとして保持され、上書きされることはありません。**保持されたが
書き込まれていない**結果はカメラのレジスタにのみ存在し、電源再投入または
`~/hw_reset` で保存済みのキャリブレーションに戻ります。これにより「ライブ
ストリームでキャリブレーションするが flash には決して書き込まない」運用が
既定で安全になります —— ロボットがドッキングして静止しているときに実行する
だけです。

知っておくべき点が 2 つあります:

- **繰り返し実行すると積み重なります。** commit や電源再投入の前に
  `selfcal/run` をもう一度実行すると、flash からではなく前回の保持結果の
  続きから始まります。クリーンな基準が欲しい場合は先に `commit` するか
  電源を再投入してください。
- **非対応 / 実行中 → rejected であって failed ではありません。** セルフ
  キャリブレーションに対応していないモジュール、およびセッション実行中に
  送られた 2 つ目の `selfcal/run` は、どちらも即座に拒否されます ——
  `ros2 action send_goal` が「Goal was rejected」を報告し(`Result` は
  ありません)、理由は node log で確認します。

**プロセスごとに 1 セッション。** キャリブレーターはカメラハンドルをプロセス
グローバルに束縛するため、プロセスを共有するカメラ —— 1 つの
`ComposableNodeContainer` にある複数のドライバ —— は一度に 1 台ずつしか
キャリブレーションできません。2 つ目の `selfcal/run` は、最初の実行が決着する
まで拒否され node log にメッセージが残ります。別プロセスとして起動したカメラは
影響を受けません。プロセス内通信はこれとは直交します:キャリブレーターは深度
フレームを自前のコピーとして受け取るため、実行中も `use_intra_process_comms`
の配信は変わりません。

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
`/diagnostics` が `input_dropped = 0` を示す場合、損失は driver ではなく
DDS 伝送層で起きています。

サポートされる video mode では、画像 1 フレームのサイズが kernel
UDP socket と IP fragment reassembly の単発バーストで吸収できる
容量を超えます(`rgb8` 1280×720 ≈ 2.7 MB、典型的な point cloud は
最大 ~11 MB)。このため kernel 側バッファが溢れます —— x86 ホストが
速く送りすぎる場合も、ARM ホストが遅く読みすぎる場合も同じです。

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

