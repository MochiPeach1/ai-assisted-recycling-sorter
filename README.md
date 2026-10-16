# AIロボットによる資源分別

## 課題概要

ROS 2 HumbleとGazebo Classicを用いた、カメラ認識型の移動マニピュレータによる資源分別課題である。差動二輪ベースに3関節アーム（肩・肘・手首）と平行グリッパーを搭載し、次の流れを自律的に実行する。

1. 箱の手前へ移動し、カメラ画像から箱の種類（paper / plastic / metal）を分類する。
2. 逆運動学でアームを箱の上へ伸ばし、グリッパーを下ろして閉じ、箱を持ち上げる。
3. 箱を保持したまま、分類結果と同じ色のエリアへ差動二輪で移動する。
4. アームで箱をエリア上へ下ろし、グリッパーを開いて置く。

3個の箱すべてについて上記を繰り返す。色はシミュレーション内の分類ラベルを表す視覚マーカーであり、実物の廃棄物を色だけで材質判定するものではない。

## 提出物

| ファイル・ディレクトリ | 内容 |
|---|---|
| `ros_ws/src/pa1_sortbot` | ROS 2パッケージ（ロボットモデル、Gazeboワールド、起動ファイル、ミッションノード、単体テスト） |
| `arm_sorting_demo.mp4` | Gazebo上で3個の箱を把持・搬送・配置した実行記録（俯瞰カメラ、早送り） |
| `report.md` | 課題目的、設計、検証結果、制約を記載した日本語レポート |
| `説明資料_PAI最終課題_MochiMomo.pdf` | `report.md`から生成した日本語PDF |
| `scripts/demo_camera.sdf`, `scripts/record_demo.py` | 動画収録用の俯瞰カメラと録画スクリプト |

## 使用したAI

- **ロボット内のAI（認識）**：同梱のRGBサンプルCSVからクラス別重心を学習する最近傍重心分類器（自作、外部API・事前学習済みモデルは不使用）。カメラ画像中央の画素を分類し、5フレーム中4フレーム以上で一致したラベルを採用する。

## 把持の実現方法

アーム関節はGazebo標準の`libgazebo_ros_joint_pose_trajectory.so`で位置指令しているが、このプラグインは関節位置を運動学的に設定するため、指が箱を締め付ける力（摩擦把持力）が物理的に発生しない。実際に試験したところ、指を閉じても箱は持ち上がらなかった。

そこで次の「把持ロック」を実装した（`sorter_node.py`の`secure_package`）。

1. グリッパーを閉じた後、Gazeboの実際の箱の位置を手首リンク座標系へ変換し、箱が左右の指の間（許容差 前後5 cm・左右4 cm・上下5 cm）にあるかを検証する。
2. 指の間にない場合はミッションを失敗として停止する。
3. 指の間にある場合のみ、`/gazebo/set_entity_state`で箱を手首に対する相対姿勢に50 Hzで追従させ、グリッパーを開いた時点で解放する。解放後の箱は重力で落下し、分別エリア上に載る。

つまり「箱が指の間にあるか」「配置後に箱がエリア上にあるか」はシミュレーション上の実際の位置で判定しているが、保持力そのものはシミュレーション上の補助で代替している。実機では、指の力制御または吸着機構で置き換える必要がある。

## 再現手順

授業用イメージ`airobotbook/ros2-desktop-ai-robot-book-humble:latest`を使用する。プロジェクトルートでコンテナを起動する。

```powershell
docker compose up -d
docker compose exec sim bash
```

```bash
source /opt/ros/humble/setup.bash
cd /workspace/ros_ws
colcon build --symlink-install --packages-select pa1_sortbot
source install/setup.bash
colcon test --packages-select pa1_sortbot && colcon test-result --verbose
ros2 launch pa1_sortbot sort_demo.launch.py gui:=false
```

進行状況は`ros2 topic echo /sortbot/sort_status`で確認でき、最後に`All packages sorted successfully`が出力される。Gazebo GUIで確認する場合は`gui:=true`とする。

動画を収録する場合は、起動後に別端末で以下を実行する。

```bash
cd /workspace/project
ros2 run gazebo_ros spawn_entity.py -entity demo_overview_camera -file scripts/demo_camera.sdf
python3 scripts/record_demo.py arm_sorting_demo.mp4
```

レポートPDFは`python3 /workspace/project/scripts/generate_report.py`で`report.md`から生成する。

## 検証結果（2026年10月16日、授業用コンテナ）

- `colcon build`：成功。
- `colcon test`：18件すべて成功（アーム逆運動学7件、移動制御6件、画像分類5件）。
- Gazebo上の全工程試験：3回連続で`All packages sorted successfully`まで完了。最終時点の箱の位置（Gazeboのモデル状態）は以下の通りで、すべて同色エリア（中心から約4 cm以内、エリア上面の高さ）に置かれた。

| 箱 | 分類結果 | 最終位置 (x, y, z) [m] | エリア中心 (x, y) |
|---|---|---|---|
| sample_1（青） | plastic | (1.292, 0.023, 0.120) | (1.25, 0.00) |
| sample_2（黄） | paper | (1.301, 0.740, 0.120) | (1.25, 0.72) |
| sample_3（赤） | metal | (1.289, -0.738, 0.120) | (1.25, -0.72) |

## 制約

- 色分類はシミュレーション用の単純な色ラベル分類であり、実物ごみの材質・形状・汚れは識別しない。
- 把持力は上記の把持ロックで代替しており、接触力による物理把持ではない。
- 移動はオドメトリのみに基づくため、箱の位置と配置は既知の座標を前提としている。