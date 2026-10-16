# AIロボットによる資源分別

## 課題概要

ROS 2 HumbleとGazebo Classicを用いた、カメラ認識型の差動二輪ロボットによる資源分別課題である。差動二輪ベースに3関節アームと平行グリッパーを搭載し、色付きボックスを認識して同色エリアへ搬送・配置する構成を実装した。色はシミュレーション内の分類ラベルを表す視覚マーカーであり、実物の廃棄物を色だけで材質判定するものではない。

分類器は同梱のRGBサンプルCSVからクラス別重心を計算する最近傍重心方式である。ROS 2/Gazeboのモデル状態を使って箱の位置を監視し、把持・配置の成否を状態機械で判定する。

## 提出物

| ファイル・ディレクトリ | 内容 |
|---|---|
| `ros_ws/src/pa1_sortbot` | ROS 2パッケージ、ロボットモデル、Gazeboワールド、起動ファイル、単体テスト |
| `report.md` | 課題目的、設計、検証結果、制約を記載した日本語レポート |
| `説明資料_PAI最終課題_MochiMomo.pdf` | `report.md`から生成した日本語PDF |
| `assignment_demo.mp4` | 旧方式の分別動作記録。現在のアーム把持・搬送は収録していない |

## 再現手順

授業用イメージ`airobotbook/ros2-desktop-ai-robot-book-humble:latest`を使用する。プロジェクトルートからコンテナを起動し、以下を実行する。

```powershell
docker compose up -d
docker compose exec sim bash
```

```bash
source /opt/ros/humble/setup.bash
cd /workspace/ros_ws
colcon build --symlink-install --packages-select pa1_sortbot
source install/setup.bash
colcon test --packages-select pa1_sortbot
ros2 launch pa1_sortbot sort_demo.launch.py gui:=false
```

Gazebo GUIを確認する場合は、起動コマンドの`gui:=false`を`gui:=true`にする。レポートPDFは`report.md`を原稿とし、コンテナ内で`python3 /workspace/project/scripts/generate_report.py`を実行して生成する。

## 検証結果

- ROS 2 Humble環境で`colcon build --symlink-install --packages-select pa1_sortbot`が成功。
- `colcon test --packages-select pa1_sortbot`は18件成功（アーム逆運動学7件、移動制御6件、画像分類5件）。
- Gazebo Classicのヘッドレス起動とカメラ分類は動作した。1品目を「plastic」と分類するところまで確認した。
- 物理把持試験では1品目目を持ち上げられず、ミッションは`Gripper did not pick up package 1`で停止した。3品の搬送・配置完了は確認できていない。

このため、現時点で成功が確認できた範囲はビルド、単体テスト、シミュレーション起動、色分類までである。アームの物理的把持と全品仕分けは未達であり、同梱動画も現行アーム版の実行記録ではない。
