# AIロボットによる資源分別

ROS 2 HumbleとGazebo Classicを使った、AI支援型の資源分別ロボットです。差動二輪ロボットが色付きパッケージをカメラで認識し、分類結果に応じた分別箱まで搬送します。

分類には、同梱のサンプルCSVからクラスごとの重心を計算する最近傍重心分類器を使用します。APIキー、外部ネットワークサービス、ダウンロードした学習済みモデルは不要です。色はこの試作環境で分類ラベルを表す目印であり、実際のごみを色だけで判定できることを示すものではありません。

## 提出物・プロジェクト構成

- `ros_ws/src/pa1_sortbot`：ROS 2パッケージ、Gazeboのワールド・ロボットモデル、起動ファイル、テスト
- `report.md`：課題の概要、AIの活用方法、実行結果、制約、今後の改善をまとめた日本語レポート
- `説明資料_PAI最終課題_MochiMomo.pdf`：検索可能な日本語PDF形式の説明資料
- `assignment_demo.mp4`：Gazeboで3つの荷物を分別する様子を収録した2分40秒の動画（H.264、1920×990）
- `docker-compose.yml`：授業用ROS 2 Humble環境の設定
- `scripts/generate_report.ps1`、`scripts/generate_report.py`：PDF生成スクリプト

PDFを再生成する場合は、プロジェクトのルートでPowerShellから実行します。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\generate_report.ps1
```

授業用DockerコンテナのReportLabを使って、日本語テキストを検索できるPDFを生成します。出力ファイル名は`説明資料_PAI最終課題_MochiMomo.pdf`です。

## Dockerでの実行方法

Docker Desktopを起動し、プロジェクトのルートで次を実行します。

```powershell
docker compose up -d
docker compose exec sim bash
```

コンテナ内でROS 2環境を読み込み、ビルド・テスト・実行します。

```bash
source /opt/ros/humble/setup.bash
cd /workspace/ros_ws
colcon build --symlink-install --packages-select pa1_sortbot
source install/setup.bash
colcon test --packages-select pa1_sortbot
ros2 launch pa1_sortbot sort_demo.launch.py
```

`http://localhost:6080/`でコース用デスクトップを開くと、Gazeboを確認できます。ロボットは3つの荷物に順番に近づいてカメラを向け、分類後に対応する分別箱まで運びます。箱への衝突を避けるため、箱の手前で停止し、Gazeboの`set_entity_state`サービスを使って仮想的につかんだ荷物を箱の中に配置します。荷物は接近を妨げないよう衝突判定のない視覚マーカーとして扱っています。この方式は実際の把持力や接触を再現するものではありません。

CPU性能に制約がある場合は、Gazeboの画面を表示せずに実行できます。

```bash
ros2 launch pa1_sortbot sort_demo.launch.py gui:=false
```

授業用イメージがローカルにない場合、Docker Composeが`airobotbook/ros2-desktop-ai-robot-book-humble:latest`を取得します。初回の取得には数分かかる場合があります。環境を終了するには、プロジェクトのルートで`docker compose down`を実行します。

## 動画について

同梱の`assignment_demo.mp4`には、Gazeboで3つすべての荷物を仕分ける様子が収録されています。Gazeboウィンドウを直接録画しており、デスクトップ上の余分な表示は含みません。YouTubeへのアップロードはまだ行っていません。提出時に動画リンクが必要な場合は、YouTubeに限定公開でアップロードしてください。

## テスト

分類器と移動制御の補助関数は、コース用コンテナ内で次のようにテストできます。

```bash
python3 -m unittest discover -s /workspace/ros_ws/src/pa1_sortbot/test -v
```

ROS統合テストには、ROS 2 HumbleとGazebo Classicを含むコース用イメージが必要です。課題の設計、実行結果、制約、改善案は`report.md`およびPDFの説明資料を参照してください。
