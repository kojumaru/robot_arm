# SpaceMouse UR5e Webots Controller

SpaceMouse Compactを使用して、Webots上のUR5eロボットアームを直感的に操作するための外部コントローラ（Extern Controller）です。
Mac環境特有の権限問題と、Webotsライブラリのパス問題を解決済みです。

## 1. セットアップ

### 依存ライブラリのインストール

USBデバイスに直接アクセスするために pyusb を使用します。

```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Webots側の設定

1. Webotsを起動し、UR5eが含まれるワールドを開きます。
2. シーンツリーから **UR5e** を選択します。
3. **controller** フィールドを `<extern>` に書き換えます。
4. シミュレーションを **再生（または一時停止）** 状態にし、外部からの接続待ち状態（Waiting for local or remote connection...）にします。

## 2. 実行方法

USBデバイス権限とWebotsの通信パスを維持するため、以下の環境変数を指定して sudo で実行します。

```bash
sudo WEBOTS_HOME=/Applications/Webots.app \
     WEBOTS_CONTROLLER_URL=UR5e \
     USER=$(whoami) \
     ./venv/bin/python usb_test.py
```

### 環境変数の詳細

| 変数 | 説明 |
|------|------|
| `WEBOTS_HOME` | Webots本体のインストールパス |
| `WEBOTS_CONTROLLER_URL` | 接続先ロボットの name フィールドと一致させる必要があります（今回は `UR5e`） |
| `USER` | sudo 実行時にWebotsの通信パイプ（`/tmp/webots/ユーザー名`）が root にズレるのを防ぐために必須 |

## 3. 操作マニュアル

### 3D操作 (Velocity Control)

マウスの6自由度入力を各関節の角速度（rad/s）に割り当てています。

- **並進 (XYZ)**: 肩・肘の主要3関節を動かします。
- **回転 (RPY)**: 手首（Wrist 1-3）の姿勢を変化させます。

### リセット機能

**右ボタン (Button 1)**:
- 押している間、アームをプリセットされた **HOME POSITION**（L字姿勢）へ位置制御で自動復帰させます。
- ボタンを離すと、自動的にSpaceMouseでの自由操作モード（速度制御）に戻ります。

## 4. トラブルシューティング

| エラー | 対処方法 |
|--------|--------|
| `ModuleNotFoundError` | `sys.path.append` が `from controller import Robot` より前に記述されているか確認してください |
| `Access denied (USB)` | SpaceMouseのデバイスファイルへのアクセスには `sudo` が必須です |
| `Cannot open directory /tmp/webots/root` | `USER=$(whoami)` 変数が正しく渡されているか、Webots本体が自分と同じユーザーで起動しているか確認してください |
