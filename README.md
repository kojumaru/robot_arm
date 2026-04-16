# SpaceMouse UR3e Webots Controller

SpaceMouse Compactを使用して、Webots上のUR3eロボットアームを直感的に操作するための外部コントローラ（Extern Controller）です。
Mac環境で実装しています。

## 1. セットアップ

### 依存ライブラリのインストール

USBデバイスに直接アクセスするために pyusb を使用します。

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Webots側の設定

1. Webotsを起動し、UR3eが含まれるワールドを開きます。
2. シーンツリーから **UR3e** を選択します。
3. **controller** フィールドを `<extern>` に書き換えます。
4. シミュレーションを **再生（または一時停止）** 状態にし、外部からの接続待ち状態（Waiting for local or remote connection...）にします。

## 2. 実行方法

USBデバイス権限とWebotsの通信パスを維持するため、以下の環境変数を指定して sudo で実行します。

### ur3e_ik.py の実行方法

逆運動学（IK）ソルバーを使用した位置制御。SpaceMouse の並進入力（X, Y, Z）によりエンドエフェクタの位置を直感的に操作でき、姿勢は自動的に保持されます。

```bash
sudo WEBOTS_HOME=/Applications/Webots.app WEBOTS_CONTROLLER_URL=UR3e USER=yoshidakouji ./.venv/bin/python ur3e_ik.py
```

**特徴:**

- 逆運動学により、指定した空間座標から関節角を自動計算
- SpaceMouse の並進のみを使用（回転は使用しない）
- エンドエフェクタの姿勢を固定したまま位置のみ制御

### 環境変数の詳細

| 変数                    | 説明                                                                                          |
| ----------------------- | --------------------------------------------------------------------------------------------- |
| `WEBOTS_HOME`           | Webots本体のインストールパス                                                                  |
| `WEBOTS_CONTROLLER_URL` | 接続先ロボットの name フィールドと一致させる必要があります（今回は `UR3e`）                   |
| `USER`                  | sudo 実行時にWebotsの通信パイプ（`/tmp/webots/ユーザー名`）が root にズレるのを防ぐために必須 |

## 3. 操作マニュアル

### 3D操作 (逆運動学による位置制御)

SpaceMouse の並進入力（X, Y, Z）を使用して、エンドエフェクタの位置を直感的に操作します。姿勢（回転）は固定されたまま、位置のみが制御されます。

**操作方法:**

- **前後 (X軸)**: アームを前後に移動させます
- **左右 (Y軸)**: アームを左右に移動させます
- **上下 (Z軸)**: アームを上下に移動させます

**重要:** ur3e_ik.py は逆運動学（IK）ソルバーを使用しており、指定した空間座標を達成するために各関節を自動的に計算します。そのため、物理的に到達不可能な位置が指定されると計算が失敗し、その移動は実行されません。

### リセット機能

**右ボタン (Button 1)**:

- 押すと、アームをプリセットされた **HOME POSITION**（L字姿勢）へ強制的に復帰させます。
- 全関節が初期値にリセットされます。

### パラメータ調整

**GAIN_IK** (ur3e_ik.py の28行目):

- SpaceMouse の入力感度を制御します（デフォルト: 0.005）
- 値を大きくすると、より少ないマウス動作で大きく移動します
- 値を小さくすると、より細かい調整が可能になります

**HOME_JOINTS** (ur3e_ik.py の40行目):

- リセット時のホームポジション（単位: ラジアン）
- 6要素の配列で各関節の角度を指定

## 4. デバッグツール

### usb_test.py

SpaceMouse の USB 接続を確認するためのデバッグツール：

```bash
sudo WEBOTS_HOME=/Applications/Webots.app WEBOTS_CONTROLLER_URL=UR3e USER=yoshidakouji ./.venv/bin/python usb_test.py
```

このツールは SpaceMouse が正しく認識されているか、入力値が正常に読み取られているか確認する際に使用します。

## 5. トラブルシューティング

| エラー                                   | 対処方法                                                                                                    |
| ---------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `ModuleNotFoundError`                    | `sys.path.append` が `from controller import Robot` より前に記述されているか確認してください                |
| `Access denied (USB)`                    | SpaceMouseのデバイスファイルへのアクセスには `sudo` が必須です                                              |
| `Cannot open directory /tmp/webots/root` | `USER=$(whoami)` 変数が正しく渡されているか、Webots本体が自分と同じユーザーで起動しているか確認してください |
