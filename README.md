# SpaceMouse UR3e Controller

SpaceMouse Compact で UR3e をリアルタイム操作するコントローラです。Windows PC 上で Python をネイティブ実行します。

メインスクリプト: **`ur3e_controller.py`**

---

## 1. 仕組み

### 全体像

```
SpaceMouse Compact (USB, VID:PID = 256f:c635)
     │  pyusb + libusb-win32 で HID レポートを直接読む
     │  (3Dconnexion 純正ドライバは libusb と競合するので停止)
     ▼
Windows ネイティブ Python (.venv_win, Python 3.11)
     │  ur3e_controller.py
     │    ├─ SpaceMouse スレッド: 並進入力 (X/Y/Z) を TCP 速度に変換
     │    ├─ 関節ソフトリミット: 関節がリミット付近で自動減速
     │    └─ speedL: TCP速度ベクトルを 125 Hz で送信 (IK 不要)
     │
     │  TCP: ROBOT_IP:29999 / 30001-30004 (RTDE)
     ▼
UR3e コントローラ (実機 or URSim)
```

### 制御ループ

1. **初期化** — `moveJ` でホームポジション `[0, -1.57, 1.57, -1.57, -1.57, 0]` へ移動
2. **SpaceMouse スレッド** — USB エンドポイント `0x81` を 10 ms タイムアウトで読み続け、report ID=1 (並進) / ID=3 (ボタン) をパース。生値を `MAX_VAL` で正規化し `GAIN` を乗じて m/s 単位の TCP 速度として共有状態に書き込む
3. **制御ループ (125 Hz)**
   - 共有状態から `vx, vy, vz` を取得
   - `MAX_TCP_SPEED` でクリップ
   - `getActualQ()` で実際の関節角を取得し、リミット付近なら速度を 0 に向けてスケール
   - `speedL([vx, vy, vz, 0, 0, 0], acceleration, DT)` で送信
4. **右ボタン** — `speedStop` → `moveJ(HOME_JOINTS)` でリセット

### speedL について

SpaceMouse の入力は自然に Cartesian 速度 (X/Y/Z) なので、speedL に直接渡せます。IK 計算・関節角度の内部追跡が不要で、位置ドリフトや振動が発生しません。ロボット内部の Jacobian が関節速度への変換を担当します。

---

## 2. セットアップ

### 2.1 前提

- Windows 11
- Docker Desktop (URSim を使う場合)
- Python 3.11 (Python Launcher `py` で呼べる状態)
- [Zadig](https://zadig.akeo.ie/) — SpaceMouse を libusb-win32 ドライバに差し替える

### 2.2 SpaceMouse を libusb-win32 に差し替え (初回のみ)

1. Zadig を管理者で起動 → `Options` → `List All Devices`
2. `SpaceMouse Compact` (256f:c635) を選択
3. ドライバを `libusb-win32` にして `Replace Driver`

### 2.3 venv 作成とパッケージインストール

```powershell
cd C:\Users\kojum\robot_arm
py -3.11 -m venv .venv_win
.venv_win\Scripts\Activate.ps1

pip install numpy pyusb ur_rtde scipy scipy-openblas32
```

---

## 3. 実行手順 (毎回)

### 3.1 SpaceMouse の 3Dconnexion ドライバを停止

pyusb (libusb-win32) と競合するので一時的に止めます。

```powershell
Stop-Process -Name "3DxService","3DxWinCore" -Force -ErrorAction SilentlyContinue
```

### 3.2 URSim を使う場合: コンテナを起動

```powershell
docker run -d --name ursim `
  -p 6080:6080 -p 5900:5900 `
  -p 29999:29999 -p 30001:30001 -p 30002:30002 -p 30003:30003 -p 30004:30004 `
  universalrobots/ursim_e-series:latest
```

ブラウザで `http://localhost:6080/vnc.html` を開き Polyscope を起動:

1. ロボットの電源を ON → 初期化
2. 右上の **Remote Control** を ON にする

### 3.3 コントローラを起動

**URSim**

```powershell
.venv_win\Scripts\Activate.ps1
python ur3e_controller.py sim
```

**実機 UR3e**

```powershell
.venv_win\Scripts\Activate.ps1
python ur3e_controller.py real
```

> 実機の場合は Polyscope で Remote Control を ON にしてください。

期待される表示:

```
--- REAL モードで起動します ---
接続先ターゲット: 192.168.1.102
接続中... (192.168.1.102)
接続成功！
ホームポジションへ移動中...
ホームポジション到達
SpaceMouse を検索中...
SpaceMouse 検出: SpaceMouse Compact

TCP速度制御開始（speedL モード）
[TCP] X:-0.2976  Y:-0.1342  Z:-0.2515  v:[0.000,0.000,0.000]  accel:0.50  gain:0.100
```

---

## 4. 操作

| 入力 | 動作 |
|---|---|
| SpaceMouse 前後 | TCP を X 方向に並進 |
| SpaceMouse 左右 | TCP を Y 方向に並進 |
| SpaceMouse 上下 | TCP を Z 方向に並進 |
| 右ボタン | ホームポジションへ `moveJ` でリセット |
| ↑ キー | acceleration を +0.05 rad/s² (最大 5.0) |
| ↓ キー | acceleration を -0.05 rad/s² (最小 0.05) |
| → キー | gain を +0.01 (最大 0.50) |
| ← キー | gain を -0.01 (最小 0.01) |
| Ctrl+C | 終了 (`speedStop` + `stopScript`) |

姿勢 (回転) は固定。関節がソフトリミット付近に達すると TCP 速度が自動的にスケールダウンします。

### パラメータ (ur3e_controller.py 冒頭)

| 変数 | 既定値 | 意味 |
|---|---|---|
| `GAIN` | `0.1` | SpaceMouse 生値 → TCP 速度 (m/s) の倍率。→←キーで実行中に変更可 (0.01〜0.50、ステップ 0.01) |
| `MAX_TCP_SPEED` | `0.25` | TCP 速度の上限 (m/s) |
| `acceleration` | `0.5` | speedL の加速度 (rad/s²)。↑↓キーで実行中に変更可 (0.05〜5.0、ステップ 0.05) |
| `CONTROL_HZ` | `125` | 制御ループの周波数 (Hz) |
| `DEADZONE` | `10` | SpaceMouse 生値のデッドゾーン |
| `MAX_VAL` | `350` | SpaceMouse 生値の正規化最大値 |
| `JOINT_LIMIT_MARGIN` | `0.1` | ソフトリミット手前の減速開始マージン (rad) |

### 関節ソフトリミット

| 関節 | 範囲 (rad) |
|---|---|
| joint 2 (elbow) | 0.7 〜 2.3 |
| joint 4 (wrist 2) | -3.0 〜 6.28 |

---

## 5. トラブルシューティング

| 症状 | 原因 / 対処 |
|---|---|
| `インターフェース取得失敗` | 3Dconnexion ドライバが動いている。`Stop-Process -Name '3DxService','3DxWinCore' -Force` を実行 |
| `SpaceMouse が見つかりません` | Zadig の差し替えが未完了、または USB 未接続 |
| `No backend available` | pyusb が libusb を見つけられない。`pip install pyusb` し直すか Zadig やり直し |
| `Failed to start control script` | Polyscope で Remote Control が OFF / ロボット未初期化 |
| `Connection refused` (localhost) | URSim コンテナが起動していない。`docker ps` で確認 |
| 動作がガクつく | `.venv_win` ではなく WSL/Docker から動かしていないか確認。Windows ネイティブ実行が前提 |

---

## 6. ファイル構成

| パス | 役割 |
|---|---|
| `ur3e_controller.py` | メインスクリプト |
| `debug_spacemouse.py` | SpaceMouse の USB 入力を生データで確認するデバッグ用ツール |
| `.venv_win/` | Python 3.11 仮想環境 (コミット対象外) |
