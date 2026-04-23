# SpaceMouse UR3e Controller (Windows ネイティブ)

SpaceMouse Compact で UR3e をリアルタイム操作するコントローラです。Windows PC 上で Python をネイティブ実行します。

- **デフォルト**: URSim (Docker コンテナ) に `localhost` で接続
- **実機**: 環境変数 `ROBOT_IP` に実機の IP アドレスを設定するだけで切り替え可能（コード改変不要）

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
     │    ├─ 入力: SpaceMouse の並進 (X/Y/Z) を共有状態に積算
     │    ├─ IK : ur_ikfast (C++ 拡張) で 6 関節角を算出
     │    └─ 出力: ur_rtde.servoJ で 125 Hz 送信
     │
     │  TCP: localhost:29999 / 30001-30004 (RTDE)
     ▼
Docker コンテナ: universalrobots/ursim_e-series
     ├─ Polyscope (UR の公式 GUI / VNC: http://localhost:6080/vnc.html)
     └─ 仮想 UR3e コントローラ (RTDE で指令を受ける)
```

Docker は **URSim の中だけ** で使います。コントローラ側を Windows ネイティブで動かす理由は、Docker の仮想 NIC を挟むと数十 ms のジッターが乗って servoJ (8 ms 周期) が不安定になるためです。

### 制御ループ (ur3e_controller.py)

1. **初期化** — `moveJ` で `HOME_JOINTS` (L 字姿勢) に移動し、そこでの FK から `current_pos` と `fixed_orientation` を保持
2. **SpaceMouse スレッド** — `usb.core.find` → エンドポイント `0x81` を 10 ms タイムアウトで読み続け、レポート ID=1 (並進) / ID=3 (ボタン) をパースして共有状態に積算
3. **IK ループ (125 Hz)**
   - 共有状態から `dx, dy, dz` を取り出して `current_pos` を更新 (姿勢は固定)
   - `ur_ikfast.inverse(..., q_guess=current_joints)` で 6 関節角を計算
   - 関節制限 (elbow: 0.7〜2.3、wrist2: -3.0〜6.28) でクリップ
   - 1 ステップで 0.3 rad 以上のジャンプは棄却 (IK 解の枝切り替わり対策)
   - 問題なければ `servoJ(q, 0.5, 0.5, dt=0.008, lookahead=0.03, gain=600)` で送信
   - 失敗時は `current_pos` をロールバック (位置を進めない)
4. **右ボタン** — `servoStop` → `moveJ(HOME_JOINTS)` でリセット

### ur_ikfast と LAPACK

`ur_ikfast` の自動生成コード (`ur3e_ikfast61.cpp`) は内部で **LAPACK** (`dgetrf_`, `dgeev_`, `dgetrs_` 等) を呼んで多項式固有値問題を解いています。Linux なら `apt install liblapack-dev` で済みますが、Windows にはシステム LAPACK が無いので、このリポジトリでは **`scipy-openblas32`** パッケージ同梱の OpenBLAS を流用しています。

- scipy-openblas32 はシンボルを `scipy_` プレフィックス付きで公開 (`scipy_dgetrf_` など)。
- `C:\tmp\ur_ikfast\ur3e\lapack_stubs.cpp` が **プレフィックス無しのシンボルを scipy\_\* に転送するトランポリン**になっている。
- `char*` 引数を取る関数 (`dgetrs_`, `dgeev_`) は gfortran ABI の末尾 strlen=1 を付けて渡す。
- `libscipy_openblas.dll` は site-packages 内にあり Windows の DLL 検索パスに入らないので、`ur3e_controller.py` の冒頭で `os.add_dll_directory(scipy_openblas32.get_lib_dir())` を呼んでから import する。

---

## 2. 初回セットアップ

### 2.1 前提

- Windows 11
- Docker Desktop (URSim を動かすため)
- Python 3.11 (Python Launcher `py` で呼べる状態。3.12 以降は未検証)
- Visual Studio 2026 Build Tools (`vcvars64.bat`, rc.exe, 14.50 MSVC toolset が入っていればよい) — ur_ikfast のビルドに必要
- [Zadig](https://zadig.akeo.ie/) — SpaceMouse を libusb-win32 ドライバに差し替える

### 2.2 SpaceMouse を libusb-win32 に差し替え (初回のみ)

1. Zadig を管理者で起動 → `Options` → `List All Devices`
2. `SpaceMouse Compact` (256f:c635) を選択
3. ドライバを `libusb-win32` にして `Replace Driver`
4. 3Dconnexion 純正アプリが入っている場合は競合するので、使う直前に停止する (下記「実行手順」参照)

### 2.3 venv 作成とパッケージインストール

```powershell
cd C:\Users\kojum\robot_arm
py -3.11 -m venv .venv_win
.venv_win\Scripts\Activate.ps1

pip install Cython numpy pyusb ur_rtde scipy scipy-openblas32 wheel setuptools
```

### 2.4 ur_ikfast のビルド (Windows 向け)

アップストリームの ur_ikfast は Linux 前提なので、このリポジトリ用にパッチを当ててビルドします。

```powershell
# ソースを取得 (初回のみ)
git clone https://github.com/cambel/ur_ikfast.git C:\tmp\ur_ikfast
cd C:\tmp\ur_ikfast
git checkout 85b8274
```

以下 3 ファイルをこのリポジトリの `patches/ur_ikfast/` に同梱しています。`C:\tmp\ur_ikfast\` にコピー (同名ファイルを上書き):

| patches/ur_ikfast/ | 配置先 |
|---|---|
| `setup.py` | `C:\tmp\ur_ikfast\setup.py` (上書き。`scipy-openblas32` の `.lib` を `library_dirs` / `libraries` に追加) |
| `ur3e/lapack_stubs.cpp` | `C:\tmp\ur_ikfast\ur3e\lapack_stubs.cpp` (新規。LAPACK → scipy\_\* トランポリン) |
| `build_win.bat` | `C:\tmp\ur_ikfast\build_win.bat` (新規。vcvars64 を読んでビルド) |

配置後、`C:\tmp\ur_ikfast\build_win.bat` をダブルクリック (または cmd から実行)。

> **注意**: build_win.bat は `DISTUTILS_USE_SDK=1` と `MSSdk=1` を set しています。これが無いと distutils が PATH を再構築して `LNK1158: cannot run rc.exe` で失敗します。

ビルドが通ったら動作確認:

```powershell
.venv_win\Scripts\python -c "import os, scipy_openblas32; os.add_dll_directory(scipy_openblas32.get_lib_dir()); from ur_ikfast import ur_kinematics as k; a=k.URKinematics('ur3e'); print(len(a.kinematics.inverse(list(a.forward([0,-1.57,1.57,-1.57,-1.57,0]))))//6)"
```

`8` と出れば成功。`0` は LAPACK がスタブのままになっている合図。

---

## 3. 実行手順 (毎回)

### 3.1 URSim コンテナを起動

```powershell
docker run -d --name ursim `
  -p 6080:6080 -p 5900:5900 `
  -p 29999:29999 -p 30001:30001 -p 30002:30002 -p 30003:30003 -p 30004:30004 `
  universalrobots/ursim_e-series:latest
```

ブラウザで `http://localhost:6080/vnc.html` を開き、Polyscope を起動:

1. ロボットの電源を ON → 初期化
2. 右上の **Remote Control** を ON にする

### 3.2 SpaceMouse の 3Dconnexion ドライバを停止

pyusb (libusb-win32) と競合するので一時的に止めます。

```powershell
Stop-Process -Name "3DxService","3DxWinCore" -Force -ErrorAction SilentlyContinue
```

### 3.3 コントローラを起動

**URSim (デフォルト)**

```powershell
.venv_win\Scripts\Activate.ps1
Remove-Item Env:ROBOT_IP -ErrorAction SilentlyContinue   # 前セッションの残骸を消す
python ur3e_controller.py
```

**実機 UR3e**

```powershell
.venv_win\Scripts\Activate.ps1
$env:ROBOT_IP = "192.168.x.x"   # 実機の IP アドレスに変更
python ur3e_controller.py
```

> 実機の場合、3.1 の URSim 起動は不要です。実機側で Polyscope を起動し Remote Control を ON にしてください。

期待される表示:

```
接続中... (localhost)      # 実機の場合は指定した IP が表示される
接続成功！
ホームポジションへ移動中...
ホームポジション到達
SpaceMouse を検索中...
SpaceMouse 検出: SpaceMouse Compact

IK 制御開始（姿勢固定モード）
[IK] X: 0.2981  Y: 0.1311  Z: 0.3039  J:[-0.00,-1.57,1.57,-1.57,-1.57,-0.00]
```

`接続中...` の括弧内が意図した接続先であること、**`SpaceMouse 検出`** が出ることを確認。

---

## 4. 操作

| 入力 | 動作 |
|---|---|
| SpaceMouse 前後 (X) | TCP を X 方向に並進 |
| SpaceMouse 左右 (Y) | TCP を Y 方向に並進 |
| SpaceMouse 上下 (Z) | TCP を Z 方向に並進 |
| 右ボタン | ホームポジションへ `moveJ` でリセット |
| Ctrl+C | 終了 (`servoStop` + `stopScript` で安全に切断) |

姿勢 (回転) は固定。到達不可・関節制限・急ジャンプが検出されるとそのステップをスキップします。

### パラメータ (ur3e_controller.py 冒頭)

| 変数 | 既定 | 意味 |
|---|---|---|
| `GAIN_IK` | `0.002` | SpaceMouse 入力 → 目標差分の倍率。大きいほど速く動く |
| `CONTROL_HZ` | `125` | `servoJ` の送信周期 (Hz) |
| `HOME_JOINTS` | `[0, -1.57, 1.57, -1.57, -1.57, 0]` | リセット時の関節角 (rad) |
| `DEADZONE` / `MAX_VAL` | `10` / `350` | SpaceMouse 生値のデッドゾーンと正規化最大値 |

### 接続先の切り替え (環境変数)

| 環境変数 | 既定 | 意味 |
|---|---|---|
| `ROBOT_IP` | `localhost` | 接続先ロボットの IP アドレス。URSim なら `localhost`、実機なら実際の IP を指定 |

コード改変不要。`$env:ROBOT_IP = "192.168.x.x"` を設定するだけで実機に切り替わります。

---

## 5. トラブルシューティング

| 症状 | 原因 / 対処 |
|---|---|
| `接続中... (host.docker.internal)` と出る | `ROBOT_IP` 環境変数が残っている。`Remove-Item Env:ROBOT_IP` |
| `SpaceMouse が見つかりません` | 3Dconnexion ドライバが掴んでいる or Zadig の差し替えが未完了。`3DxService`/`3DxWinCore` を停止し、それでもダメなら Zadig で libusb-win32 を再適用 |
| `No backend available` | pyusb が libusb を見つけられない。`pip install pyusb` し直すか Zadig やり直し |
| IK が `None` ばかり / 動かない | `libscipy_openblas.dll` が読めていない。`.venv_win` で実行しているか、`scipy-openblas32` が入っているか確認。§2.4 末尾の検証コマンドで `8` が出るか確認 |
| `Failed to start control script` | Polyscope で Remote Control が OFF / ロボット未初期化 |
| `Connection refused` (localhost) | URSim コンテナが起動していない。`docker ps` で確認。ポート 29999/30001-30004 がマップされているか |
| ビルド時 `LNK1158: rc.exe` | vcvars64 後に `DISTUTILS_USE_SDK=1` と `MSSdk=1` を set し忘れ (§2.4 参照) |
| 動作がガクつく | `.venv_win` ではなく `.venv` や WSL/Docker から動かしていないか (仮想 NIC ジッター)。本リポジトリのセットアップは Windows ネイティブ実行が前提 |

---

## 6. ファイル構成

| パス | 役割 |
|---|---|
| `ur3e_controller.py` | メインスクリプト (SpaceMouse → IK → RTDE)。`ROBOT_IP` 環境変数で URSim/実機を切り替え |
| `debug_spacemouse.py` | SpaceMouse が pyusb から見えるかだけ確認する小物 |
| `.venv_win/` | Python 3.11 仮想環境 (コミット対象外) |
| `Dockerfile` | 現状は URSim の起動を README で書いているため未使用だが、将来の CI 用に残している |
| `C:\tmp\ur_ikfast\` | ur_ikfast のソース (リポジトリ外。§2.4 でクローンする) |
