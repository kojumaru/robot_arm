import sys
import os
import msvcrt
import usb.core
import usb.util
import struct
import threading
import time
import numpy as np

import scipy_openblas32
os.add_dll_directory(scipy_openblas32.get_lib_dir())

import rtde_control
import rtde_receive
import argparse


# --- 接続設定 ---
parser = argparse.ArgumentParser(description="UR3e SpaceMouse Controller")
parser.add_argument("mode", choices=["real", "sim"], help="実行モード: 'real' (実機) または 'sim' (URsim)")
args = parser.parse_args()

if args.mode == "real":
    ROBOT_IP = "192.168.1.102"
else:
    ROBOT_IP = "localhost"

print(f"--- {args.mode.upper()} モードで起動します ---")
print(f"接続先ターゲット: {ROBOT_IP}")

# --- SpaceMouse 設定 ---
VENDOR_ID  = 0x256f
PRODUCT_ID = 0xc635
DEADZONE   = 10
MAX_VAL    = 350
GAIN          = 0.1   # TCP速度ゲイン (m/s per 最大入力)
MAX_TCP_SPEED = 0.25   # m/s 上限

# --- 制御周期 ---
CONTROL_HZ = 125
DT = 1.0 / CONTROL_HZ

# ホームポジション（ラジアン）
HOME_JOINTS = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

# --- 関節ソフトリミット ---
# None = ロボット本体に任せる、(min, max) = ソフトウェアで減速
JOINT_LIMITS = [
    None,           # joint 0
    None,           # joint 1
    (0.7, 2.3),     # joint 2 (elbow)
    None,           # joint 3
    (-3.0, 6.28),   # joint 4 (wrist 2)
    None,           # joint 5
]
JOINT_LIMIT_MARGIN = 0.1  # rad: この余裕を切ったら減速開始


def compute_limit_scale(actual_q):
    """関節がリミットに近いほど 0 に近いスケールを返す。余裕十分なら 1.0。"""
    scale = 1.0
    for i, limits in enumerate(JOINT_LIMITS):
        if limits is None:
            continue
        dist = min(actual_q[i] - limits[0], limits[1] - actual_q[i])
        if dist < JOINT_LIMIT_MARGIN:
            scale = min(scale, max(0.0, dist / JOINT_LIMIT_MARGIN))
    return scale

# --- SpaceMouse 共有状態 ---
current_state = {
    "vx": 0.0, "vy": 0.0, "vz": 0.0,
    "buttons": [0, 0]
}
state_lock = threading.Lock()


def parse_axis(low, high):
    value = struct.unpack('<h', bytes([low, high]))[0]
    return value if abs(value) >= DEADZONE else 0


def run_spacemouse():
    print("SpaceMouse を検索中...")
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)

    if dev is None:
        print("!!! SpaceMouse が見つかりません。USB 接続を確認してください。 !!!")
        return

    print(f"SpaceMouse 検出: {dev.product}")

    try:
        for config in dev:
            for intf in config:
                ifnum = intf.bInterfaceNumber
                try:
                    if dev.is_kernel_driver_active(ifnum):
                        dev.detach_kernel_driver(ifnum)
                except Exception:
                    pass
        try:
            dev.set_configuration()
        except usb.core.USBError:
            pass
        try:
            usb.util.claim_interface(dev, 0)
        except usb.core.USBError as e:
            print(f"!!! インターフェース取得失敗 (3DxService が起動中の可能性あり): {e} !!!")
            print("!!! PowerShell で以下を実行してから再起動してください: !!!")
            print("!!!   Stop-Process -Name '3DxService','3DxWinCore' -Force -ErrorAction SilentlyContinue !!!")
            return
        endpoint = 0x81
        btn_held_until = 0.0

        while True:
            try:
                now = time.time()
                if now < btn_held_until:
                    with state_lock:
                        current_state["buttons"][1] = 1

                data = dev.read(endpoint, 16, timeout=10)
                if data:
                    report_id = data[0]
                    if report_id == 1:
                        # 生値を正規化してTCP速度に変換（-1.0〜1.0 × GAIN）
                        vx = -(parse_axis(data[1], data[2]) / MAX_VAL)
                        vy = -(parse_axis(data[3], data[4]) * -1 / MAX_VAL)
                        vz = (parse_axis(data[5], data[6]) * -1 / MAX_VAL)
                        with state_lock:
                            current_state["vx"] = vx
                            current_state["vy"] = vy
                            current_state["vz"] = vz
                    elif report_id == 3:
                        with state_lock:
                            if data[1] & 0x01:
                                current_state["buttons"][0] = 1
                            if (data[1] & 0x02) >> 1:
                                current_state["buttons"][1] = 1
                                btn_held_until = time.time() + 0.3
            except usb.core.USBError as e:
                if e.errno in [60, 110] or 'timeout' in str(e).lower():
                    continue
                print(f"\n[SpaceMouse] エラー: {e}")
                break
    except Exception as e:
        print(f"SpaceMouse スレッドエラー: {e}")
    finally:
        usb.util.dispose_resources(dev)


def main():
    print(f"接続中... ({ROBOT_IP})")

    try:
        rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    except Exception as e:
        print(f"接続失敗: {e}")
        sys.exit(1)

    print("接続成功！")

    print("ホームポジションへ移動中...")
    rtde_c.moveJ(HOME_JOINTS, speed=1, acceleration=1.5)
    print("ホームポジション到達")

    sm_thread = threading.Thread(target=run_spacemouse, daemon=True)
    sm_thread.start()

    print("\nTCP速度制御開始（speedL モード）")
    print("  SpaceMouse 並進: 手先を X/Y/Z 方向へ移動")
    print("  右ボタン      : ホームポジションへリセット")
    print("  ↑ / ↓        : 加速度を増減 (0.05 〜 5.0 rad/s²、ステップ 0.05)")
    print("  → / ←        : ゲインを増減 (0.01 〜 0.50、ステップ 0.01)")
    print("  Ctrl+C        : 終了\n")

    acceleration = 0.5
    gain = GAIN
    limit_scale = 0.01
    actual_q = None
    vx = vy = vz = 0.0

    try:
        while True:
            loop_start = time.time()

            # --- キーボードで acceleration を調整 ---
            if msvcrt.kbhit():
                key = msvcrt.getch()
                if key in (b'\xe0', b'\x00'):
                    key2 = msvcrt.getch()
                    if key2 == b'H':  # ↑
                        acceleration = min(5.0, round(acceleration + 0.05, 2))
                        print(f"\n[Acceleration] {acceleration:.2f} rad/s²")
                    elif key2 == b'P':  # ↓
                        acceleration = max(0.05, round(acceleration - 0.05, 2))
                        print(f"\n[Acceleration] {acceleration:.2f} rad/s²")
                    elif key2 == b'M':  # →
                        gain = min(0.5, round(gain + 0.01, 3))
                        print(f"\n[Gain] {gain:.3f}")
                    elif key2 == b'K':  # ←
                        gain = max(0.01, round(gain - 0.01, 3))
                        print(f"\n[Gain] {gain:.3f}")

            # --- 右ボタンでホームリセット ---
            with state_lock:
                btn_right = current_state["buttons"][1]
                current_state["buttons"][1] = 0
            if btn_right == 1:
                rtde_c.speedStop()
                rtde_c.moveJ(HOME_JOINTS, speed=1.5, acceleration=1.5)
                print("\n[Home] ホームポジションへリセット")
            else:
                # --- SpaceMouse の速度を読み取り ---
                with state_lock:
                    vx = current_state["vx"] * gain
                    vy = current_state["vy"] * gain
                    vz = current_state["vz"] * gain

                # 速度上限クリップ
                speed = np.sqrt(vx**2 + vy**2 + vz**2)
                if speed > MAX_TCP_SPEED:
                    scale = MAX_TCP_SPEED / speed
                    vx, vy, vz = vx * scale, vy * scale, vz * scale

                # 関節ソフトリミット：リミット手前で滑らかに減速
                actual_q = rtde_r.getActualQ()
                if actual_q:
                    limit_scale = compute_limit_scale(actual_q)
                    vx *= limit_scale
                    vy *= limit_scale
                    vz *= limit_scale

                # speedL: [vx, vy, vz, rx, ry, rz] 姿勢回転はゼロ固定
                rtde_c.speedL([vx, vy, vz, 0.0, 0.0, 0.0], acceleration, DT)

            # --- ステータス表示 ---
            tcp = rtde_r.getActualTCPPose()
            if tcp:
                limit_str = f"  lim:{limit_scale:.2f}" if actual_q and limit_scale < 1.0 else ""
                sys.stdout.write(
                    f"\r[TCP] X:{tcp[0]:7.4f}  Y:{tcp[1]:7.4f}  Z:{tcp[2]:7.4f}"
                    f"  v:[{vx:.3f},{vy:.3f},{vz:.3f}]  accel:{acceleration:.2f}  gain:{gain:.3f}{limit_str}"
                )
                sys.stdout.flush()

            elapsed = time.time() - loop_start
            sleep_time = DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n終了中...")
    finally:
        rtde_c.speedStop()
        rtde_c.stopScript()
        print("RTDE 接続を切断しました。")


if __name__ == "__main__":
    main()
