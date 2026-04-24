import sys
import os
import msvcrt
import usb.core
import usb.util
import struct
import threading
import time
import numpy as np

# ur_ikfast は scipy-openblas32 の libscipy_openblas.dll を必要とする。
# Windows では site-packages 内の DLL は自動検索されないので明示的に登録。
import scipy_openblas32
os.add_dll_directory(scipy_openblas32.get_lib_dir())

from ur_ikfast import ur_kinematics
import rtde_control
import rtde_receive
import argparse


# --- 接続設定 ---
parser = argparse.ArgumentParser(description="UR3e SpaceMouse Controller")
parser.add_argument("mode", choices=["real", "sim"], help="実行モード: 'real' (実機) または 'sim' (URsim)")
args = parser.parse_args()

# モードによるIPの分岐
if args.mode == "real":
    ROBOT_IP = "192.168.1.102"  # 実機の固定IP
else:
    ROBOT_IP = "localhost"      # Docker上のURSim用

print(f"--- {args.mode.upper()} モードで起動します ---")
print(f"接続先ターゲット: {ROBOT_IP}")

# --- SpaceMouse 設定 ---
VENDOR_ID  = 0x256f
PRODUCT_ID = 0xc635
DEADZONE = 10
MAX_VAL  = 350
GAIN_IK  = 0.01  # 入力感度（大きくすると動きが速くなる）
MAX_JOINT_SPEED = 3.14  # rad/s（UR3e 関節速度上限）

# --- UR3e / ur_ikfast 設定 ---
ur3e_arm = ur_kinematics.URKinematics('ur3e')

# ホームポジション（ラジアン）: 肘を直角にした L 字姿勢
HOME_JOINTS = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

# --- 制御周期 ---
CONTROL_HZ = 125          # servoJ の目標周波数
DT = 1.0 / CONTROL_HZ    # 約 0.008 秒

# --- SpaceMouse 共有状態 ---
current_state = {
    "x": 0.0, "y": 0.0, "z": 0.0,
    "buttons": [0, 0]
}
state_lock = threading.Lock()


def parse_axis(low, high):
    value = struct.unpack('<h', bytes([low, high]))[0]
    return value if abs(value) >= DEADZONE else 0


def run_spacemouse():
    """SpaceMouse の入力を別スレッドで読み続ける。"""
    global current_state
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
        except usb.core.USBError:
            pass
        endpoint = 0x81

        while True:
            try:
                data = dev.read(endpoint, 16, timeout=10)
                if data:
                    report_id = data[0]
                    if report_id == 1:
                        x = (parse_axis(data[1], data[2]) / MAX_VAL) * GAIN_IK
                        y = (parse_axis(data[3], data[4]) * -1 / MAX_VAL) * GAIN_IK
                        z = (parse_axis(data[5], data[6]) * -1 / MAX_VAL) * GAIN_IK
                        with state_lock:
                            current_state["x"] += x
                            current_state["y"] += y
                            current_state["z"] += z
                        if abs(x) + abs(y) + abs(z) > 0:
                            print(f"\n[SM] x={x:.4f} y={y:.4f} z={z:.4f} raw={list(data[:7])}")
                    elif report_id == 3:
                        with state_lock:
                            if data[1] & 0x01:
                                current_state["buttons"][0] = 1
                            if (data[1] & 0x02) >> 1:
                                current_state["buttons"][1] = 1
            except usb.core.USBError as e:
                if e.errno in [60, 110] or 'timeout' in str(e).lower():  # タイムアウトは無視（Windows: errno=None）
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
        print(f"ヒント: URSim コンテナが起動しているか確認してください。")
        print(f"       docker run ... -p 29999:29999 -p 30001:30001 ... universalrobots/ursim_e-series:latest")
        print(f"       接続先: ROBOT_IP={ROBOT_IP}")
        sys.exit(1)

    print("接続成功！")

    # ホームポジションへ移動
    print("ホームポジションへ移動中...")
    rtde_c.moveJ(HOME_JOINTS, speed=1, acceleration=1.5)
    print("ホームポジション到達")

    # IK 初期化: ホームポジションの FK から現在位置・姿勢を取得
    current_joints  = list(HOME_JOINTS)
    initial_matrix  = ur3e_arm.forward(current_joints, 'matrix')
    fixed_orientation = initial_matrix[:3, :3].copy()  # 姿勢を固定
    current_pos       = initial_matrix[:3, 3].copy()   # 現在の手先位置

    # SpaceMouse スレッド起動
    sm_thread = threading.Thread(target=run_spacemouse, daemon=True)
    sm_thread.start()

    print("\nIK 制御開始（姿勢固定モード・speedJ）")
    print("  SpaceMouse 並進: 手先を X/Y/Z 方向へ移動")
    print("  右ボタン      : ホームポジションへリセット")
    print("  ↑ / ↓        : 加速度 (acceleration) を増減 (1.0 〜 40.0 rad/s²)")
    print("  Ctrl+C        : 終了\n")

    acceleration = 10.0
    dx = dy = dz = 0.0

    try:
        while True:
            loop_start = time.time()

            # --- キーボードで acceleration を調整 ---
            if msvcrt.kbhit():
                key = msvcrt.getch()
                if key in (b'\xe0', b'\x00'):  # 拡張キープレフィックス
                    key2 = msvcrt.getch()
                    if key2 == b'H':  # ↑
                        acceleration = min(40.0, round(acceleration + 2.0, 1))
                        print(f"\n[Acceleration] {acceleration:.1f} rad/s²")
                    elif key2 == b'P':  # ↓
                        acceleration = max(1.0, round(acceleration - 2.0, 1))
                        print(f"\n[Acceleration] {acceleration:.1f} rad/s²")

            # --- 実機の関節角を読み戻して速度計算のズレを排除 ---
            actual_q = rtde_r.getActualQ()
            if actual_q:
                current_joints = list(actual_q)

            # --- 右ボタンでホームリセット（押した瞬間のみ）---
            with state_lock:
                btn_right = current_state["buttons"][1]
                current_state["buttons"][1] = 0
            if btn_right == 1:
                rtde_c.speedStop()
                rtde_c.moveJ(HOME_JOINTS, speed=1.5, acceleration=1.5)
                fixed_orientation = ur3e_arm.forward(HOME_JOINTS, 'matrix')[:3, :3].copy()
                print("\n[Home] ホームポジションへリセット")

            else:
                # --- SpaceMouse の差分を今フレームの目標位置に加算 ---
                with state_lock:
                    dx = current_state["x"]
                    dy = current_state["y"]
                    dz = current_state["z"]
                    current_state["x"] = current_state["y"] = current_state["z"] = 0.0
                current_pos[0] += dx
                current_pos[1] += dy
                current_pos[2] += dz

            # 目標行列を組み立て（姿勢は固定、位置のみ更新）
            target_matrix = np.eye(4, dtype=np.float64)
            target_matrix[:3, :3] = fixed_orientation
            target_matrix[:3, 3]  = current_pos

            # IK 計算
            ik_input   = target_matrix[:3, :]
            ik_results = ur3e_arm.inverse(ik_input, False, q_guess=current_joints)

            if abs(dx) + abs(dy) + abs(dz) > 0:
                print(f"\n[DBG] dx={dx:.4f} dy={dy:.4f} dz={dz:.4f}", end="")

            if ik_results is not None:
                ik_clipped = np.array(ik_results)
                # 関節制限（肘・手首）
                ik_clipped[2] = np.clip(ik_clipped[2], 0.7, 2.3)
                ik_clipped[4] = np.clip(ik_clipped[4], -3.0, 6.28319)

                # IK解のジャンプ検出（1ステップで0.3rad以上変化したら棄却）
                max_joint_delta = np.max(np.abs(ik_clipped - np.array(current_joints)))
                jump_ok = max_joint_delta < 0.3

                if jump_ok:
                    # IK差分を速度に変換して speedJ で送信（クリップ後の値をそのまま使用）
                    qd = (ik_clipped - np.array(current_joints)) / DT
                    qd = np.clip(qd, -MAX_JOINT_SPEED, MAX_JOINT_SPEED)
                    rtde_c.speedJ(qd.tolist(), acceleration, DT)
                else:
                    # IK解が飛びすぎ → 停止（次フレームで実機位置に自動リセット）
                    rtde_c.speedJ([0.0] * 6, acceleration, DT)
            else:
                if abs(dx) + abs(dy) + abs(dz) > 0:
                    print(f" IK=None", end="")
                # IK 解なし → 停止（次フレームで実機位置に自動リセット）
                rtde_c.speedJ([0.0] * 6, acceleration, DT)

            # ターミナル表示
            joint_str = ",".join(f"{a:.2f}" for a in current_joints)
            sys.stdout.write(
                f"\r[IK] X:{current_pos[0]:7.4f}  Y:{current_pos[1]:7.4f}  Z:{current_pos[2]:7.4f}"
                f"  J:[{joint_str}]  accel:{acceleration:.1f}rad/s²"
            )
            sys.stdout.flush()

            # 制御周期を維持
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
