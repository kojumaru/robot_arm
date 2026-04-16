import sys
import os
import usb.core
import usb.util
import struct
import threading
import numpy as np

# --- Webots パス設定 ---
WEBOTS_HOME = "/Applications/Webots.app"
os.environ['WEBOTS_HOME'] = WEBOTS_HOME
os.environ['WEBOTS_CONTROLLER_URL'] = 'UR3e'
sys.path.append(os.path.join(WEBOTS_HOME, "Contents/lib/controller/python"))

try:
    from controller import Robot
    # ikpy の代わりに ur_ikfast をインポート
    from ur_ikfast import ur_kinematics
except ImportError as e:
    print(f"Import Error: {e}")
    sys.exit(1)

# --- SpaceMouse 設定 ---
VENDOR_ID = 0x256f
PRODUCT_ID = 0xc635
DEADZONE = 10
MAX_VAL = 350
GAIN_IK = 0.005 

current_state = {
    "x": 0.0, "y": 0.0, "z": 0.0,
    "buttons": [0, 0]
}

# --- UR3e / ur_ikfast 設定 ---
ur3e_arm = ur_kinematics.URKinematics('ur3e')

# 初期姿勢 (Rad)
# 肩を下向き、肘を直角にする等の初期値
HOME_JOINTS = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

def parse_axis(low, high):
    value = struct.unpack('<h', bytes([low, high]))[0]
    return value if abs(value) >= DEADZONE else 0

def run_spacemouse():
    global current_state
    print("Checking for SpaceMouse...")
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    
    if dev is None:
        print("!!! SpaceMouse not found. Check USB connection. !!!")
        return

    print(f"SpaceMouse found: {dev.product}")

    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
        dev.set_configuration()
        endpoint = 0x81
        
        print("Starting read loop...")
        while True:
            try:
                # タイムアウトを短く設定
                data = dev.read(endpoint, 16, timeout=10) 
                if data:
                    report_id = data[0]
                    # --- ここからが重要！ ---
                    if report_id == 1:
                        # スティックの平行移動成分を計算して current_state に代入
                        current_state["x"] = (parse_axis(data[1], data[2]) / MAX_VAL) * GAIN_IK
                        current_state["y"] = (parse_axis(data[3], data[4]) * -1 / MAX_VAL) * GAIN_IK
                        current_state["z"] = (parse_axis(data[5], data[6]) * -1 / MAX_VAL) * GAIN_IK
                    elif report_id == 3:
                        # ボタン入力を代入
                        current_state["buttons"] = [data[1] & 0x01, (data[1] & 0x02) >> 1]
                    # --- ここまで ---
            except usb.core.USBError as e:
                if e.errno in [60, 110]:
                    continue
                else:
                    print(f"\n[SpaceMouse] Error: {e}")
                    break
    except Exception as e:
        print(f"SpaceMouse Thread Error: {e}")
    finally:
        usb.util.dispose_resources(dev)

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    motors = [robot.getDevice(name) for name in joint_names]
    
    # 初期位置へ移動
    print("Moving to initial position...")
    for i in range(6):
        motors[i].setVelocity(1.0)
        motors[i].setPosition(HOME_JOINTS[i])

    for _ in range(int(2000 / timestep)):
        robot.step(timestep)

    # --- 逆運動学（IK）の準備 ---
    current_joints = list(HOME_JOINTS)
    # 最初の姿勢（真下向きなど）をFKで取得
    initial_matrix = ur3e_arm.forward(current_joints, 'matrix')
    
    # 【追加】回転成分（姿勢）だけを取り出して固定する
    fixed_orientation = initial_matrix[:3, :3].copy()
    # 現在の座標（位置）を保持
    current_pos = initial_matrix[:3, 3].copy()

    sm_thread = threading.Thread(target=run_spacemouse, daemon=True)
    sm_thread.start()

    print("\nIK Control Started (Orientation Fixed)")

    while robot.step(timestep) != -1:
        # --- 右ボタンでホーム（全関節リセット）に強制復帰 ---
        if current_state["buttons"][1] == 1:
            # 1. 関節の目標値を初期値に上書き
            for i in range(6):
                motors[i].setPosition(HOME_JOINTS[i])
            
            # 2. 現在保持している状態変数をすべて初期化
            current_joints = list(HOME_JOINTS)
            initial_matrix = ur3e_arm.forward(current_joints, 'matrix')
            current_pos = initial_matrix[:3, 3].copy()
            # 姿勢も初期位置のものに更新（もしズレていた場合のため）
            fixed_orientation = initial_matrix[:3, :3].copy()
            
            # 3. 移動量もクリア
            dx, dy, dz = 0.0, 0.0, 0.0
            
            print("\n[Home] Resetting all joints to HOME_JOINTS")
        
        else:
            # --- 通常のSpaceMouse操作 ---
            dx, dy, dz = current_state["x"], current_state["y"], current_state["z"]
            current_pos[0] += dx
            current_pos[1] += dy
            current_pos[2] += dz

        # SpaceMouseの状態リセット
        current_state["x"], current_state["y"], current_state["z"] = 0.0, 0.0, 0.0

        # 行列の組み立て
        target_matrix = np.eye(4, dtype=np.float64)
        target_matrix[:3, :3] = fixed_orientation
        target_matrix[:3, 3] = current_pos

        # IK計算（ホームボタンを押した直後は計算をスキップしても良いですが、整合性のために流します）
        ik_input = target_matrix[:3, :]
        ik_results = ur3e_arm.inverse(ik_input, False, q_guess=current_joints)

        if ik_results is not None:
            # クリップ前の値を保存
            ik_clipped = np.array(ik_results)
            # 第三関節（elbow_joint, インデックス2）の制限: 0.7 ~ 2.3
            ik_clipped[2] = np.clip(ik_clipped[2], 0.7, 2.3)
            # 第五関節（wrist_2_joint, インデックス4）の最低値: -3
            ik_clipped[4] = np.clip(ik_clipped[4], -3, 6.28319)

            # クリップが必要だった場合は移動をスキップ
            if np.allclose(ik_clipped, ik_results):
                for i in range(6):
                    motors[i].setVelocity(2.0)
                    motors[i].setPosition(ik_clipped[i])
                current_joints = list(ik_clipped)
            else:
                # クリップが必要 = 到達不可能な位置 → 位置を戻す
                current_pos[0] -= dx
                current_pos[1] -= dy
                current_pos[2] -= dz
        else:
            # IK 計算失敗時は位置を戻す
            current_pos[0] -= dx
            current_pos[1] -= dy
            current_pos[2] -= dz

        # シンプルな出力
        joint_str = ",".join(f"{angle:.2f}" for angle in current_joints)
        sys.stdout.write(f"\r[IK] X:{current_pos[0]:7.4f}  Y:{current_pos[1]:7.4f}  Z:{current_pos[2]:7.4f}  J:[{joint_str}] ")
        sys.stdout.flush()

if __name__ == "__main__":
    main()