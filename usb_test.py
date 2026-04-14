import usb.core
import usb.util
import struct
import threading
import sys
import os

# --- 環境設定 ---
WEBOTS_HOME = "/Applications/Webots.app"
os.environ['WEBOTS_HOME'] = WEBOTS_HOME
os.environ['WEBOTS_CONTROLLER_URL'] = 'UR5e'
sys.path.append(os.path.join(WEBOTS_HOME, "Contents/lib/controller/python"))
from controller import Robot

# --- SpaceMouse 設定 ---
VENDOR_ID = 0x256f
PRODUCT_ID = 0xc635
DEADZONE = 10
MAX_VAL = 350

# ゲイン設定
GAIN_LINEAR = 2.0
GAIN_ANGULAR = 1.0

# 初期姿勢（ホームポジション：ラジアン単位）
# UR5eが使いやすい「L字姿勢」になる角度の例です
HOME_POSITIONS = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

# グローバルで状態を管理
current_state = {
    "x": 0.0, "y": 0.0, "z": 0.0,
    "pitch": 0.0, "roll": 0.0, "yaw": 0.0,
    "buttons": [0, 0]
}

def parse_axis(low, high):
    value = struct.unpack('<h', bytes([low, high]))[0]
    return value if abs(value) >= DEADZONE else 0

def run_spacemouse():
    global current_state
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    if dev is None:
        print("Error: SpaceMouse not found.")
        return

    try:
        if dev.is_kernel_driver_active(0): dev.detach_kernel_driver(0)
        dev.set_configuration()
        endpoint = 0x81 

        while True:
            try:
                data = dev.read(endpoint, 16, timeout=100)
                if not data: continue
                report_id = data[0]

                if report_id == 1 and len(data) >= 7:
                    current_state["x"] = (parse_axis(data[1], data[2]) / MAX_VAL) * GAIN_LINEAR
                    current_state["y"] = (parse_axis(data[3], data[4]) * -1 / MAX_VAL) * GAIN_LINEAR
                    current_state["z"] = (parse_axis(data[5], data[6]) * -1 / MAX_VAL) * GAIN_LINEAR
                elif report_id == 2 and len(data) >= 7:
                    current_state["roll"]  = (parse_axis(data[1], data[2]) / MAX_VAL) * GAIN_ANGULAR
                    current_state["pitch"] = (parse_axis(data[3], data[4]) * -1 / MAX_VAL) * GAIN_ANGULAR
                    current_state["yaw"]   = (parse_axis(data[5], data[6]) * -1 / MAX_VAL) * GAIN_ANGULAR
                elif report_id == 3:
                    current_state["buttons"] = [data[1] & 0x01, (data[1] & 0x02) >> 1]
            except usb.core.USBError:
                continue
    finally:
        usb.util.dispose_resources(dev)

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    joint_names = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    ]
    motors = []
    for name in joint_names:
        motor = robot.getDevice(name)
        motor.setPosition(float('inf')) # 最初は速度制御モード
        motor.setVelocity(0.0)
        motors.append(motor)

    sm_thread = threading.Thread(target=run_spacemouse, daemon=True)
    sm_thread.start()

    print("--- Webots Extern Controller Started ---")
    print("右ボタンを押すと初期位置にリセットします")

    is_resetting = False

    while robot.step(timestep) != -1:
        # 右ボタン (buttons[1]) が押された時のリセット処理
        if current_state["buttons"][1] == 1 and not is_resetting:
            is_resetting = True
            print("\nResetting to Home Position...")
            for i in range(6):
                motors[i].setPosition(HOME_POSITIONS[i]) # 位置制御に切り替え
                motors[i].setVelocity(1.0) # リセット時の速度

        # リセット完了判定（全関節が目標角度に十分近づいたら速度制御に戻す）
        if is_resetting:
            # 各関節の現在位置を確認（簡易的に1秒待つ処理でもOKですが、ここではフラグ管理）
            # ボタンを離したら速度制御モードに復帰するようにします
            if current_state["buttons"][1] == 0:
                print("\nResuming SpaceMouse Control...")
                for i in range(6):
                    motors[i].setPosition(float('inf')) # 速度制御モードに復帰
                    motors[i].setVelocity(0.0)
                is_resetting = False
            continue

        # 通常のSpaceMouse操作（is_resettingがFalseの時のみ実行）
        motors[0].setVelocity(current_state["x"])
        motors[1].setVelocity(current_state["y"])
        motors[2].setVelocity(current_state["z"])
        motors[3].setVelocity(current_state["pitch"])
        motors[4].setVelocity(current_state["roll"])
        motors[5].setVelocity(current_state["yaw"])
        
        sys.stdout.write(f"\r[Terminal] X:{current_state['x']:.3f} B:{current_state['buttons']} ")
        sys.stdout.flush()

if __name__ == "__main__":
    main()