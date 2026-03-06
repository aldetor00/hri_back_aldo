from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, B1HandAction, RobotMode, B1HandIndex, GripperControlMode, Position, Orientation, Posture
import sys, time

def dance(client: B1LocoClient):
    client.ChangeMode(RobotMode.kPrepare)  # 先站稳
    time.sleep(1)
    client.ChangeMode(RobotMode.kCustom)   # 切换自定义模式
    client.SwitchHandEndEffectorControlMode(True)

    # 关键：自定义模式下主动保持站立
    client.Move(0, 0, 0)  # 或 client.StandUp()，具体看SDK
    time.sleep(0.5)

    # 手臂动作
    left_posture = Posture()
    left_posture.position = Position(0.7, 0.5, 0.3)
    left_posture.orientation = Orientation(-1.57, -1.57, 0.0)
    right_posture = Posture()
    right_posture.position = Position(0.7, -0.5, 0.3)
    right_posture.orientation = Orientation(-1.57, -1.57, 0.0)
    client.MoveHandEndEffectorV2(left_posture, 3000, B1HandIndex.kLeftHand)
    client.MoveHandEndEffectorV2(right_posture, 3000, B1HandIndex.kRightHand)
    time.sleep(2)

    print("回正")
    left_posture.position = Position(0.3, 0.0, 0.1)
    right_posture.position = Position(0.3, 0.0, 0.1)
    client.MoveHandEndEffectorV2(left_posture, 2000, B1HandIndex.kLeftHand)
    client.MoveHandEndEffectorV2(right_posture, 2000, B1HandIndex.kRightHand)
    time.sleep(2)

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactory.Instance().Init(0, sys.argv[1])

    client = B1LocoClient()
    client.Init()
    x, y, z, yaw, pitch = 0.0, 0.0, 0.0, 0.0, 0.0
    res = 0

    while True:
        need_print = False
        input_cmd = input().strip()
        if input_cmd:
            if input_cmd == "mp":
                res = client.ChangeMode(RobotMode.kPrepare)
            elif input_cmd == "md":
                res = client.ChangeMode(RobotMode.kDamping)
            elif input_cmd == "mw":
                res = client.ChangeMode(RobotMode.kWalking)
            elif input_cmd == 'mc':
                res = client.ChangeMode(RobotMode.kCustom)
            elif input_cmd == "stop":
                x, y, z = 0.0, 0.0, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "w":
                x, y, z = 0.8, 0.0, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "a":
                x, y, z = 0.0, 0.2, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "s":
                x, y, z = -0.2, 0.0, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "d":
                x, y, z = 0.0, -0.2, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "ld":
                client.LieDown()
            elif input_cmd == "gu":
                client.GetUp()
            elif input_cmd == "q":
                x, y, z = 0.0, 0.0, 0.2
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "e":
                x, y, z = 0.0, 0.0, -0.2
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "hd":
                yaw, pitch = 0.0, 1.0
                need_print = True
                res = client.RotateHead(pitch, yaw)
            elif input_cmd == "hu":
                yaw, pitch = 0.0, -0.3
                need_print = True
                res = client.RotateHead(pitch, yaw)
            elif input_cmd == "hr":
                yaw, pitch = -0.785, 0.0
                need_print = True
                res = client.RotateHead(pitch, yaw)
            elif input_cmd == "hl":
                yaw, pitch = 0.785, 0.0
                need_print = True
                res = client.RotateHead(pitch, yaw)
            elif input_cmd == "ho":
                yaw, pitch = 0.0, 0.0
                need_print = True
                res = client.RotateHead(pitch, yaw)
            elif input_cmd == "dance":
                dance(client)

            if need_print:
                print(f"Param: {x} {y} {z}")
                print(f"Head param: {pitch} {yaw}")

            if res != 0:
                print(f"Request failed: error = {res}")

if __name__ == "__main__":
    main()