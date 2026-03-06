from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, RobotMode, B1HandIndex, GripperControlMode, Position, Orientation, Posture
import sys, time

def pose_baoquan(client: B1LocoClient):
    print("\u6267\u884c\u62b1\u62f3\u52a8\u4f5c...")
    left_posture = Posture()
    left_posture.position = Position(0.25, 0.2, 0.25)
    left_posture.orientation = Orientation(-1.57, 0.0, 0.0)

    right_posture = Posture()
    right_posture.position = Position(0.25, -0.2, 0.25)
    right_posture.orientation = Orientation(-1.57, 0.0, 0.0)

    client.MoveHandEndEffectorV2(left_posture, 800, B1HandIndex.kLeftHand)
    client.MoveHandEndEffectorV2(right_posture, 800, B1HandIndex.kRightHand)

def pose_chongquan(client: B1LocoClient):
    print("\u6267\u884c\u5f13\u6b65\u51b2\u62f3\u52a8\u4f5c...")
    client.Move(0.2, 0.0, 0.0)
    time.sleep(0.5)

    right_posture = Posture()
    right_posture.position = Position(0.45, -0.15, 0.3)
    right_posture.orientation = Orientation(0.0, 0.0, 0.0)

    left_posture = Posture()
    left_posture.position = Position(0.15, 0.2, 0.25)
    left_posture.orientation = Orientation(0.0, 0.0, 0.0)

    client.MoveHandEndEffectorV2(right_posture, 1000, B1HandIndex.kRightHand)
    client.MoveHandEndEffectorV2(left_posture, 1000, B1HandIndex.kLeftHand)

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactory.Instance().Init(0, sys.argv[1])
    client = B1LocoClient()
    client.Init()

    while True:
        input_cmd = input().strip()
        if input_cmd == "mp":
            client.ChangeMode(RobotMode.kPrepare)
        elif input_cmd == "md":
            client.ChangeMode(RobotMode.kDamping)
        elif input_cmd == "mw":
            client.ChangeMode(RobotMode.kWalking)
        elif input_cmd == "mc":
            client.ChangeMode(RobotMode.kCustom)
        elif input_cmd == "stop":
            client.Move(0.0, 0.0, 0.0)
        elif input_cmd == "w":
            client.Move(0.8, 0.0, 0.0)
        elif input_cmd == "a":
            client.Move(0.0, 0.2, 0.0)
        elif input_cmd == "s":
            client.Move(-0.2, 0.0, 0.0)
        elif input_cmd == "d":
            client.Move(0.0, -0.2, 0.0)
        elif input_cmd == "q":
            client.Move(0.0, 0.0, 0.2)
        elif input_cmd == "e":
            client.Move(0.0, 0.0, -0.2)
        elif input_cmd == "baoquan":
            pose_baoquan(client)
        elif input_cmd == "chongquan":
            pose_chongquan(client)
        else:
            print("\u672a\u77e5\u6307\u4ee4", input_cmd)

if __name__ == "__main__":
    main()
