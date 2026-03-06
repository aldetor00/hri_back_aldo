import sys
import time
import math

# 从SDK导入所有必要的类
from booster_robotics_sdk_python import (
    B1LocoClient,
    ChannelFactory,
    RobotMode,
    B1JointIndex,
    B1LowCmdPublisher,
    LowCmd,
    MotorCmd,
    LowCmdType
)

# --- 全局变量和常量 ---
ARM_KP = 150.0
ARM_KD = 4.0

# 初始化底层指令发布者
low_cmd_publisher = B1LowCmdPublisher()

# 定义串联关节及其在指令数组中的顺序
SERIAL_JOINT_MAP = {
    0: B1JointIndex.kHeadYaw, 1: B1JointIndex.kHeadPitch,
    2: B1JointIndex.kLeftShoulderPitch, 3: B1JointIndex.kLeftShoulderRoll,
    4: B1JointIndex.kLeftElbowPitch, 5: B1JointIndex.kLeftElbowYaw,
    6: B1JointIndex.kRightShoulderPitch, 7: B1JointIndex.kRightShoulderRoll,
    8: B1JointIndex.kRightElbowPitch, 9: B1JointIndex.kRightElbowYaw,
    10: B1JointIndex.kWaist,
}
NUM_SERIAL_JOINTS = len(SERIAL_JOINT_MAP)

# --- 最终版核心函数：增加了motor_cmd.mode的设置 ---
def move_joints(target_angles: dict, default_angles: dict = None):
    """
    通过底层指令，精确控制串联关节（手臂、头、腰）。
    """
    if default_angles is None:
        default_angles = {}

    low_cmd = LowCmd()
    low_cmd.cmd_type = LowCmdType.SERIAL

    motor_cmds = [MotorCmd() for _ in range(NUM_SERIAL_JOINTS)]

    for i in range(NUM_SERIAL_JOINTS):
        joint_name = SERIAL_JOINT_MAP[i]
        motor_cmd = motor_cmds[i]
        
        # 检查关节是否在本次控制的目标中
        if joint_name in target_angles or joint_name in default_angles:
            # === 关键中的关键：设置电机为位置控制模式 ===
            # '1' 是伺服/位置控制模式的一个非常通用的值
            motor_cmd.mode = 1
            # ==========================================
            
            motor_cmd.kp = ARM_KP
            motor_cmd.kd = ARM_KD
            
            # 优先使用本次动作的目标角度
            if joint_name in target_angles:
                motor_cmd.q = target_angles[joint_name]
            # 否则使用默认姿态的角度
            else:
                motor_cmd.q = default_angles[joint_name]
        else:
            # 对于本次完全不控制的关节，将其模式设为0（关闭），避免产生意外力矩
            motor_cmd.mode = 0
            motor_cmd.kp = 0
            motor_cmd.kd = 0
    
    low_cmd.motor_cmd = motor_cmds
    low_cmd_publisher.Write(low_cmd)

# --- 简化版的调试动作 ---
def perform_simple_test(client: B1LocoClient):
    print("--- Starting Final Debugging Test ---")

    # 切换到准备模式，这个模式下机器人站立但不会行走
    client.ChangeMode(RobotMode.kPrepare)
    print("Robot is in PREPARE mode. Body should be stationary.")
    time.sleep(2)

    # 定义一个基础姿态（手臂自然下垂）
    base_pose = {
        B1JointIndex.kHeadYaw: 0.0, B1JointIndex.kHeadPitch: 0.0,
        B1JointIndex.kLeftShoulderPitch: 0.8, B1JointIndex.kLeftShoulderRoll: 0.0,
        B1JointIndex.kLeftElbowPitch: 1.5, B1JointIndex.kLeftElbowYaw: 0.0,
        B1JointIndex.kRightShoulderPitch: 0.8, B1JointIndex.kRightShoulderRoll: 0.0,
        B1JointIndex.kRightElbowPitch: 1.5, B1JointIndex.kRightElbowYaw: 0.0,
        B1JointIndex.kWaist: 0.0,
    }
    
    print("\nStep 1: Moving arms to defined base pose.")
    move_joints(base_pose)
    time.sleep(3)
    print("--> Check: Did the arms move to a natural hanging pose?")

    # 动作1: 旋转左臂肩关节的Roll轴（向外抬起）
    print("\nStep 2: Rotating Left Shoulder Roll to 90 degrees outward.")
    left_roll_pose = {B1JointIndex.kLeftShoulderRoll: 1.57} # 1.57 约等于 pi/2
    move_joints(left_roll_pose, default_angles=base_pose)
    time.sleep(3)
    print("--> Check: Did the left arm raise sideways?")

    # 动作2: 旋转右臂肩关节的Pitch轴（向前抬起）
    print("\nStep 3: Rotating Right Shoulder Pitch to 90 degrees forward.")
    right_pitch_pose = {B1JointIndex.kRightShoulderPitch: -1.57}
    move_joints(right_pitch_pose, default_angles=base_pose)
    time.sleep(3)
    print("--> Check: Did the right arm raise forward?")

    # 结束: 回到初始姿态
    print("\nStep 4: Returning to base pose.")
    move_joints(base_pose)
    time.sleep(3)
    print("--> Check: Did both arms return to the hanging pose?")

    print("\n--- Test Finished ---")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        sys.exit(-1)
    ChannelFactory.Instance().Init(0, sys.argv[1])
    
    loco_client = B1LocoClient()
    loco_client.Init()
    
    low_cmd_publisher.InitChannel()
    time.sleep(1)
    
    try:
        perform_simple_test(loco_client)
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        print("Ensuring robot is stable before exiting.")
        # 发送一个让所有串联关节回归基础姿态的指令
        base_pose = {
             B1JointIndex.kLeftShoulderPitch: 0.8, B1JointIndex.kLeftElbowPitch: 1.5,
             B1JointIndex.kRightShoulderPitch: 0.8, B1JointIndex.kRightElbowPitch: 1.5,
        }
        move_joints({}, default_angles=base_pose)
        time.sleep(0.5)
        loco_client.ChangeMode(RobotMode.kPrepare)
        low_cmd_publisher.CloseChannel()
        print("Cleanup complete. Robot is in Prepare mode.")

if __name__ == "__main__":
    main()