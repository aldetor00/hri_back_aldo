#include <array>
#include <chrono>
#include <iostream>
#include <thread>

#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/b1/b1_api_const.hpp>

static const std::string kTopicArmSDK = "rt/joint_ctrl";

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  booster::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  booster::robot::ChannelPublisherPtr<booster_interface::msg::LowCmd> arm_sdk_publisher;
  booster_interface::msg::LowCmd msg;

  arm_sdk_publisher.reset(
      new booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>(kTopicArmSDK));
  arm_sdk_publisher->InitChannel();

  std::array<booster::robot::b1::JointIndex, 4> right_arm_joints = {
      booster::robot::b1::JointIndex::kRightShoulderPitch,
      booster::robot::b1::JointIndex::kRightShoulderRoll,
      booster::robot::b1::JointIndex::kRightElbowPitch,
      booster::robot::b1::JointIndex::kRightElbowYaw};

  float weight = 0.f;
  float weight_rate = 0.2f;

  float kp = 60.f;
  float kd = 1.5f;
  float dq = 0.f;
  float tau_ff = 0.f;

  float control_dt = 0.02f;
  float max_joint_velocity = 1.0f;

  float weight_margin = weight_rate * control_dt;
  float max_joint_delta = max_joint_velocity * control_dt;
  auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

  for (size_t i = 0; i < booster::robot::b1::kJointCnt; i++) {
    booster_interface::msg::MotorCmd motor_cmd;
    msg.motor_cmd().push_back(motor_cmd);
  }

  // 初始化 current_jpos_des
  std::array<float, 4> current_jpos_des{};

  // 定义抱拳动作关节角（右臂）
  std::array<float, 4> hug_pose = {
      -0.6f,  // 右肩 Pitch：手臂往内收
      0.2f,   // 右肩 Roll：手臂向胸部靠拢
      0.8f,   // 右肘 Pitch：手臂弯曲
      0.3f    // 右肘 Yaw：肘部稍微向内转
  };

  // 定义冲拳目标位置（右臂）
  std::array<float, 4> punch_pose = {
      -1.2f,  // 向前打出
      0.0f,
      0.0f,
      0.0f
  };

  std::cout << "Press ENTER to start hug pose..." << std::endl;
  std::cin.get();

  std::cout << "Starting hug pose..." << std::endl;
  int hug_steps = static_cast<int>(2.0f / control_dt);
  for (int i = 0; i < hug_steps; ++i) {
    // 提升权重
    weight += weight_margin;
    weight = std::clamp(weight, 0.f, 0.5f);

    for (int j = 0; j < right_arm_joints.size(); ++j) {
      current_jpos_des[j] += std::clamp(hug_pose[j] - current_jpos_des[j],
                                        -max_joint_delta, max_joint_delta);

      msg.motor_cmd().at(int(right_arm_joints.at(j))).q(current_jpos_des.at(j));
      msg.motor_cmd().at(int(right_arm_joints.at(j))).dq(dq);
      msg.motor_cmd().at(int(right_arm_joints.at(j))).kp(kp);
      msg.motor_cmd().at(int(right_arm_joints.at(j))).kd(kd);
      msg.motor_cmd().at(int(right_arm_joints.at(j))).tau(tau_ff);
      msg.motor_cmd().at(int(right_arm_joints.at(j))).weight(weight);
    }

    arm_sdk_publisher->Write(&msg);
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Hug done! Press ENTER to punch." << std::endl;
  std::cin.get();

  std::cout << "Starting punch..." << std::endl;
  int punch_steps = static_cast<int>(1.5f / control_dt);
  for (int i = 0; i < punch_steps; ++i) {
    for (int j = 0; j < right_arm_joints.size(); ++j) {
      current_jpos_des[j] += std::clamp(punch_pose[j] - current_jpos_des[j],
                                        -max_joint_delta, max_joint_delta);

      msg.motor_cmd().at(int(right_arm_joints.at(j))).q(current_jpos_des.at(j));
      msg.motor_cmd().at(int(right_arm_joints.at(j))).dq(dq);
      msg.motor_cmd().at(int(right_arm_joints.at(j))).kp(kp);
      msg.motor_cmd().at(int(right_arm_joints.at(j))).kd(kd);
      msg.motor_cmd().at(int(right_arm_joints.at(j))).tau(tau_ff);
    }

    arm_sdk_publisher->Write(&msg);
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Punch complete!" << std::endl;
  return 0;
}
