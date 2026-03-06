#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>  // 用于三角函数计算

#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_publisher.hpp>

#define M_PI 3.14
// 抱拳控制示例
// 执行流程：准备姿势 -> 缓慢过渡到抱拳姿势 -> 保持姿势 -> 恢复准备姿势
static const std::string kTopicLowSDK = booster::robot::b1::kTopicJointCtrl;
using namespace booster::robot::b1;

// 弧度转角度（调试用）
constexpr float rad2deg(float rad) { return rad * 180.0f / M_PI; }
// 角度转弧度
constexpr float deg2rad(float deg) { return deg * M_PI / 180.0f; }

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  booster::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  booster::robot::ChannelPublisherPtr<booster_interface::msg::LowCmd>
      low_sdk_publisher;
  booster_interface::msg::LowCmd msg;

  low_sdk_publisher.reset(
      new booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>(
          kTopicLowSDK));
  low_sdk_publisher->InitChannel();

  // 所有关节索引
  std::array<JointIndex, 23> low_joints = {
      JointIndex::kHeadYaw, JointIndex::kHeadPitch,
      JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
      JointIndex::kLeftElbowPitch,    JointIndex::kLeftElbowYaw,
      JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
      JointIndex::kRightElbowPitch,   JointIndex::kRightElbowYaw,
      JointIndex::kWaist,
      JointIndex::kLeftHipPitch, JointIndex::kLeftHipRoll, JointIndex::kLeftHipYaw,
      JointIndex::kLeftKneePitch, JointIndex::kCrankUpLeft, JointIndex::kCrankDownLeft,
      JointIndex::kRightHipPitch, JointIndex::kRightHipRoll, JointIndex::kRightHipYaw,
      JointIndex::kRightKneePitch, JointIndex::kCrankUpRight, JointIndex::kCrankDownRight
  };

  // 控制参数
  float control_dt = 0.01f;  // 更高的控制频率，使动作更平滑
  float max_joint_velocity = 0.3f;  // 动作需要更缓慢
  float transition_time = 3.0f;     // 从准备姿势到抱拳的过渡时间
  float hold_time = 2.0f;           // 保持抱拳姿势的时间
  
  float max_joint_delta = max_joint_velocity * control_dt;
  auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt * 1000));
  
  // 使用并行控制模式
  msg.cmd_type(booster_interface::msg::CmdType::PARALLEL);

  // 初始化电机命令
  for (size_t i = 0; i < booster::robot::b1::kJointCnt; i++) {
    booster_interface::msg::MotorCmd motor_cmd;
    msg.motor_cmd().push_back(motor_cmd);
  }

  // 关节PID参数
  std::array<float, 23> kps = {
      5., 5.,                                 // 头部
      40., 50., 20., 10.,                     // 左手臂
      40., 50., 20., 10.,                     // 右手臂
      100.,                                    // 腰部
      400., 350., 200., 400., 600., 600.,     // 左腿
      300., 250., 150., 300., 400., 400.      // 右腿
  };
  std::array<float, 23> kds = {
      .1, .1,                                  // 头部
      .5, 1.5, .2, .2,                        // 左手臂
      .5, 1.5, .2, .2,                        // 右手臂
      5.0,                                     // 腰部
      8.0, 7.5, 4.0, 7.0, 2.0, 2.0,           // 左腿
      6.0, 5.5, 3.0, 5.0, 1.5, 1.5            // 右腿
  };

  // 初始准备姿势
  std::array<float, 23> initial_pos = { 
      0.00,  0.00,                             // 头部：正视前方
      0.10, -0.30,  0.50, -0.20,              // 左手臂：自然弯曲
      0.10,  0.30,  0.50,  0.20,              // 右手臂：自然弯曲
      0.0,                                     // 腰部：直立
      -0.1, 0.05, 0.0, 0.2, -0.1, 0.1,        // 左腿：微弯站立
      -0.1, -0.05, 0.0, 0.2, -0.1, 0.1         // 右腿：微弯站立
  };

  // 抱拳姿势，假设左手高右手低摆在胸前
  std::array<float, 23> fist_clasp_pos = { 
      0.00,  0.05,                             // 头部：略微低头看前方
      1.0, -0.5,  1.2, -0.3,                   // 左手臂：高位置，抬起到胸前
      0.5,  0.3,  0.8,  0.2,                   // 右手臂：低位置，摆到胸前
      -0.05,                                    // 腰部：略微前倾
      -0.1, 0.05, 0.0, 0.2, -0.1, 0.1,        // 左腿：保持微弯站立
      -0.1, -0.05, 0.0, 0.2, -0.1, 0.1         // 右腿：保持微弯站立
  };

  // 等待用户开始
  std::cout << "请确保机器人处于'准备'模式" << std::endl;
  std::cout << "按ENTER键开始抱拳演示..." << std::endl;
  std::cin.get();

  // 初始化当前位置为初始姿势
  std::array<float, 23> current_pos = initial_pos;

  // 计算过渡步骤数
  int transition_steps = static_cast<int>(transition_time / control_dt);
  int hold_steps = static_cast<int>(hold_time / control_dt);

  std::cout << "开始执行抱拳动作..." << std::endl;

  // 阶段1：从初始姿势过渡到抱拳姿势
  for (int i = 0; i < transition_steps; ++i) {
    // 计算当前步骤的目标位置（线性插值）
    float ratio = static_cast<float>(i) / transition_steps;
    for (int j = 0; j < current_pos.size(); ++j) {
      float target = initial_pos[j] + ratio * (fist_clasp_pos[j] - initial_pos[j]);
      // 平滑过渡，限制最大关节移动量
      current_pos[j] += std::clamp(target - current_pos[j], -max_joint_delta, max_joint_delta);
    }
  
    // 设置关节命令
    for (int j = 0; j < current_pos.size(); ++j) {
      auto& cmd = msg.motor_cmd().at(int(low_joints.at(j)));
      cmd.q(current_pos[j]);
      cmd.dq(0.0f);
      cmd.kp(kps.at(j));
      cmd.kd(kds.at(j));
      cmd.tau(0.0f);
    }
  
    // 发布命令
    low_sdk_publisher->Write(&msg);
    std::this_thread::sleep_for(sleep_time);
  }

  // 阶段2：保持抱拳姿势
  std::cout << "保持抱拳姿势..." << std::endl;
  for (int i = 0; i < hold_steps; ++i) {
    // 微小的晃动，增加真实感
    for (int j = 0; j < current_pos.size(); ++j) {
      // 只在手臂关节添加微小扰动
      if ((j >= 2 && j <= 5) || (j >= 6 && j <= 9)) {
        float small_oscillation = 0.005f * sinf(2 * M_PI * 0.5f * i * control_dt);
        current_pos[j] = fist_clasp_pos[j] + small_oscillation;
      } else {
        current_pos[j] = fist_clasp_pos[j];
      }
    }
  
    // 设置关节命令
    for (int j = 0; j < current_pos.size(); ++j) {
      auto& cmd = msg.motor_cmd().at(int(low_joints.at(j)));
      cmd.q(current_pos[j]);
      cmd.dq(0.0f);
      cmd.kp(kps.at(j));
      cmd.kd(kds.at(j));
      cmd.tau(0.0f);
    }
  
    // 发布命令
    low_sdk_publisher->Write(&msg);
    std::this_thread::sleep_for(sleep_time);
  }

  // 阶段3：从抱拳恢复到初始姿势
  std::cout << "恢复初始姿势..." << std::endl;
  for (int i = 0; i < transition_steps; ++i) {
    float ratio = 1.0f - static_cast<float>(i) / transition_steps;
    for (int j = 0; j < current_pos.size(); ++j) {
      float target = initial_pos[j] + ratio * (fist_clasp_pos[j] - initial_pos[j]);
      current_pos[j] += std::clamp(target - current_pos[j], -max_joint_delta, max_joint_delta);
    }
  
    // 设置关节命令
    for (int j = 0; j < current_pos.size(); ++j) {
      auto& cmd = msg.motor_cmd().at(int(low_joints.at(j)));
      cmd.q(current_pos[j]);
      cmd.dq(0.0f);
      cmd.kp(kps.at(j));
      cmd.kd(kds.at(j));
      cmd.tau(0.0f);
    }
  
    // 发布命令
    low_sdk_publisher->Write(&msg);
    std::this_thread::sleep_for(sleep_time);
  }

  // 停止控制
  std::cout << "动作完成，停止控制..." << std::endl;
  float stop_time = 2.0f;
  int stop_steps = static_cast<int>(stop_time / control_dt);

  for (int i = 0; i < stop_steps; ++i) {
    low_sdk_publisher->Write(&msg);
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "演示结束！" << std::endl;
  return 0;
}