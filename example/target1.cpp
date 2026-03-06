#include <array>
#include <chrono>
#include <iostream>
#include <thread>

#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_publisher.hpp>

using namespace booster::robot::b1;
static const std::string kTopicLowSDK = kTopicJointCtrl;

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

    booster::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    booster::robot::ChannelPublisherPtr<booster_interface::msg::LowCmd> low_sdk_publisher;
    booster_interface::msg::LowCmd msg;

    low_sdk_publisher.reset(
        new booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>(kTopicLowSDK));
    low_sdk_publisher->InitChannel();

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

    float kp = 160.f;
    float kd = 5.5f;
    float dq = 0.f;
    float tau_ff = 0.f;

    float control_dt = 0.02f;
    float max_joint_velocity = 0.5f;
    float max_joint_delta = max_joint_velocity * control_dt;
    auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

    msg.cmd_type(booster_interface::msg::CmdType::PARALLEL);

    // 【关键】定义扎马步的目标关节角度，可根据实际机器人关节范围微调
    std::array<float, 23> target_pos = {
        0.00, 0.00,          // 头部关节：保持水平
        0.10, -1.50, 0.00, -0.20,  // 左上肢：维持示例姿态
        0.10, 1.50, 0.00, 0.20,    // 右上肢：维持示例姿态
        0.0,                   // 腰部：稳定
        -0.5, 0.0, 0.0, -0.6, 0.2, 0.14,  // 左下肢：髋部前屈、膝部弯曲
        -0.5, 0.0, 0.0, -0.6, 0.2, -0.14  // 右下肢：对称姿态，与左下肢镜像（微调补偿）
    };

    std::array<float, 23> kps = {
        5., 5.,
        40., 50., 20., 10.,
        40., 50., 20., 10.,
        100., 
        350., 350., 180., 350., 550., 550.,
        350., 350., 180., 350., 550., 550.,
    };
    std::array<float, 23> kds = {
        .1, .1,
        .5, 1.5, .2, .2,
        .5, 1.5, .2, .2,
        5.0,
        7.5, 7.5, 3., 5.5, 1.5, 1.5,
        7.5, 7.5, 3., 5.5, 1.5, 1.5,
    };

    for (size_t i = 0; i < kJointCnt; i++) {
        booster_interface::msg::MotorCmd motor_cmd;
        msg.motor_cmd().push_back(motor_cmd);
    }

    std::cout << "Press ENTER to start ctrl ..." << std::endl;
    std::cin.get();

    std::cout << "Start low ctrl!" << std::endl;
    float period = 50000.f;
    int num_time_steps = static_cast<int>(period / control_dt);

    std::array<float, 23> current_jpos_des = {0.00, 0.00,
                                              0.10, -1.50, 0.00, -0.20,
                                              0.10, 1.50, 0.00, 0.20,
                                              0.0,
                                              -0.2, 0., 0., 0.4, -0.35, 0.03,
                                              -0.2, 0., 0., 0.4, -0.35, -0.03,};

    // 从初始姿态过渡到扎马步姿态
    for (int i = 0; i < num_time_steps; ++i) {
        for (int j = 0; j < current_jpos_des.size(); ++j) {
            current_jpos_des.at(j) += std::clamp(target_pos.at(j) - current_jpos_des.at(j),
                                                  -max_joint_delta, max_joint_delta);
        }

        for (int j = 0; j < current_jpos_des.size(); ++j) {
            msg.motor_cmd().at(int(low_joints.at(j))).q(current_jpos_des.at(j));
            msg.motor_cmd().at(int(low_joints.at(j))).dq(dq);
            msg.motor_cmd().at(int(low_joints.at(j))).kp(kps.at(j));
            msg.motor_cmd().at(int(low_joints.at(j))).kd(kds.at(j));
            msg.motor_cmd().at(int(low_joints.at(j))).tau(tau_ff);
        }

        low_sdk_publisher->Write(&msg);
        std::this_thread::sleep_for(sleep_time);
    }

    // 这里简单演示：维持 5 秒
    int hold_steps = static_cast<int>(5.0f / control_dt);
    for (int i = 0; i < hold_steps; ++i) {
        low_sdk_publisher->Write(&msg);
        std::this_thread::sleep_for(sleep_time);
    }

    // 停止控制（缓慢卸载力，让机器人回归稳定）
    std::cout << "Stoping low ctrl ...";
    float stop_time = 2.0f;
    int stop_time_steps = static_cast<int>(stop_time / control_dt);

    for (int i = 0; i < stop_time_steps; ++i) {
        low_sdk_publisher->Write(&msg);
        std::this_thread::sleep_for(sleep_time);
    }

    std::cout << "Done!" << std::endl;

    return 0;
}