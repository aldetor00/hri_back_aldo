#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <algorithm>
#include <cmath>

#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/idl/b1/LowState.h>

using namespace booster::robot;
using namespace booster_interface::msg;

// Variabili globali per i sensori
std::array<float, 29> current_real_pos;
bool first_state_received = false;

// Handler per ricevere i dati dai sensori
void LowStateHandler(const void *msg) {
    const LowState *low_state_msg = static_cast<const LowState *>(msg);
    
    // Accediamo ai motori paralleli (braccia e gambe)
    auto parallel_motors = low_state_msg->motor_state_parallel();
    
    // Salviamo la posizione attuale (q) per la sincronizzazione
    for (size_t i = 0; i < parallel_motors.size() && i < 29; i++) {
        current_real_pos.at(i) = parallel_motors[i].q();
    }
    first_state_received = true;
}

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface (es. lo o eth0)" << std::endl;
        return -1;
    }

    // 1. Inizializzazione Factory (Fondamentale per evitare Segmentation Fault)
    ChannelFactory::Instance()->Init(0, argv[1]);

    // 2. Configurazione Subscriber
    ChannelSubscriber<LowState> state_subscriber("rt/low_state", LowStateHandler);
    state_subscriber.InitChannel();

    std::cout << "Sincronizzazione sensori in corso..." << std::endl;
    while (!first_state_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Sensori OK! Posizione iniziale rilevata correttamente." << std::endl;

    // 3. Configurazione Publisher
    ChannelPublisher<LowCmd> channel_publisher(b1::kTopicJointCtrl);
    channel_publisher.InitChannel();

    // Mappatura dei 29 giunti (Testa, Braccia, Vita, Gambe)
    std::array<b1::JointIndexWith7DofArm, 29> low_joints = {
        b1::JointIndexWith7DofArm::kHeadYaw, b1::JointIndexWith7DofArm::kHeadPitch,
        b1::JointIndexWith7DofArm::kLeftShoulderPitch, b1::JointIndexWith7DofArm::kLeftShoulderRoll,
        b1::JointIndexWith7DofArm::kLeftElbowPitch, b1::JointIndexWith7DofArm::kLeftElbowYaw,
        b1::JointIndexWith7DofArm::kLeftWristPitch, b1::JointIndexWith7DofArm::kLeftWristYaw,
        b1::JointIndexWith7DofArm::kLeftHandRoll,
        b1::JointIndexWith7DofArm::kRightShoulderPitch, b1::JointIndexWith7DofArm::kRightShoulderRoll,
        b1::JointIndexWith7DofArm::kRightElbowPitch, b1::JointIndexWith7DofArm::kRightElbowYaw,
        b1::JointIndexWith7DofArm::kRightWristPitch, b1::JointIndexWith7DofArm::kRightWristYaw,
        b1::JointIndexWith7DofArm::kRightHandRoll,
        b1::JointIndexWith7DofArm::kWaist,
        b1::JointIndexWith7DofArm::kLeftHipPitch, b1::JointIndexWith7DofArm::kLeftHipRoll, b1::JointIndexWith7DofArm::kLeftHipYaw,
        b1::JointIndexWith7DofArm::kLeftKneePitch, b1::JointIndexWith7DofArm::kCrankUpLeft, b1::JointIndexWith7DofArm::kCrankDownLeft,
        b1::JointIndexWith7DofArm::kRightHipPitch, b1::JointIndexWith7DofArm::kRightHipRoll, b1::JointIndexWith7DofArm::kRightHipYaw,
        b1::JointIndexWith7DofArm::kRightKneePitch, b1::JointIndexWith7DofArm::kCrankUpRight, b1::JointIndexWith7DofArm::kCrankDownRight
    };

    // Parametri di movimento
    float control_dt = 0.02f;
    float max_joint_velocity = 0.1f;
    float max_joint_delta = max_joint_velocity * control_dt;
    auto sleep_time = std::chrono::milliseconds(20);

    // Guadagni Kp/Kd
    std::array<float, 29> kps = { 5., 5., 70., 70., 70., 70., 50., 70., 50., 70., 70., 70., 70., 50., 70., 50., 100., 350., 350., 180., 350., 450., 450., 350., 350., 180., 350., 450., 450. };
    std::array<float, 29> kds = { .1, .1, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 5.0, 7.5, 7.5, 3., 5.5, 0.5, 0.5, 7.5, 7.5, 3., 5.5, 0.5, 0.5 };

    // Sincronizzazione: current_jpos_des parte dalla posizione reale sensorizzata [Elimina lo scatto]
    std::array<float, 29> current_jpos_des = current_real_pos;

    // --- DEFINIZIONE TARGET ---
    std::array<float, 29> target_home = { 0.00, 0.00, 0.25, -1.40, 0.00, -0.50, 0.0, 0.0, 0.0, 0.25, 1.40, 0.00, 0.50, 0.0, 0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.2, 0.137, 0.125, -0.1, 0.0, 0.0, 0.2, 0.137, 0.125 };
    std::array<float, 29> target_pos_1 = { 0.00, 0.00, -0.149, -1.5, 0.17, -1.5, -1.76, 0.0, 0.0, -0.149, 1.5, 0.1, 1.5, -1.76, 0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.2, 0.137, 0.125, -0.1, 0.0, 0.0, 0.2, 0.137, 0.125 };
    std::array<float, 29> target_pos_2 = { 0.00, 0.00, -1.4, -1.5, 0.17, -0.3, -1.76, 0.0, 0.0, -1.4, 1.5, 0.1, 0.3, -1.76, 0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.2, 0.137, 0.125, -0.1, 0.0, 0.0, 0.2, 0.137, 0.125 };

    std::cout << "Premi ENTER per iniziare la sequenza fluida..." << std::endl;
    std::cin.get();

    // Funzione helper per il movimento fluido
    auto move_to = [&](const std::array<float, 29>& target, const std::string& label) {
        std::cout << "Esecuzione: " << label << "..." << std::endl;
        bool reached = false;
        std::vector<MotorCmd> motor_cmds(b1::kJointCnt7DofArm);

        while (!reached) {
            reached = true; 
            LowCmd low_cmd;
            low_cmd.cmd_type(PARALLEL); //
            
            for (int j = 0; j < 29; j++) {
                float diff = target.at(j) - current_jpos_des.at(j);
                if (std::abs(diff) > 0.001f) {
                    reached = false;
                    current_jpos_des.at(j) += std::clamp(diff, -max_joint_delta, max_joint_delta);
                }
                
                int j_idx = int(low_joints.at(j));
                motor_cmds[j_idx].q(current_jpos_des.at(j));
                motor_cmds[j_idx].kp(kps.at(j));
                motor_cmds[j_idx].kd(kds.at(j));
                motor_cmds[j_idx].weight(1.0); // Peso massimo per il controllo
            }
            
            low_cmd.motor_cmd(motor_cmds); //
            channel_publisher.Write(&low_cmd);
            std::this_thread::sleep_for(sleep_time);
        }
        std::cout << label << " completato." << std::endl;
    };

    // Sequenza di movimenti
    move_to(target_home, "Fase 0 (Posizionamento Home)");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    move_to(target_pos_1, "Fase 1 (Target 1)");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    move_to(target_pos_2, "Fase 2 (Target 2)");

    std::cout << "Sequenza terminata correttamente." << std::endl;
    return 0;
}
