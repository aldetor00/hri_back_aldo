#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/idl/b1/LowState.h>

#include <thread>
#include <chrono>
#include <iostream>

#define TOPIC "T1_release/low_state"

using namespace booster::robot;
using namespace booster::common;
using namespace booster_interface::msg;

void Handler(const void *msg) {
    const LowState *low_state_msg = static_cast<const LowState *>(msg);
    std::cout << "Received message: " << std::endl
              << "  serial motor count: " << low_state_msg->motor_state_serial().size() << std::endl
              << "  parallel motor count: " << low_state_msg->motor_state_parallel().size() << std::endl;
    
    // Esempio lettura motori paralleli (braccia/gambe)
    for(int i = 0; i < low_state_msg->motor_state_parallel().size(); i++) {
        std::cout << "  parallel motor " << i << " pos: "
                  << low_state_msg->motor_state_parallel()[i].q() << std::endl;
    }
    std::cout << "-----------------------------------------------" << std::endl;
}

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Errore: Devi specificare l'interfaccia di rete!" << std::endl;
        std::cout << "Usage: " << argv[0] << " lo" << std::endl;
        return -1;
    }

    // Inizializzazione con l'interfaccia passata (es. "lo")
    ChannelFactory::Instance()->Init(0, argv[1]);

    // Creazione del subscriber utilizzando la firma corretta const void*
    ChannelSubscriber<LowState> channel_subscriber(TOPIC, Handler);
    
    // MODIFICA: InitChannel non restituisce un valore, quindi lo chiamiamo direttamente
    channel_subscriber.InitChannel();

    std::cout << "Subscriber avviato sull'interfaccia: " << argv[1] << std::endl;
    std::cout << "In attesa di messaggi..." << std::endl;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
