#include "../include/mks_protocol.hpp"
#include <iostream>
#include <vector>
#include <cstdio>

// Simple test program to test MKS protocol commands
int main() {
    std::cout << "=== Simple Motor Test Program ===" << std::endl;
    std::cout << "Testing MKS protocol command generation..." << std::endl;
    
    // Test motor ID 1
    uint8_t motor_id = 1;
    int32_t position = 1000; // 1000 steps
    
    // Create position command
    auto pos_cmd = mks::MksProtocol::createPositionCommand(motor_id, position);
    
    std::cout << "Position command for motor " << (int)motor_id 
              << " to position " << position << ":" << std::endl;
    std::cout << "CAN Frame: ";
    for (size_t i = 0; i < pos_cmd.size(); ++i) {
        printf("%02X ", pos_cmd[i]);
    }
    std::cout << std::endl;
    
    // Create enable command
    auto enable_cmd = mks::MksProtocol::createEnableCommand(motor_id, true);
    std::cout << "Enable command for motor " << (int)motor_id << ":" << std::endl;
    std::cout << "CAN Frame: ";
    for (size_t i = 0; i < enable_cmd.size(); ++i) {
        printf("%02X ", enable_cmd[i]);
    }
    std::cout << std::endl;
    
    // Create position read command
    auto read_cmd = mks::MksProtocol::createPositionReadCommand(motor_id);
    std::cout << "Position read command for motor " << (int)motor_id << ":" << std::endl;
    std::cout << "CAN Frame: ";
    for (size_t i = 0; i < read_cmd.size(); ++i) {
        printf("%02X ", read_cmd[i]);
    }
    std::cout << std::endl;
    
    std::cout << "\n=== Expected Output ===" << std::endl;
    std::cout << "Position command should start with: F5 01" << std::endl;
    std::cout << "Enable command should start with: F3 01" << std::endl;
    std::cout << "Read command should start with: 31 01" << std::endl;
    std::cout << "\nIf these match your Arduino output, the protocol is working!" << std::endl;
    
    return 0;
}
