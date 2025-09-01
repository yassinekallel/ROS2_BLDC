#include "../include/mks_protocol.hpp"
#include <iostream>
#include <vector>
#include <cstdio>

// Simple test program to test MKS protocol commands
int main() {
    std::cout << "=== Simple Motor Test Program ===" << std::endl;
    std::cout << "Testing MKS protocol command generation..." << std::endl;
    
    // Test position command
    int32_t position = 1000; // 1000 steps
    uint16_t speed = 1024;   // Default speed
    
    // Create position command
    auto pos_msg = mks::MksProtocol::createPositionCommand(position, speed);
    auto pos_cmd = mks::MksProtocol::messageToBytes(pos_msg);
    
    std::cout << "Position command to position " << position << ":" << std::endl;
    std::cout << "CAN Frame: ";
    for (size_t i = 0; i < pos_cmd.size(); ++i) {
        printf("%02X ", pos_cmd[i]);
    }
    std::cout << std::endl;
    
    // Create enable command
    auto enable_msg = mks::MksProtocol::createEnableDisableCommand(true);
    auto enable_cmd = mks::MksProtocol::messageToBytes(enable_msg);
    std::cout << "Enable command:" << std::endl;
    std::cout << "CAN Frame: ";
    for (size_t i = 0; i < enable_cmd.size(); ++i) {
        printf("%02X ", enable_cmd[i]);
    }
    std::cout << std::endl;
    
    // Create position read command
    auto read_msg = mks::MksProtocol::createPositionReadCommand();
    auto read_cmd = mks::MksProtocol::messageToBytes(read_msg);
    std::cout << "Position read command:" << std::endl;
    std::cout << "CAN Frame: ";
    for (size_t i = 0; i < read_cmd.size(); ++i) {
        printf("%02X ", read_cmd[i]);
    }
    std::cout << std::endl;
    
    // Create disable command
    auto disable_msg = mks::MksProtocol::createEnableDisableCommand(false);
    auto disable_cmd = mks::MksProtocol::messageToBytes(disable_msg);
    std::cout << "Disable command:" << std::endl;
    std::cout << "CAN Frame: ";
    for (size_t i = 0; i < disable_cmd.size(); ++i) {
        printf("%02X ", disable_cmd[i]);
    }
    std::cout << std::endl;
    
    std::cout << "\n=== Expected Output ===" << std::endl;
    std::cout << "Position command should start with: F5" << std::endl;
    std::cout << "Enable command should start with: F3" << std::endl;
    std::cout << "Position read should start with: 31" << std::endl;
    std::cout << "Disable command should start with: F3" << std::endl;
    std::cout << "\nIf these match your Arduino output, the protocol is working!" << std::endl;
    std::cout << "\nNOTE: This test doesn't send to CAN - it just generates the protocol bytes." << std::endl;
    std::cout << "To test with actual motors, use: ros2 run refboard_base test_mks_can can0" << std::endl;
    
    return 0;
}
