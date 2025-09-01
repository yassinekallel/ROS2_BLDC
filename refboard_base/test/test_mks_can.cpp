#include "mks_can_interface.hpp"
#include "mks_protocol.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <can_interface>" << std::endl;
        std::cout << "Example: " << argv[0] << " can0" << std::endl;
        return 1;
    }
    
    std::string can_interface = argv[1];
    mks::MksCanInterface mks_can;
    
    // Set up callbacks
    mks_can.setPositionCallback([](mks::JointId joint_id, const mks::PositionData& data) {
        std::cout << "Position received for joint " << static_cast<int>(joint_id) 
                  << ": position=" << data.position 
                  << ", speed=" << data.speed 
                  << ", angle=" << mks::MksProtocol::positionToAngle(data.position) << " degrees"
                  << std::endl;
    });
    
    mks_can.setEnableStatusCallback([](mks::JointId joint_id, bool enabled, bool error) {
        std::cout << "Enable status for joint " << static_cast<int>(joint_id) 
                  << ": enabled=" << (enabled ? "true" : "false")
                  << ", error=" << (error ? "true" : "false") << std::endl;
    });
    
    mks_can.setGenericResponseCallback([](mks::JointId joint_id, mks::MksCommand command, bool success) {
        std::cout << "Response for joint " << static_cast<int>(joint_id) 
                  << ", command=" << static_cast<int>(command)
                  << ", success=" << (success ? "true" : "false") << std::endl;
    });
    
    // Initialize CAN interface
    std::cout << "Initializing MKS CAN interface on " << can_interface << "..." << std::endl;
    if (!mks_can.init(can_interface)) {
        std::cerr << "Failed to initialize CAN interface" << std::endl;
        return 1;
    }
    
    std::cout << "CAN interface initialized successfully!" << std::endl;
    std::cout << "Available commands:" << std::endl;
    std::cout << "  e <joint_id>     - Enable motor (1-6)" << std::endl;
    std::cout << "  d <joint_id>     - Disable motor (1-6)" << std::endl;
    std::cout << "  p <joint_id>     - Read position (1-6)" << std::endl;
    std::cout << "  s <joint_id> <pos> - Set position (1-6) <position_steps>" << std::endl;
    std::cout << "  c <joint_id>     - Calibrate motor (1-6)" << std::endl;
    std::cout << "  h                - Show this help" << std::endl;
    std::cout << "  q                - Quit" << std::endl;
    std::cout << std::endl;
    
    // Test commands
    std::cout << "Testing basic commands..." << std::endl;
    
    // Test enable joint 1
    mks::JointId test_joint = mks::JointId::X_MOTOR;
    std::cout << "Enabling joint 1..." << std::endl;
    mks_can.enableMotor(test_joint);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Test position read
    std::cout << "Reading position of joint 1..." << std::endl;
    mks_can.readPosition(test_joint);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Interactive loop
    char command;
    int joint_id, position;
    
    while (true) {
        std::cout << "Enter command: ";
        std::cin >> command;
        
        switch (command) {
            case 'e':
                std::cin >> joint_id;
                if (joint_id >= 1 && joint_id <= 6) {
                    mks_can.enableMotor(static_cast<mks::JointId>(joint_id));
                    std::cout << "Enabling motor " << joint_id << std::endl;
                } else {
                    std::cout << "Invalid joint ID. Use 1-6." << std::endl;
                }
                break;
                
            case 'd':
                std::cin >> joint_id;
                if (joint_id >= 1 && joint_id <= 6) {
                    mks_can.disableMotor(static_cast<mks::JointId>(joint_id));
                    std::cout << "Disabling motor " << joint_id << std::endl;
                } else {
                    std::cout << "Invalid joint ID. Use 1-6." << std::endl;
                }
                break;
                
            case 'p':
                std::cin >> joint_id;
                if (joint_id >= 1 && joint_id <= 6) {
                    mks_can.readPosition(static_cast<mks::JointId>(joint_id));
                    std::cout << "Reading position of motor " << joint_id << std::endl;
                } else {
                    std::cout << "Invalid joint ID. Use 1-6." << std::endl;
                }
                break;
                
            case 's':
                std::cin >> joint_id >> position;
                if (joint_id >= 1 && joint_id <= 6) {
                    mks_can.setPosition(static_cast<mks::JointId>(joint_id), position);
                    std::cout << "Setting position of motor " << joint_id << " to " << position << std::endl;
                } else {
                    std::cout << "Invalid joint ID. Use 1-6." << std::endl;
                }
                break;
                
            case 'c':
                std::cin >> joint_id;
                if (joint_id >= 1 && joint_id <= 6) {
                    mks_can.calibrateMotor(static_cast<mks::JointId>(joint_id));
                    std::cout << "Calibrating motor " << joint_id << std::endl;
                } else {
                    std::cout << "Invalid joint ID. Use 1-6." << std::endl;
                }
                break;
                
            case 'h':
                std::cout << "Available commands:" << std::endl;
                std::cout << "  e <joint_id>     - Enable motor (1-6)" << std::endl;
                std::cout << "  d <joint_id>     - Disable motor (1-6)" << std::endl;
                std::cout << "  p <joint_id>     - Read position (1-6)" << std::endl;
                std::cout << "  s <joint_id> <pos> - Set position (1-6) <position_steps>" << std::endl;
                std::cout << "  c <joint_id>     - Calibrate motor (1-6)" << std::endl;
                std::cout << "  h                - Show this help" << std::endl;
                std::cout << "  q                - Quit" << std::endl;
                break;
                
            case 'q':
                std::cout << "Exiting..." << std::endl;
                return 0;
                
            default:
                std::cout << "Unknown command. Type 'h' for help." << std::endl;
                break;
        }
        
        // Small delay to see responses
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
}
