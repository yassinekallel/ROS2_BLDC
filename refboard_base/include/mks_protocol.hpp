#ifndef MKS_PROTOCOL_HPP
#define MKS_PROTOCOL_HPP

#include <cstdint>
#include <vector>

namespace mks {

// MKS Protocol Command IDs (based on your Arduino const.h)
enum class MksCommand : uint8_t {
    POSITION_COMMAND = 0xF5,    // Move to absolute position
    POSITION_READ = 0x31,       // Read current position
    VELOCITY_READ = 0x32,       // Read current velocity
    ENABLE_DISABLE = 0xF3,      // Enable/disable motor
    EMERGENCY_STOP = 0xF7,      // Emergency stop command
    CALIBRATION = 0xF8          // Calibration command
};

// Joint CAN IDs (based on your Arduino const.h)
enum class JointId : uint8_t {
    X_MOTOR = 0x01,             // CAN ID for joint 1 (X)
    Y_MOTOR = 0x02,             // CAN ID for joint 2 (Y)
    Z_MOTOR = 0x03,             // CAN ID for joint 3 (Z)
    A_MOTOR = 0x04,             // CAN ID for joint 4 (A)
    B_MOTOR = 0x05,             // CAN ID for joint 5 (B)
    C_MOTOR = 0x06              // CAN ID for joint 6 (C)
};

// MKS Protocol message structure
struct MksMessage {
    uint8_t command;            // MksCommand
    uint8_t data[6];            // 6 bytes of data
    uint8_t crc;                // CRC checksum
    
    MksMessage() : command(0), crc(0) {
        std::fill(data, data + 6, 0);
    }
};

// Position data structure (decoded from CAN frame)
struct PositionData {
    uint8_t command_id;
    uint16_t speed;
    uint8_t torque;
    int32_t position;           // 24-bit signed position value
    bool valid;
    
    PositionData() : command_id(0), speed(0), torque(0), position(0), valid(false) {}
};

class MksProtocol {
public:
    // Create command messages
    static MksMessage createPositionCommand(int32_t position, uint16_t speed = 1024);
    static MksMessage createEnableDisableCommand(bool enable);
    static MksMessage createPositionReadCommand();
    static MksMessage createVelocityReadCommand();
    static MksMessage createCalibrationCommand(int32_t old_position);
    static MksMessage createEmergencyStopCommand();
    
    // Parse received data
    static PositionData parsePositionResponse(const std::vector<uint8_t>& data);
    static bool parseEnableDisableResponse(const std::vector<uint8_t>& data, bool& enabled);
    
    // Utility functions
    static uint8_t calculateCrc(const uint8_t* data, size_t length);
    static std::vector<uint8_t> messageToBytes(const MksMessage& msg);
    static MksMessage bytesToMessage(const std::vector<uint8_t>& bytes);
    
    // Convert between different units
    static double positionToAngle(int32_t position);      // position to degrees
    static int32_t angleToPosition(double angle);         // degrees to position
    static double positionToRevolutions(int32_t position); // position to revolutions
    
private:
    static constexpr uint16_t DEFAULT_SPEED = 1024;
    static constexpr int32_t POSITION_PER_REVOLUTION = 36000;  // Based on your Arduino code
};

} // namespace mks

#endif // MKS_PROTOCOL_HPP
