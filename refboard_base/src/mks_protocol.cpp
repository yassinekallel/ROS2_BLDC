#include "mks_protocol.hpp"
#include <algorithm>
#include <cmath>

namespace mks {

MksMessage MksProtocol::createPositionCommand(int32_t position, uint16_t speed) {
    MksMessage msg;
    msg.command = static_cast<uint8_t>(MksCommand::POSITION_COMMAND);
    
    // Pack speed (bytes 1-2, big endian)
    msg.data[0] = static_cast<uint8_t>(speed >> 8);
    msg.data[1] = static_cast<uint8_t>(speed & 0xFF);
    
    // Byte 2 is reserved (torque), set to 0
    msg.data[2] = 0;
    
    // Pack position (bytes 3-5, big endian, 24-bit)
    msg.data[3] = static_cast<uint8_t>((position >> 16) & 0xFF);
    msg.data[4] = static_cast<uint8_t>((position >> 8) & 0xFF);
    msg.data[5] = static_cast<uint8_t>(position & 0xFF);
    
    // Calculate CRC
    std::vector<uint8_t> crc_data = {msg.command, msg.data[0], msg.data[1], msg.data[2], 
                                     msg.data[3], msg.data[4], msg.data[5]};
    msg.crc = calculateCrc(crc_data.data(), crc_data.size());
    
    return msg;
}

MksMessage MksProtocol::createEnableDisableCommand(bool enable) {
    MksMessage msg;
    msg.command = static_cast<uint8_t>(MksCommand::ENABLE_DISABLE);
    msg.data[0] = enable ? 1 : 0;
    std::fill(msg.data + 1, msg.data + 6, 0);
    
    // Calculate CRC
    std::vector<uint8_t> crc_data = {msg.command, msg.data[0], msg.data[1], msg.data[2], 
                                     msg.data[3], msg.data[4], msg.data[5]};
    msg.crc = calculateCrc(crc_data.data(), crc_data.size());
    
    return msg;
}

MksMessage MksProtocol::createPositionReadCommand() {
    MksMessage msg;
    msg.command = static_cast<uint8_t>(MksCommand::POSITION_READ);
    std::fill(msg.data, msg.data + 6, 0);
    
    // Calculate CRC
    std::vector<uint8_t> crc_data = {msg.command, msg.data[0], msg.data[1], msg.data[2], 
                                     msg.data[3], msg.data[4], msg.data[5]};
    msg.crc = calculateCrc(crc_data.data(), crc_data.size());
    
    return msg;
}

MksMessage MksProtocol::createVelocityReadCommand() {
    MksMessage msg;
    msg.command = static_cast<uint8_t>(MksCommand::VELOCITY_READ);
    std::fill(msg.data, msg.data + 6, 0);
    
    // Calculate CRC
    std::vector<uint8_t> crc_data = {msg.command, msg.data[0], msg.data[1], msg.data[2], 
                                     msg.data[3], msg.data[4], msg.data[5]};
    msg.crc = calculateCrc(crc_data.data(), crc_data.size());
    
    return msg;
}

MksMessage MksProtocol::createCalibrationCommand(int32_t old_position) {
    MksMessage msg;
    msg.command = static_cast<uint8_t>(MksCommand::CALIBRATION);
    
    // Set first 3 bytes to 0
    msg.data[0] = 0;
    msg.data[1] = 0;
    msg.data[2] = 0;
    
    // Pack old position (bytes 3-5, big endian, 24-bit)
    msg.data[3] = static_cast<uint8_t>((old_position >> 16) & 0xFF);
    msg.data[4] = static_cast<uint8_t>((old_position >> 8) & 0xFF);
    msg.data[5] = static_cast<uint8_t>(old_position & 0xFF);
    
    // Calculate CRC
    std::vector<uint8_t> crc_data = {msg.command, msg.data[0], msg.data[1], msg.data[2], 
                                     msg.data[3], msg.data[4], msg.data[5]};
    msg.crc = calculateCrc(crc_data.data(), crc_data.size());
    
    return msg;
}

MksMessage MksProtocol::createEmergencyStopCommand() {
    MksMessage msg;
    msg.command = static_cast<uint8_t>(MksCommand::EMERGENCY_STOP);
    std::fill(msg.data, msg.data + 6, 0);
    
    // Calculate CRC
    std::vector<uint8_t> crc_data = {msg.command, msg.data[0], msg.data[1], msg.data[2], 
                                     msg.data[3], msg.data[4], msg.data[5]};
    msg.crc = calculateCrc(crc_data.data(), crc_data.size());
    
    return msg;
}

PositionData MksProtocol::parsePositionResponse(const std::vector<uint8_t>& data) {
    PositionData pos_data;
    
    if (data.size() < 8) {
        pos_data.valid = false;
        return pos_data;
    }
    
    pos_data.command_id = data[0];
    
    // Parse speed (bytes 1-2, big endian)
    pos_data.speed = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    
    // Parse torque (byte 3)
    pos_data.torque = data[3];
    
    // Parse position (bytes 4-6, big endian, 24-bit signed)
    int32_t raw_position = (static_cast<int32_t>(data[4] & 0xFF) << 16) |
                          (static_cast<int32_t>(data[5] & 0xFF) << 8) |
                          (static_cast<int32_t>(data[6] & 0xFF));
    
    // Sign extend if negative (check bit 23)
    if (raw_position & 0x00800000) {
        raw_position |= 0xFF000000;  // Sign extend
    } else {
        raw_position &= 0x00FFFFFF;  // Clear sign bit
    }
    
    pos_data.position = raw_position;
    pos_data.valid = true;
    
    return pos_data;
}

bool MksProtocol::parseEnableDisableResponse(const std::vector<uint8_t>& data, bool& enabled) {
    if (data.size() < 2) {
        return false;
    }
    
    if (data[0] != static_cast<uint8_t>(MksCommand::ENABLE_DISABLE)) {
        return false;
    }
    
    enabled = (data[1] != 0);
    return true;
}

uint8_t MksProtocol::calculateCrc(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc += data[i];
    }
    return crc;
}

std::vector<uint8_t> MksProtocol::messageToBytes(const MksMessage& msg) {
    std::vector<uint8_t> bytes(8);
    bytes[0] = msg.command;
    std::copy(msg.data, msg.data + 6, bytes.begin() + 1);
    bytes[7] = msg.crc;
    return bytes;
}

MksMessage MksProtocol::bytesToMessage(const std::vector<uint8_t>& bytes) {
    MksMessage msg;
    if (bytes.size() >= 8) {
        msg.command = bytes[0];
        std::copy(bytes.begin() + 1, bytes.begin() + 7, msg.data);
        msg.crc = bytes[7];
    }
    return msg;
}

double MksProtocol::positionToAngle(int32_t position) {
    return static_cast<double>(position % POSITION_PER_REVOLUTION) / 100.0;
}

int32_t MksProtocol::angleToPosition(double angle) {
    return static_cast<int32_t>(angle * 100.0);
}

double MksProtocol::positionToRevolutions(int32_t position) {
    return static_cast<double>(position) / static_cast<double>(POSITION_PER_REVOLUTION);
}

} // namespace mks
