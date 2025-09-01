#include "mks_can_interface.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <tuple>

namespace mks {

MksCanInterface::MksCanInterface() 
    : can_socket_(-1), initialized_(false), running_(false) {
    // Initialize motor states using piecewise_construct
    for (uint8_t i = 1; i <= 6; ++i) {
        JointId joint_id = static_cast<JointId>(i);
        motor_states_.emplace(std::piecewise_construct,
                             std::forward_as_tuple(joint_id),
                             std::forward_as_tuple());
    }
}

MksCanInterface::~MksCanInterface() {
    deinit();
}

bool MksCanInterface::init(const std::string& can_interface, int /* bitrate */) {
    if (initialized_) {
        std::cerr << "MksCanInterface already initialized" << std::endl;
        return false;
    }
    
    can_interface_ = can_interface;
    
    // Create CAN socket
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        std::cerr << "Failed to create CAN socket" << std::endl;
        return false;
    }
    
    // Set up CAN interface
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Failed to get interface index for " << can_interface << std::endl;
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }
    
    // Bind socket to CAN interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind CAN socket to " << can_interface << std::endl;
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }
    
    // Start receiver thread
    running_ = true;
    receiver_thread_ = std::thread(&MksCanInterface::receiverLoop, this);
    
    initialized_ = true;
    std::cout << "MksCanInterface initialized on " << can_interface << std::endl;
    return true;
}

void MksCanInterface::deinit() {
    if (!initialized_) {
        return;
    }
    
    // Stop receiver thread
    running_ = false;
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
    
    // Close socket
    if (can_socket_ >= 0) {
        close(can_socket_);
        can_socket_ = -1;
    }
    
    initialized_ = false;
    std::cout << "MksCanInterface deinitialized" << std::endl;
}

bool MksCanInterface::setPosition(JointId joint_id, int32_t position, uint16_t speed) {
    if (!initialized_) {
        std::cerr << "MksCanInterface not initialized" << std::endl;
        return false;
    }
    
    MksMessage msg = MksProtocol::createPositionCommand(position, speed);
    return sendCanFrame(joint_id, msg);
}

bool MksCanInterface::enableMotor(JointId joint_id) {
    if (!initialized_) {
        std::cerr << "MksCanInterface not initialized" << std::endl;
        return false;
    }
    
    MksMessage msg = MksProtocol::createEnableDisableCommand(true);
    return sendCanFrame(joint_id, msg);
}

bool MksCanInterface::disableMotor(JointId joint_id) {
    if (!initialized_) {
        std::cerr << "MksCanInterface not initialized" << std::endl;
        return false;
    }
    
    MksMessage msg = MksProtocol::createEnableDisableCommand(false);
    return sendCanFrame(joint_id, msg);
}

bool MksCanInterface::readPosition(JointId joint_id) {
    if (!initialized_) {
        std::cerr << "MksCanInterface not initialized" << std::endl;
        return false;
    }
    
    MksMessage msg = MksProtocol::createPositionReadCommand();
    return sendCanFrame(joint_id, msg);
}

bool MksCanInterface::readVelocity(JointId joint_id) {
    if (!initialized_) {
        std::cerr << "MksCanInterface not initialized" << std::endl;
        return false;
    }
    
    MksMessage msg = MksProtocol::createVelocityReadCommand();
    return sendCanFrame(joint_id, msg);
}

bool MksCanInterface::calibrateMotor(JointId joint_id, int32_t old_position) {
    if (!initialized_) {
        std::cerr << "MksCanInterface not initialized" << std::endl;
        return false;
    }
    
    MksMessage msg = MksProtocol::createCalibrationCommand(old_position);
    return sendCanFrame(joint_id, msg);
}

bool MksCanInterface::emergencyStop(JointId joint_id) {
    if (!initialized_) {
        std::cerr << "MksCanInterface not initialized" << std::endl;
        return false;
    }
    
    MksMessage msg = MksProtocol::createEmergencyStopCommand();
    return sendCanFrame(joint_id, msg);
}

bool MksCanInterface::emergencyStopAll() {
    bool success = true;
    for (uint8_t i = 1; i <= 6; ++i) {
        JointId joint_id = static_cast<JointId>(i);
        success &= emergencyStop(joint_id);
    }
    return success;
}

bool MksCanInterface::sendCanFrame(JointId joint_id, const MksMessage& message) {
    if (can_socket_ < 0) {
        return false;
    }
    
    struct can_frame frame;
    frame.can_id = jointIdToCanId(joint_id);
    frame.can_dlc = 8;
    
    std::vector<uint8_t> msg_bytes = MksProtocol::messageToBytes(message);
    std::copy(msg_bytes.begin(), msg_bytes.end(), frame.data);
    
    ssize_t bytes_sent = write(can_socket_, &frame, sizeof(frame));
    if (bytes_sent != sizeof(frame)) {
        std::cerr << "Failed to send CAN frame" << std::endl;
        return false;
    }
    
    return true;
}

void MksCanInterface::receiverLoop() {
    while (running_) {
        struct can_frame frame;
        ssize_t bytes_received = read(can_socket_, &frame, sizeof(frame));
        
        if (bytes_received == sizeof(frame)) {
            processCanFrame(frame);
        } else if (bytes_received < 0 && running_) {
            std::cerr << "Error reading CAN frame" << std::endl;
            break;
        }
    }
}

void MksCanInterface::processCanFrame(const can_frame& frame) {
    JointId joint_id = canIdToJointId(frame.can_id);
    
    if (frame.can_dlc < 8) {
        return; // Invalid frame size
    }
    
    std::vector<uint8_t> data(frame.data, frame.data + frame.can_dlc);
    MksCommand command = static_cast<MksCommand>(data[0]);
    
    // Update motor state and call callbacks
    switch (command) {
        case MksCommand::POSITION_READ: {
            PositionData pos_data = MksProtocol::parsePositionResponse(data);
            if (pos_data.valid) {
                // Update motor state
                {
                    std::lock_guard<std::mutex> lock(motor_states_[joint_id].mutex);
                    motor_states_[joint_id].last_position = pos_data.position;
                    motor_states_[joint_id].position_valid = true;
                }
                
                // Call callback
                if (position_callback_) {
                    position_callback_(joint_id, pos_data);
                }
            }
            break;
        }
        
        case MksCommand::ENABLE_DISABLE: {
            bool enabled;
            if (MksProtocol::parseEnableDisableResponse(data, enabled)) {
                bool error = (data.size() > 1 && data[1] == 0);
                
                // Update motor state
                {
                    std::lock_guard<std::mutex> lock(motor_states_[joint_id].mutex);
                    motor_states_[joint_id].last_enable_status = enabled;
                    motor_states_[joint_id].enable_status_valid = true;
                }
                
                // Call callback
                if (enable_status_callback_) {
                    enable_status_callback_(joint_id, enabled, error);
                }
            }
            break;
        }
        
        case MksCommand::POSITION_COMMAND:
        case MksCommand::CALIBRATION:
        case MksCommand::EMERGENCY_STOP: {
            bool success = (data.size() > 1 && data[1] != 0);
            
            // Call generic callback
            if (generic_response_callback_) {
                generic_response_callback_(joint_id, command, success);
            }
            
            // Complete any pending sync operations
            std::string op_key = makeSyncOperationKey(joint_id, command);
            completeSyncOperation(op_key, success);
            break;
        }
        
        default:
            // Unknown command
            break;
    }
}

bool MksCanInterface::getLastPosition(JointId joint_id, int32_t& position) {
    std::lock_guard<std::mutex> lock(motor_states_[joint_id].mutex);
    if (motor_states_[joint_id].position_valid) {
        position = motor_states_[joint_id].last_position;
        return true;
    }
    return false;
}

bool MksCanInterface::getLastEnableStatus(JointId joint_id, bool& enabled) {
    std::lock_guard<std::mutex> lock(motor_states_[joint_id].mutex);
    if (motor_states_[joint_id].enable_status_valid) {
        enabled = motor_states_[joint_id].last_enable_status;
        return true;
    }
    return false;
}

// Synchronous operations implementation
bool MksCanInterface::setPositionSync(JointId joint_id, int32_t position, uint16_t speed, int timeout_ms) {
    std::string op_key = makeSyncOperationKey(joint_id, MksCommand::POSITION_COMMAND);
    auto sync_op = std::make_shared<SyncOperation>();
    sync_op->joint_id = joint_id;
    sync_op->expected_command = MksCommand::POSITION_COMMAND;
    
    {
        std::lock_guard<std::mutex> lock(pending_operations_mutex_);
        pending_operations_[op_key] = sync_op;
    }
    
    if (!setPosition(joint_id, position, speed)) {
        std::lock_guard<std::mutex> lock(pending_operations_mutex_);
        pending_operations_.erase(op_key);
        return false;
    }
    
    std::unique_lock<std::mutex> lock(sync_op->mutex);
    bool completed = sync_op->cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                                          [&sync_op] { return sync_op->completed; });
    
    {
        std::lock_guard<std::mutex> op_lock(pending_operations_mutex_);
        pending_operations_.erase(op_key);
    }
    
    return completed && sync_op->success;
}

bool MksCanInterface::enableMotorSync(JointId joint_id, int timeout_ms) {
    std::string op_key = makeSyncOperationKey(joint_id, MksCommand::ENABLE_DISABLE);
    auto sync_op = std::make_shared<SyncOperation>();
    sync_op->joint_id = joint_id;
    sync_op->expected_command = MksCommand::ENABLE_DISABLE;
    
    {
        std::lock_guard<std::mutex> lock(pending_operations_mutex_);
        pending_operations_[op_key] = sync_op;
    }
    
    if (!enableMotor(joint_id)) {
        std::lock_guard<std::mutex> lock(pending_operations_mutex_);
        pending_operations_.erase(op_key);
        return false;
    }
    
    std::unique_lock<std::mutex> lock(sync_op->mutex);
    bool completed = sync_op->cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                                          [&sync_op] { return sync_op->completed; });
    
    {
        std::lock_guard<std::mutex> op_lock(pending_operations_mutex_);
        pending_operations_.erase(op_key);
    }
    
    return completed && sync_op->success;
}

bool MksCanInterface::disableMotorSync(JointId joint_id, int timeout_ms) {
    std::string op_key = makeSyncOperationKey(joint_id, MksCommand::ENABLE_DISABLE);
    auto sync_op = std::make_shared<SyncOperation>();
    sync_op->joint_id = joint_id;
    sync_op->expected_command = MksCommand::ENABLE_DISABLE;
    
    {
        std::lock_guard<std::mutex> lock(pending_operations_mutex_);
        pending_operations_[op_key] = sync_op;
    }
    
    if (!disableMotor(joint_id)) {
        std::lock_guard<std::mutex> lock(pending_operations_mutex_);
        pending_operations_.erase(op_key);
        return false;
    }
    
    std::unique_lock<std::mutex> lock(sync_op->mutex);
    bool completed = sync_op->cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                                          [&sync_op] { return sync_op->completed; });
    
    {
        std::lock_guard<std::mutex> op_lock(pending_operations_mutex_);
        pending_operations_.erase(op_key);
    }
    
    return completed && sync_op->success;
}

bool MksCanInterface::readPositionSync(JointId joint_id, int32_t& position, int timeout_ms) {
    if (!readPosition(joint_id)) {
        return false;
    }
    
    // Wait for position update
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(timeout_ms)) {
        if (getLastPosition(joint_id, position)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return false;
}

void MksCanInterface::completeSyncOperation(const std::string& operation_key, bool success) {
    std::lock_guard<std::mutex> lock(pending_operations_mutex_);
    auto it = pending_operations_.find(operation_key);
    if (it != pending_operations_.end()) {
        std::lock_guard<std::mutex> op_lock(it->second->mutex);
        it->second->completed = true;
        it->second->success = success;
        it->second->cv.notify_one();
    }
}

std::string MksCanInterface::makeSyncOperationKey(JointId joint_id, MksCommand command) {
    return std::to_string(static_cast<int>(joint_id)) + "_" + std::to_string(static_cast<int>(command));
}

} // namespace mks
