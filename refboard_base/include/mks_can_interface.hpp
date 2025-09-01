#ifndef MKS_CAN_INTERFACE_HPP
#define MKS_CAN_INTERFACE_HPP

#include "mks_protocol.hpp"
#include <linux/can.h>
#include <string>
#include <functional>
#include <map>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace mks {

// Callback types for received data
using PositionCallback = std::function<void(JointId joint_id, const PositionData& data)>;
using EnableStatusCallback = std::function<void(JointId joint_id, bool enabled, bool error)>;
using GenericResponseCallback = std::function<void(JointId joint_id, MksCommand command, bool success)>;

class MksCanInterface {
public:
    MksCanInterface();
    ~MksCanInterface();
    
    // Initialize and deinitialize CAN interface
    bool init(const std::string& can_interface, int bitrate = 500000);
    void deinit();
    bool isInitialized() const { return initialized_; }
    
    // Motor control commands
    bool setPosition(JointId joint_id, int32_t position, uint16_t speed = 1024);
    bool enableMotor(JointId joint_id);
    bool disableMotor(JointId joint_id);
    bool readPosition(JointId joint_id);
    bool readVelocity(JointId joint_id);
    bool calibrateMotor(JointId joint_id, int32_t old_position = 0);
    bool emergencyStop(JointId joint_id);
    bool emergencyStopAll();
    
    // Set callbacks for received data
    void setPositionCallback(PositionCallback callback) { position_callback_ = callback; }
    void setEnableStatusCallback(EnableStatusCallback callback) { enable_status_callback_ = callback; }
    void setGenericResponseCallback(GenericResponseCallback callback) { generic_response_callback_ = callback; }
    
    // Get latest motor state (non-blocking)
    bool getLastPosition(JointId joint_id, int32_t& position);
    bool getLastEnableStatus(JointId joint_id, bool& enabled);
    
    // Synchronous operations (blocking with timeout)
    bool setPositionSync(JointId joint_id, int32_t position, uint16_t speed = 1024, int timeout_ms = 1000);
    bool enableMotorSync(JointId joint_id, int timeout_ms = 1000);
    bool disableMotorSync(JointId joint_id, int timeout_ms = 1000);
    bool readPositionSync(JointId joint_id, int32_t& position, int timeout_ms = 1000);
    
private:
    // CAN socket management
    int can_socket_;
    std::string can_interface_;
    bool initialized_;
    std::atomic<bool> running_;
    
    // Receiver thread
    std::thread receiver_thread_;
    void receiverLoop();
    
    // Send CAN frame
    bool sendCanFrame(JointId joint_id, const MksMessage& message);
    
    // Process received CAN frames
    void processCanFrame(const can_frame& frame);
    
    // Callbacks
    PositionCallback position_callback_;
    EnableStatusCallback enable_status_callback_;
    GenericResponseCallback generic_response_callback_;
    
    // Motor state tracking
    struct MotorState {
        int32_t last_position;
        bool last_enable_status;
        bool position_valid;
        bool enable_status_valid;
        std::mutex mutex;
        
        // Make the struct non-copyable but movable
        MotorState() : last_position(0), last_enable_status(false), 
                      position_valid(false), enable_status_valid(false) {}
        MotorState(const MotorState&) = delete;
        MotorState& operator=(const MotorState&) = delete;
        MotorState(MotorState&&) = default;
        MotorState& operator=(MotorState&&) = default;
    };
    std::map<JointId, MotorState> motor_states_;
    
    // Synchronous operation support
    struct SyncOperation {
        JointId joint_id;
        MksCommand expected_command;
        bool completed;
        bool success;
        std::mutex mutex;
        std::condition_variable cv;
        
        SyncOperation() : completed(false), success(false) {}
    };
    std::map<std::string, std::shared_ptr<SyncOperation>> pending_operations_;
    std::mutex pending_operations_mutex_;
    
    void completeSyncOperation(const std::string& operation_key, bool success);
    std::string makeSyncOperationKey(JointId joint_id, MksCommand command);
    
    // Utility functions
    static uint8_t jointIdToCanId(JointId joint_id) { return static_cast<uint8_t>(joint_id); }
    static JointId canIdToJointId(uint8_t can_id) { return static_cast<JointId>(can_id); }
};

} // namespace mks

#endif // MKS_CAN_INTERFACE_HPP
