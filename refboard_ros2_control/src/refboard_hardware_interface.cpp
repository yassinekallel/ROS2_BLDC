#include "refboard_hardware_interface.hpp"
#include <cmath>
#include <map>

namespace refboard_ros2_control {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn RefBoardHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Get CAN interface name from hardware parameters
    can_interface_name_ = info_.hardware_parameters["can_interface"];
    
    // Map joint names to MKS Joint IDs
    std::map<std::string, mks::JointId> joint_name_to_id = {
        {"joint_x", mks::JointId::X_MOTOR},
        {"joint_y", mks::JointId::Y_MOTOR}, 
        {"joint_z", mks::JointId::Z_MOTOR},
        {"joint_a", mks::JointId::A_MOTOR},
        {"joint_b", mks::JointId::B_MOTOR},
        {"joint_c", mks::JointId::C_MOTOR}
    };

    // Initialize joints from hardware info
    for (const auto& joint_info : info_.joints) {
        auto it = joint_name_to_id.find(joint_info.name);
        if (it != joint_name_to_id.end()) {
            joints_.emplace_back(it->second, joint_info.name);
            RCLCPP_INFO(
                rclcpp::get_logger("RefBoardHardwareInterface"),
                "Initialized joint: %s with ID: %d", 
                joint_info.name.c_str(), 
                static_cast<int>(it->second)
            );
        } else {
            RCLCPP_ERROR(
                rclcpp::get_logger("RefBoardHardwareInterface"),
                "Unknown joint name: %s", 
                joint_info.name.c_str()
            );
            return CallbackReturn::ERROR;
        }
    }

    active_ = false;
    
    RCLCPP_INFO(
        rclcpp::get_logger("RefBoardHardwareInterface"),
        "RefBoard Hardware Interface initialized with %zu joints on CAN interface: %s",
        joints_.size(),
        can_interface_name_.c_str()
    );

    return CallbackReturn::SUCCESS;
}

CallbackReturn RefBoardHardwareInterface::on_configure(const State&) {
    // Initialize MKS CAN interface
    if (!mks_can_.init(can_interface_name_)) {
        RCLCPP_ERROR(
            rclcpp::get_logger("RefBoardHardwareInterface"),
            "Failed to initialize MKS CAN interface on %s",
            can_interface_name_.c_str()
        );
        return CallbackReturn::ERROR;
    }

    // Set up callbacks
    mks_can_.setPositionCallback(
        std::bind(&RefBoardHardwareInterface::onPositionReceived, this, 
                 std::placeholders::_1, std::placeholders::_2)
    );
    
    mks_can_.setEnableStatusCallback(
        std::bind(&RefBoardHardwareInterface::onEnableStatusReceived, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );
    
    mks_can_.setGenericResponseCallback(
        std::bind(&RefBoardHardwareInterface::onGenericResponseReceived, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );

    RCLCPP_INFO(
        rclcpp::get_logger("RefBoardHardwareInterface"), 
        "MKS CAN interface configured on %s", 
        can_interface_name_.c_str()
    );
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn RefBoardHardwareInterface::on_cleanup(const State&) {
    mks_can_.deinit();
    RCLCPP_INFO(rclcpp::get_logger("RefBoardHardwareInterface"), "RefBoard interface cleaned up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RefBoardHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("RefBoardHardwareInterface"), "Activating RefBoard motors...");
    
    active_ = true;
    
    // Enable all motors
    if (!enableAllMotors()) {
        RCLCPP_ERROR(rclcpp::get_logger("RefBoardHardwareInterface"), "Failed to enable motors");
        return CallbackReturn::ERROR;
    }
    
    // Read initial positions
    for (auto& joint : joints_) {
        mks_can_.readPosition(joint.joint_id);
    }
    
    RCLCPP_INFO(rclcpp::get_logger("RefBoardHardwareInterface"), "RefBoard motors activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RefBoardHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("RefBoardHardwareInterface"), "Deactivating RefBoard motors...");
    
    active_ = false;
    
    // Disable all motors
    disableAllMotors();
    
    RCLCPP_INFO(rclcpp::get_logger("RefBoardHardwareInterface"), "RefBoard motors deactivated");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RefBoardHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto& joint : joints_) {
        // Position state
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name,
            hardware_interface::HW_IF_POSITION,
            &joint.pos_estimate_
        ));
        
        // Velocity state (estimated from position changes)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name,
            hardware_interface::HW_IF_VELOCITY,
            &joint.vel_estimate_
        ));
        
        // Effort state (not directly available from MKS, estimated)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.name,
            hardware_interface::HW_IF_EFFORT,
            &joint.effort_estimate_
        ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RefBoardHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto& joint : joints_) {
        // Position command
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name,
            hardware_interface::HW_IF_POSITION,
            &joint.pos_setpoint_
        ));
        
        // Velocity command (not directly supported by MKS, but exported for compatibility)
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name,
            hardware_interface::HW_IF_VELOCITY,
            &joint.vel_setpoint_
        ));
        
        // Effort command (not supported by MKS, but exported for compatibility)
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.name,
            hardware_interface::HW_IF_EFFORT,
            &joint.effort_setpoint_
        ));
    }

    return command_interfaces;
}

return_type RefBoardHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    // Update control mode flags based on interface switches
    for (auto& joint : joints_) {
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{joint.name + "/" + hardware_interface::HW_IF_POSITION, &joint.pos_input_enabled_},
             {joint.name + "/" + hardware_interface::HW_IF_VELOCITY, &joint.vel_input_enabled_},
             {joint.name + "/" + hardware_interface::HW_IF_EFFORT, &joint.effort_input_enabled_}}
        };

        // Disable stopped interfaces
        for (const std::string& key : stop_interfaces) {
            for (auto& [interface_name, enabled_flag] : interfaces) {
                if (interface_name == key) {
                    *enabled_flag = false;
                    RCLCPP_INFO(
                        rclcpp::get_logger("RefBoardHardwareInterface"),
                        "Stopped interface: %s", key.c_str()
                    );
                }
            }
        }

        // Enable started interfaces
        for (const std::string& key : start_interfaces) {
            for (auto& [interface_name, enabled_flag] : interfaces) {
                if (interface_name == key) {
                    *enabled_flag = true;
                    RCLCPP_INFO(
                        rclcpp::get_logger("RefBoardHardwareInterface"),
                        "Started interface: %s", key.c_str()
                    );
                }
            }
        }
    }

    return return_type::OK;
}

return_type RefBoardHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    last_read_time_ = time;
    
    // Request position updates from all motors
    for (auto& joint : joints_) {
        mks_can_.readPosition(joint.joint_id);
    }
    
    // Estimate velocity from position changes
    static std::map<mks::JointId, double> prev_positions;
    static rclcpp::Time prev_time = time;
    
    double dt = (time - prev_time).seconds();
    if (dt > 0.001) { // Avoid division by zero
        for (auto& joint : joints_) {
            if (!std::isnan(joint.pos_estimate_) && prev_positions.count(joint.joint_id)) {
                joint.vel_estimate_ = (joint.pos_estimate_ - prev_positions[joint.joint_id]) / dt;
            }
            prev_positions[joint.joint_id] = joint.pos_estimate_;
        }
        prev_time = time;
    }
    
    return return_type::OK;
}

return_type RefBoardHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    if (!active_) {
        return return_type::OK;
    }
    
    for (auto& joint : joints_) {
        // MKS protocol primarily supports position control
        if (joint.pos_input_enabled_) {
            int32_t position_steps = radiansToSteps(joint.pos_setpoint_);
            
            // Send position command
            if (!mks_can_.setPosition(joint.joint_id, position_steps, DEFAULT_SPEED)) {
                RCLCPP_WARN(
                    rclcpp::get_logger("RefBoardHardwareInterface"),
                    "Failed to send position command to joint %s", 
                    joint.name.c_str()
                );
            }
        }
        // Note: MKS protocol doesn't directly support velocity or effort control
        // These could be implemented using position control with calculated setpoints
    }
    
    return return_type::OK;
}

// Callback implementations
void RefBoardHardwareInterface::onPositionReceived(mks::JointId joint_id, const mks::PositionData& data) {
    Joint* joint = getJointById(joint_id);
    if (joint && data.valid) {
        joint->pos_estimate_ = stepsToRadians(data.position);
        joint->last_position_steps_ = data.position;
        
        // Estimate effort from speed (very rough approximation)
        joint->effort_estimate_ = data.speed * 0.001; // Placeholder calculation
    }
}

void RefBoardHardwareInterface::onEnableStatusReceived(mks::JointId joint_id, bool enabled, bool error) {
    Joint* joint = getJointById(joint_id);
    if (joint) {
        joint->motor_enabled_ = enabled && !error;
        
        if (error) {
            RCLCPP_WARN(
                rclcpp::get_logger("RefBoardHardwareInterface"),
                "Motor error detected for joint %s", 
                joint->name.c_str()
            );
        }
    }
}

void RefBoardHardwareInterface::onGenericResponseReceived(mks::JointId joint_id, mks::MksCommand command, bool success) {
    if (!success) {
        Joint* joint = getJointById(joint_id);
        if (joint) {
            RCLCPP_WARN(
                rclcpp::get_logger("RefBoardHardwareInterface"),
                "Command failed for joint %s, command: %d", 
                joint->name.c_str(), 
                static_cast<int>(command)
            );
        }
    }
}

// Helper function implementations
Joint* RefBoardHardwareInterface::getJointById(mks::JointId joint_id) {
    for (auto& joint : joints_) {
        if (joint.joint_id == joint_id) {
            return &joint;
        }
    }
    return nullptr;
}

bool RefBoardHardwareInterface::enableAllMotors() {
    bool success = true;
    for (auto& joint : joints_) {
        if (!mks_can_.enableMotorSync(joint.joint_id, 2000)) { // 2 second timeout
            RCLCPP_ERROR(
                rclcpp::get_logger("RefBoardHardwareInterface"),
                "Failed to enable motor for joint %s", 
                joint.name.c_str()
            );
            success = false;
        } else {
            joint.motor_enabled_ = true;
        }
    }
    return success;
}

bool RefBoardHardwareInterface::disableAllMotors() {
    bool success = true;
    for (auto& joint : joints_) {
        if (!mks_can_.disableMotorSync(joint.joint_id, 2000)) { // 2 second timeout
            RCLCPP_WARN(
                rclcpp::get_logger("RefBoardHardwareInterface"),
                "Failed to disable motor for joint %s", 
                joint.name.c_str()
            );
            success = false;
        } else {
            joint.motor_enabled_ = false;
        }
    }
    return success;
}

double RefBoardHardwareInterface::stepsToRadians(int32_t steps) {
    return (static_cast<double>(steps) / STEPS_PER_REVOLUTION) * 2.0 * M_PI;
}

int32_t RefBoardHardwareInterface::radiansToSteps(double radians) {
    return static_cast<int32_t>((radians / (2.0 * M_PI)) * STEPS_PER_REVOLUTION);
}

} // namespace refboard_ros2_control

PLUGINLIB_EXPORT_CLASS(refboard_ros2_control::RefBoardHardwareInterface, hardware_interface::SystemInterface)
