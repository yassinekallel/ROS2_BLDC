#ifndef REFBOARD_HARDWARE_INTERFACE_HPP
#define REFBOARD_HARDWARE_INTERFACE_HPP

#include "mks_can_interface.hpp"
#include "mks_protocol.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace refboard_ros2_control {

class RefBoardHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_configure(const State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    struct Joint {
        mks::JointId joint_id;
        std::string name;
        
        // Commands (ros2_control => RefBoard)
        double pos_setpoint_ = 0.0;      // [rad]
        double vel_setpoint_ = 0.0;      // [rad/s] 
        double effort_setpoint_ = 0.0;   // [Nm] - not used in MKS protocol
        
        // State (RefBoard => ros2_control)
        double pos_estimate_ = NAN;      // [rad]
        double vel_estimate_ = NAN;      // [rad/s]
        double effort_estimate_ = NAN;   // [Nm] - estimated from position control
        
        // Control mode flags
        bool pos_input_enabled_ = false;
        bool vel_input_enabled_ = false;
        bool effort_input_enabled_ = false;
        
        // Motor state
        bool motor_enabled_ = false;
        int32_t last_position_steps_ = 0;
        
        Joint(mks::JointId id, const std::string& joint_name) 
            : joint_id(id), name(joint_name) {}
    };

    // Callbacks for MKS CAN interface
    void onPositionReceived(mks::JointId joint_id, const mks::PositionData& data);
    void onEnableStatusReceived(mks::JointId joint_id, bool enabled, bool error);
    void onGenericResponseReceived(mks::JointId joint_id, mks::MksCommand command, bool success);
    
    // Helper functions
    mks::JointId getJointIdFromName(const std::string& joint_name);
    Joint* getJointById(mks::JointId joint_id);
    bool enableAllMotors();
    bool disableAllMotors();
    double stepsToRadians(int32_t steps);
    int32_t radiansToSteps(double radians);
    
    // Member variables
    bool active_;
    std::string can_interface_name_;
    mks::MksCanInterface mks_can_;
    std::vector<Joint> joints_;
    rclcpp::Time last_read_time_;
    
    // Constants
    static constexpr double STEPS_PER_REVOLUTION = 36000.0;  // From Arduino code
    static constexpr uint16_t DEFAULT_SPEED = 1024;          // From Arduino code
};

} // namespace refboard_ros2_control

#endif // REFBOARD_HARDWARE_INTERFACE_HPP
