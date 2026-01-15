// ============================================================================
// FILE: include/arm_controller/arm_controller.hpp
// ============================================================================

#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <map>

#include "arm_controller/smooth_trajectory.hpp"

// Custom service definitions
#include "arm_controller/srv/set_joint_angle.hpp"
#include "arm_controller/srv/set_arm_state.hpp"
#include "arm_controller/srv/emergency_stop.hpp"
#include "arm_controller/srv/get_current_angle.hpp"

namespace arm_controller
{

// Command flag enumeration - single flag to control all operations
enum class CommandFlag
{
    NULL_STATE,      // Continue current operation
    EMERGENCY,       // Stop immediately and freeze
    IDEAL,           // Execute IDEAL sequence
    DIG,             // Execute DIG sequence
    DUMP,            // Execute DUMP sequence
    SET_WRIST,       // Set wrist angle
    SET_SHOULDER     // Set shoulder angle
};

// Smoothing type enumeration
enum class SmoothingType
{
    QUINTIC,
    CUBIC
};

// Structure to hold a single state configuration
struct StateConfig
{
    std::vector<double> angles;      // [wrist, shoulder, ...]
    std::vector<double> durations;   // Duration for each joint
    double delay;                     // Delay after reaching this state
};

// Structure to hold single joint angle command
struct SingleJointCommand
{
    size_t joint_index;
    double target_angle;
    double duration;
    SmoothingType smoothing;
};

class ArmController : public rclcpp::Node
{
public:
    ArmController();
    ~ArmController();

private:
    // ========== Initialization ==========
    void loadParameters();
    void setupPublishersAndServices();
    void moveToInitPosition();

    // ========== Service Callbacks ==========
    void handleSetWristAngle(
        const std::shared_ptr<arm_controller::srv::SetJointAngle::Request> request,
        std::shared_ptr<arm_controller::srv::SetJointAngle::Response> response);

    void handleSetShoulderAngle(
        const std::shared_ptr<arm_controller::srv::SetJointAngle::Request> request,
        std::shared_ptr<arm_controller::srv::SetJointAngle::Response> response);

    void handleSetArmState(
        const std::shared_ptr<arm_controller::srv::SetArmState::Request> request,
        std::shared_ptr<arm_controller::srv::SetArmState::Response> response);

    void handleEmergencyStop(
        const std::shared_ptr<arm_controller::srv::EmergencyStop::Request> request,
        std::shared_ptr<arm_controller::srv::EmergencyStop::Response> response);

    void handleGetCurrentAngle(
        const std::shared_ptr<arm_controller::srv::GetCurrentAngle::Request> request,
        std::shared_ptr<arm_controller::srv::GetCurrentAngle::Response> response);

    // ========== Core State Execution Functions ==========
    void idealStateTrigger();
    void digStateTrigger();
    void dumpStateTrigger();
    void setJointAngle(size_t joint_index, double target_angle, double duration, SmoothingType smoothing);

    // ========== Command Processing ==========
    void commandProcessingLoop();
    bool checkAndHandleFlag();
    void clearFlag();

    // ========== State Sequence Execution ==========
    bool executeStateSequence(const std::map<std::string, StateConfig>& states);
    bool transitionToAngles(const std::vector<double>& target_angles, 
                           const std::vector<double>& durations,
                           SmoothingType smoothing);
    bool sleepWithFlagCheck(double seconds);

    // ========== Publishing ==========
    void publishJointAngles(const std::vector<double>& angles);

    // ========== Utility ==========
    SmoothingType parseSmoothingType(const std::string& type_str);
    std::string commandFlagToString(CommandFlag flag);
    void loadStateSequence(const std::string& param_prefix, std::map<std::string, StateConfig>& state_map);

    // ========== ROS2 Components ==========
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    rclcpp::Service<arm_controller::srv::SetJointAngle>::SharedPtr set_wrist_service_;
    rclcpp::Service<arm_controller::srv::SetJointAngle>::SharedPtr set_shoulder_service_;
    rclcpp::Service<arm_controller::srv::SetArmState>::SharedPtr set_arm_state_service_;
    rclcpp::Service<arm_controller::srv::EmergencyStop>::SharedPtr emergency_stop_service_;
    rclcpp::Service<arm_controller::srv::GetCurrentAngle>::SharedPtr get_current_angle_service_;

    // ========== State Variables ==========
    std::vector<std::string> joint_names_;
    std::vector<double> current_angles_;
    std::vector<double> init_angles_;
    
    std::map<std::string, StateConfig> ideal_states_;
    std::map<std::string, StateConfig> dig_states_;
    std::map<std::string, StateConfig> dump_states_;

    double state_transition_duration_;
    double init_angle_duration_;
    double publish_rate_;

    // ========== Command Flag System ==========
    std::mutex command_mutex_;
    std::atomic<CommandFlag> command_flag_;
    SingleJointCommand pending_joint_command_;

    // ========== Threading ==========
    std::thread command_thread_;
    std::atomic<bool> running_;
};

} // namespace arm_controller

#endif // ARM_CONTROLLER_HPP