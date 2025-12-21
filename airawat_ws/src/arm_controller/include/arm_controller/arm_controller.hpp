#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>

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
#include "arm_controller/srv/stop_and_go_to_ideal.hpp"
#include "arm_controller/srv/get_current_angle.hpp"

namespace arm_controller
{

// Process state enumeration
enum class ProcessState
{
    IDLE,
    STATE_TRANSITION,
    SET_ANGLE,
    EMERGENCY_STOP,
    STOPPING_TO_IDLE
};

// Arm state enumeration
enum class ArmState
{
    IDEAL,
    DIG,
    DUMP
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

class ArmController : public rclcpp::Node
{
public:
    ArmController();
    ~ArmController();

private:
    // ========== Initialization ==========
    void loadParameters();
    void setupPublishersAndServices();

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

    void handleStopAndGoToIdeal(
        const std::shared_ptr<arm_controller::srv::StopAndGoToIdeal::Request> request,
        std::shared_ptr<arm_controller::srv::StopAndGoToIdeal::Response> response);

    void handleGetCurrentAngle(
        const std::shared_ptr<arm_controller::srv::GetCurrentAngle::Request> request,
        std::shared_ptr<arm_controller::srv::GetCurrentAngle::Response> response);

    // ========== Core Functions ==========
    void setJointAngle(size_t joint_index, double target_angle, double duration, SmoothingType smoothing);
    void executeStateSequence(ArmState state, SmoothingType smoothing);
    void idealStateTrigger(SmoothingType smoothing);
    void digStateTrigger(SmoothingType smoothing);
    void dumpStateTrigger(SmoothingType smoothing);

    // ========== Process Management ==========
    bool trySetProcessState(ProcessState new_state);
    void cancelCurrentProcess();
    void setProcessStateIdle();
    bool shouldStopExecution();

    // ========== Publishing ==========
    void publishJointAngles(const std::vector<double>& angles);
    void executeSmoothTrajectory(
        const std::vector<double>& start_angles,
        const std::vector<double>& target_angles,
        const std::vector<double>& durations,
        SmoothingType smoothing);

    // ========== Utility ==========
    SmoothingType parseSmoothingType(const std::string& type_str);
    std::string processStateToString(ProcessState state);

    // ========== ROS2 Components ==========
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    rclcpp::Service<arm_controller::srv::SetJointAngle>::SharedPtr set_wrist_service_;
    rclcpp::Service<arm_controller::srv::SetJointAngle>::SharedPtr set_shoulder_service_;
    rclcpp::Service<arm_controller::srv::SetArmState>::SharedPtr set_arm_state_service_;
    rclcpp::Service<arm_controller::srv::EmergencyStop>::SharedPtr emergency_stop_service_;
    rclcpp::Service<arm_controller::srv::StopAndGoToIdeal>::SharedPtr stop_and_go_to_ideal_service_;
    rclcpp::Service<arm_controller::srv::GetCurrentAngle>::SharedPtr get_current_angle_service_;

    // ========== State Variables ==========
    std::vector<std::string> joint_names_;
    std::vector<double> current_angles_;
    
    std::map<std::string, StateConfig> ideal_states_;
    std::map<std::string, StateConfig> dig_states_;
    std::map<std::string, StateConfig> dump_states_;

    double state_transition_duration_;
    double publish_rate_;

    // ========== Process Management ==========
    std::mutex process_mutex_;
    std::atomic<ProcessState> current_process_state_;
    std::atomic<bool> emergency_stop_flag_;
    std::atomic<bool> stop_and_go_to_idle_flag_;

    // ========== Threading ==========
    std::thread execution_thread_;
};

} // namespace arm_controller

#endif // ARM_CONTROLLER_HPP