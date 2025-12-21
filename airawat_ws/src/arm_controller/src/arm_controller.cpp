// ============================================================================
// FILE: src/arm_controller.cpp
// ============================================================================

#include "arm_controller/arm_controller.hpp"

namespace arm_controller
{

ArmController::ArmController() 
    : Node("arm_controller"),
      publish_rate_(100.0),
      current_process_state_(ProcessState::IDLE),
      emergency_stop_flag_(false),
      stop_and_go_to_idle_flag_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Arm Controller Node...");
    
    loadParameters();
    setupPublishersAndServices();
    
    RCLCPP_INFO(this->get_logger(), "Arm Controller Node initialized successfully!");
}

ArmController::~ArmController()
{
    // Cancel any ongoing process
    cancelCurrentProcess();
    
    if (execution_thread_.joinable())
    {
        execution_thread_.join();
    }
}

// ========== Initialization ==========

void ArmController::loadParameters()
{
    // Declare and get joint names
    this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{"wrist", "shoulder"});
    joint_names_ = this->get_parameter("joint_names").as_string_array();
    
    // Initialize current angles to zero
    current_angles_.resize(joint_names_.size(), 0.0);
    
    // Get state transition duration
    this->declare_parameter<double>("state_transition_duration", 2.0);
    state_transition_duration_ = this->get_parameter("state_transition_duration").as_double();
    
    // Load ideal states
    this->declare_parameter<std::string>("ideal_states", "");
    this->declare_parameter<std::string>("dig_states", "");
    this->declare_parameter<std::string>("dump_states", "");
    
    // TODO: For complex nested parameters, you might want to load from YAML file directly
    // For now, we'll provide a simplified parameter loading
    // In production, use yaml-cpp to load the full configuration
    
    RCLCPP_INFO(this->get_logger(), "Loaded %zu joints", joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(), "  Joint %zu: %s", i, joint_names_[i].c_str());
    }
}

void ArmController::setupPublishersAndServices()
{
    // Publisher
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_angles", 10);
    
    // Services
    set_wrist_service_ = this->create_service<arm_controller::srv::SetJointAngle>(
        "set_wrist_angle",
        std::bind(&ArmController::handleSetWristAngle, this, std::placeholders::_1, std::placeholders::_2));
    
    set_shoulder_service_ = this->create_service<arm_controller::srv::SetJointAngle>(
        "set_shoulder_angle",
        std::bind(&ArmController::handleSetShoulderAngle, this, std::placeholders::_1, std::placeholders::_2));
    
    set_arm_state_service_ = this->create_service<arm_controller::srv::SetArmState>(
        "set_arm_state",
        std::bind(&ArmController::handleSetArmState, this, std::placeholders::_1, std::placeholders::_2));
    
    emergency_stop_service_ = this->create_service<arm_controller::srv::EmergencyStop>(
        "emergency_stop",
        std::bind(&ArmController::handleEmergencyStop, this, std::placeholders::_1, std::placeholders::_2));
    
    stop_and_go_to_ideal_service_ = this->create_service<arm_controller::srv::StopAndGoToIdeal>(
        "stop_and_go_to_ideal",
        std::bind(&ArmController::handleStopAndGoToIdeal, this, std::placeholders::_1, std::placeholders::_2));
    
    get_current_angle_service_ = this->create_service<arm_controller::srv::GetCurrentAngle>(
        "get_current_angle",
        std::bind(&ArmController::handleGetCurrentAngle, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Services registered successfully");
}

// ========== Service Callbacks ==========

void ArmController::handleSetWristAngle(
    const std::shared_ptr<arm_controller::srv::SetJointAngle::Request> request,
    std::shared_ptr<arm_controller::srv::SetJointAngle::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service: set_wrist_angle called - angle: %.2f, duration: %.2f, smoothing: %s",
                request->angle, request->duration, request->smoothing_type.c_str());
    
    SmoothingType smoothing = parseSmoothingType(request->smoothing_type);
    
    // Find wrist joint index
    auto it = std::find(joint_names_.begin(), joint_names_.end(), "wrist");
    if (it == joint_names_.end())
    {
        response->success = false;
        response->message = "Wrist joint not found in configuration";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    size_t wrist_index = std::distance(joint_names_.begin(), it);
    
    // Cancel current process and start new one
    cancelCurrentProcess();
    
    if (!trySetProcessState(ProcessState::SET_ANGLE))
    {
        response->success = false;
        response->message = "Failed to acquire process lock";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    // Execute in separate thread
    if (execution_thread_.joinable())
        execution_thread_.join();
    
    execution_thread_ = std::thread([this, wrist_index, request, smoothing]() {
        setJointAngle(wrist_index, request->angle, request->duration, smoothing);
    });
    
    response->success = true;
    response->message = "Wrist angle command accepted";
}

void ArmController::handleSetShoulderAngle(
    const std::shared_ptr<arm_controller::srv::SetJointAngle::Request> request,
    std::shared_ptr<arm_controller::srv::SetJointAngle::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service: set_shoulder_angle called - angle: %.2f, duration: %.2f, smoothing: %s",
                request->angle, request->duration, request->smoothing_type.c_str());
    
    SmoothingType smoothing = parseSmoothingType(request->smoothing_type);
    
    auto it = std::find(joint_names_.begin(), joint_names_.end(), "shoulder");
    if (it == joint_names_.end())
    {
        response->success = false;
        response->message = "Shoulder joint not found in configuration";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    size_t shoulder_index = std::distance(joint_names_.begin(), it);
    
    cancelCurrentProcess();
    
    if (!trySetProcessState(ProcessState::SET_ANGLE))
    {
        response->success = false;
        response->message = "Failed to acquire process lock";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    if (execution_thread_.joinable())
        execution_thread_.join();
    
    execution_thread_ = std::thread([this, shoulder_index, request, smoothing]() {
        setJointAngle(shoulder_index, request->angle, request->duration, smoothing);
    });
    
    response->success = true;
    response->message = "Shoulder angle command accepted";
}

void ArmController::handleSetArmState(
    const std::shared_ptr<arm_controller::srv::SetArmState::Request> request,
    std::shared_ptr<arm_controller::srv::SetArmState::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service: set_arm_state called - state: %s, smoothing: %s",
                request->state.c_str(), request->smoothing_type.c_str());
    
    ArmState target_state;
    if (request->state == "IDEAL")
        target_state = ArmState::IDEAL;
    else if (request->state == "DIG")
        target_state = ArmState::DIG;
    else if (request->state == "DUMP")
        target_state = ArmState::DUMP;
    else
    {
        response->success = false;
        response->message = "Invalid state: " + request->state;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    SmoothingType smoothing = parseSmoothingType(request->smoothing_type);
    
    cancelCurrentProcess();
    
    if (!trySetProcessState(ProcessState::STATE_TRANSITION))
    {
        response->success = false;
        response->message = "Failed to acquire process lock";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    if (execution_thread_.joinable())
        execution_thread_.join();
    
    execution_thread_ = std::thread([this, target_state, smoothing]() {
        executeStateSequence(target_state, smoothing);
    });
    
    response->success = true;
    response->message = "State transition started";
}

void ArmController::handleEmergencyStop(
    const std::shared_ptr<arm_controller::srv::EmergencyStop::Request> request,
    std::shared_ptr<arm_controller::srv::EmergencyStop::Response> response)
{
    (void)request; // Unused
    
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP TRIGGERED!");
    
    // Check if already in emergency stop
    if (current_process_state_.load() == ProcessState::EMERGENCY_STOP)
    {
        response->success = false;
        response->message = "Emergency stop already active";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    // Set emergency flag
    emergency_stop_flag_.store(true);
    
    // Wait for current process to detect and stop
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    response->success = true;
    response->message = "Emergency stop executed";
    RCLCPP_INFO(this->get_logger(), "Emergency stop completed");
}

void ArmController::handleStopAndGoToIdeal(
    const std::shared_ptr<arm_controller::srv::StopAndGoToIdeal::Request> request,
    std::shared_ptr<arm_controller::srv::StopAndGoToIdeal::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service: stop_and_go_to_ideal called - smoothing: %s",
                request->smoothing_type.c_str());
    
    SmoothingType smoothing = parseSmoothingType(request->smoothing_type);
    
    // Set stop and go to ideal flag
    stop_and_go_to_idle_flag_.store(true);
    
    // Wait for current process to stop
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Clear flag
    stop_and_go_to_idle_flag_.store(false);
    
    // Start ideal state transition
    if (!trySetProcessState(ProcessState::STOPPING_TO_IDLE))
    {
        response->success = false;
        response->message = "Failed to acquire process lock";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    if (execution_thread_.joinable())
        execution_thread_.join();
    
    execution_thread_ = std::thread([this, smoothing]() {
        idealStateTrigger(smoothing);
    });
    
    response->success = true;
    response->message = "Returning to ideal state";
}

void ArmController::handleGetCurrentAngle(
    const std::shared_ptr<arm_controller::srv::GetCurrentAngle::Request> request,
    std::shared_ptr<arm_controller::srv::GetCurrentAngle::Response> response)
{
    (void)request; // Unused
    
    std::lock_guard<std::mutex> lock(process_mutex_);
    response->joint_names = joint_names_;
    response->angles = current_angles_;
    
    RCLCPP_INFO(this->get_logger(), "Current angles requested");
}

// ========== Core Functions ==========

void ArmController::setJointAngle(size_t joint_index, double target_angle, double duration, SmoothingType smoothing)
{
    RCLCPP_INFO(this->get_logger(), "Executing setJointAngle for joint %zu to %.2f degrees", 
                joint_index, target_angle);
    
    std::vector<double> start_angles = current_angles_;
    std::vector<double> target_angles = current_angles_;
    std::vector<double> durations(joint_names_.size(), duration);
    
    target_angles[joint_index] = target_angle;
    
    executeSmoothTrajectory(start_angles, target_angles, durations, smoothing);
    
    setProcessStateIdle();
}

void ArmController::executeStateSequence(ArmState state, SmoothingType smoothing)
{
    switch (state)
    {
        case ArmState::IDEAL:
            idealStateTrigger(smoothing);
            break;
        case ArmState::DIG:
            digStateTrigger(smoothing);
            break;
        case ArmState::DUMP:
            dumpStateTrigger(smoothing);
            break;
    }
    
    setProcessStateIdle();
}

void ArmController::idealStateTrigger(SmoothingType smoothing)
{
    RCLCPP_INFO(this->get_logger(), "Executing IDEAL state sequence");
    
    // TODO: Load from parameters and execute sequence
    // For now, simplified example with hardcoded values
    
    // Example: Move to state_1
    std::vector<double> target_angles = {10.0, 10.0}; // From params
    std::vector<double> durations = {2.0, 2.0};
    
    // First transition to state_1
    executeSmoothTrajectory(current_angles_, target_angles, durations, smoothing);
    
    if (shouldStopExecution())
    {
        RCLCPP_WARN(this->get_logger(), "IDEAL sequence interrupted");
        return;
    }
    
    // Delay after reaching state_1
    RCLCPP_INFO(this->get_logger(), "Delaying for 1 second...");
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    if (shouldStopExecution())
    {
        RCLCPP_WARN(this->get_logger(), "IDEAL sequence interrupted during delay");
        return;
    }
    
    // TODO: Continue with more states from config file
    // Example: Move to state_2
    // target_angles = {20.0, 20.0};
    // executeSmoothTrajectory(current_angles_, target_angles, durations, smoothing);
    
    RCLCPP_INFO(this->get_logger(), "IDEAL state sequence completed");
}

void ArmController::digStateTrigger(SmoothingType smoothing)
{
    RCLCPP_INFO(this->get_logger(), "Executing DIG state sequence");
    
    // TODO: Load from parameters and execute sequence
    // Similar structure to idealStateTrigger
    
    // Example state sequence
    std::vector<double> target_angles = {30.0, 40.0};
    std::vector<double> durations = {2.5, 2.5};
    
    executeSmoothTrajectory(current_angles_, target_angles, durations, smoothing);
    
    if (shouldStopExecution())
    {
        RCLCPP_WARN(this->get_logger(), "DIG sequence interrupted");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "DIG state sequence completed");
}

void ArmController::dumpStateTrigger(SmoothingType smoothing)
{
    RCLCPP_INFO(this->get_logger(), "Executing DUMP state sequence");
    
    // TODO: Load from parameters and execute sequence
    // Similar structure to idealStateTrigger
    
    // Example state sequence
    std::vector<double> target_angles = {45.0, 45.0};
    std::vector<double> durations = {2.0, 2.0};
    
    executeSmoothTrajectory(current_angles_, target_angles, durations, smoothing);
    
    if (shouldStopExecution())
    {
        RCLCPP_WARN(this->get_logger(), "DUMP sequence interrupted");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "DUMP state sequence completed");
}

// ========== Process Management ==========

bool ArmController::trySetProcessState(ProcessState new_state)
{
    std::lock_guard<std::mutex> lock(process_mutex_);
    current_process_state_.store(new_state);
    RCLCPP_INFO(this->get_logger(), "Process state changed to: %s", 
                processStateToString(new_state).c_str());
    return true;
}

void ArmController::cancelCurrentProcess()
{
    ProcessState current = current_process_state_.load();
    
    if (current != ProcessState::IDLE && current != ProcessState::EMERGENCY_STOP)
    {
        RCLCPP_INFO(this->get_logger(), "Canceling current process: %s", 
                    processStateToString(current).c_str());
        emergency_stop_flag_.store(true);
        
        // Wait for process to stop
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        emergency_stop_flag_.store(false);
        RCLCPP_INFO(this->get_logger(), "Current process canceled");
    }
}

void ArmController::setProcessStateIdle()
{
    std::lock_guard<std::mutex> lock(process_mutex_);
    current_process_state_.store(ProcessState::IDLE);
    RCLCPP_INFO(this->get_logger(), "Process completed, state set to IDLE");
}

bool ArmController::shouldStopExecution()
{
    return emergency_stop_flag_.load() || stop_and_go_to_idle_flag_.load();
}

// ========== Publishing ==========

void ArmController::publishJointAngles(const std::vector<double>& angles)
{
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position = angles;
    
    joint_pub_->publish(msg);
}

void ArmController::executeSmoothTrajectory(
    const std::vector<double>& start_angles,
    const std::vector<double>& target_angles,
    const std::vector<double>& durations,
    SmoothingType smoothing)
{
    RCLCPP_INFO(this->get_logger(), "Generating smooth trajectory...");
    
    // Generate trajectories for each joint
    std::vector<std::vector<double>> trajectories;
    size_t max_samples = 0;
    
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        std::vector<double> traj;
        
        if (smoothing == SmoothingType::QUINTIC)
        {
            traj = generateSmoothTrajectory(start_angles[i], target_angles[i], durations[i], publish_rate_);
            RCLCPP_INFO(this->get_logger(), "Joint %s: QUINTIC trajectory with %zu samples", 
                        joint_names_[i].c_str(), traj.size());
        }
        else
        {
            traj = generateSmoothTrajectoryCubic(start_angles[i], target_angles[i], durations[i], publish_rate_);
            RCLCPP_INFO(this->get_logger(), "Joint %s: CUBIC trajectory with %zu samples", 
                        joint_names_[i].c_str(), traj.size());
        }
        
        trajectories.push_back(traj);
        max_samples = std::max(max_samples, traj.size());
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishing trajectory with %zu samples at %.1f Hz", 
                max_samples, publish_rate_);
    
    // Publish at 100 Hz
    rclcpp::Rate rate(publish_rate_);
    
    for (size_t sample = 0; sample < max_samples; ++sample)
    {
        // Check for stop conditions
        if (shouldStopExecution())
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory execution stopped at sample %zu/%zu", 
                        sample, max_samples);
            
            // Clear flags
            if (emergency_stop_flag_.load())
            {
                emergency_stop_flag_.store(false);
                current_process_state_.store(ProcessState::IDLE);
            }
            
            return;
        }
        
        // Get angles at this sample
        std::vector<double> angles(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            if (sample < trajectories[i].size())
                angles[i] = trajectories[i][sample];
            else
                angles[i] = trajectories[i].back();
        }
        
        // Update current angles
        {
            std::lock_guard<std::mutex> lock(process_mutex_);
            current_angles_ = angles;
        }
        
        // Publish
        publishJointAngles(angles);
        
        rate.sleep();
    }
    
    RCLCPP_INFO(this->get_logger(), "Trajectory execution completed");
}

// ========== Utility ==========

SmoothingType ArmController::parseSmoothingType(const std::string& type_str)
{
    if (type_str == "QUINTIC")
    {
        return SmoothingType::QUINTIC;
    }
    else if (type_str == "CUBIC")
    {
        return SmoothingType::CUBIC;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Unknown smoothing type '%s', defaulting to CUBIC", 
                    type_str.c_str());
        return SmoothingType::CUBIC; // Default
    }
}

std::string ArmController::processStateToString(ProcessState state)
{
    switch (state)
    {
        case ProcessState::IDLE: return "IDLE";
        case ProcessState::STATE_TRANSITION: return "STATE_TRANSITION";
        case ProcessState::SET_ANGLE: return "SET_ANGLE";
        case ProcessState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case ProcessState::STOPPING_TO_IDLE: return "STOPPING_TO_IDLE";
        default: return "UNKNOWN";
    }
}

} // namespace arm_controller

// ========== Main Function ==========

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<arm_controller::ArmController>();
    
    // Use MultiThreadedExecutor for async service calls
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "Arm Controller Node spinning...");
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}