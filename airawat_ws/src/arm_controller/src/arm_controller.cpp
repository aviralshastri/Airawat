// ============================================================================
// FILE: src/arm_controller.cpp
// ============================================================================

#include "arm_controller/arm_controller.hpp"
#include <chrono>

namespace arm_controller
{

ArmController::ArmController() 
    : Node("arm_controller"),
      publish_rate_(100.0),
      command_flag_(CommandFlag::NULL_STATE),
      running_(true)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Arm Controller Node...");
    
    loadParameters();
    setupPublishersAndServices();
    
    // Move to initial position
    moveToInitPosition();
    
    // Start command processing thread
    command_thread_ = std::thread(&ArmController::commandProcessingLoop, this);
    
    RCLCPP_INFO(this->get_logger(), "Arm Controller Node initialized successfully!");
}

ArmController::~ArmController()
{
    running_.store(false);
    
    if (command_thread_.joinable())
    {
        command_thread_.join();
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
    
    // Get init angles
    this->declare_parameter<std::vector<double>>("init_angles", std::vector<double>{0.0, 0.0});
    init_angles_ = this->get_parameter("init_angles").as_double_array();
    
    // Get state transition duration
    this->declare_parameter<double>("state_transition_duration", 5.0);
    state_transition_duration_ = this->get_parameter("state_transition_duration").as_double();
    
    // Get init angle duration
    this->declare_parameter<double>("init_angle_duration", 5.0);
    init_angle_duration_ = this->get_parameter("init_angle_duration").as_double();
    
    // Get publish rate
    this->declare_parameter<double>("publish_rate", 100.0);
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    
    // Load state sequences
    loadStateSequence("ideal_states", ideal_states_);
    loadStateSequence("dig_states", dig_states_);
    loadStateSequence("dump_states", dump_states_);
    
    RCLCPP_INFO(this->get_logger(), "Loaded %zu joints", joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(), "  Joint %zu: %s (init: %.2f°)", 
                    i, joint_names_[i].c_str(), init_angles_[i]);
    }
}

void ArmController::loadStateSequence(const std::string& param_prefix, 
                                     std::map<std::string, StateConfig>& state_map)
{
    // Try to get the number of states
    int state_num = 1;
    while (true)
    {
        std::string state_name = "state_" + std::to_string(state_num);
        std::string angles_param = param_prefix + "." + state_name + ".angles";
        std::string durations_param = param_prefix + "." + state_name + ".durations";
        std::string delay_param = param_prefix + "." + state_name + ".delay";
        
        // Try to declare and get parameters
        try
        {
            this->declare_parameter<std::vector<double>>(angles_param, std::vector<double>{});
            this->declare_parameter<std::vector<double>>(durations_param, std::vector<double>{});
            this->declare_parameter<double>(delay_param, 0.0);
            
            auto angles = this->get_parameter(angles_param).as_double_array();
            auto durations = this->get_parameter(durations_param).as_double_array();
            double delay = this->get_parameter(delay_param).as_double();
            
            if (angles.empty() || durations.empty())
            {
                break; // No more states
            }
            
            StateConfig config;
            config.angles = angles;
            config.durations = durations;
            config.delay = delay;
            
            state_map[state_name] = config;
            
            RCLCPP_INFO(this->get_logger(), "Loaded %s.%s: angles=[%.1f, %.1f], delay=%.1fs",
                        param_prefix.c_str(), state_name.c_str(), 
                        angles[0], angles[1], delay);
            
            state_num++;
        }
        catch (const std::exception& e)
        {
            break; // No more states
        }
    }
    
    if (state_map.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No states loaded for %s", param_prefix.c_str());
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
    
    get_current_angle_service_ = this->create_service<arm_controller::srv::GetCurrentAngle>(
        "get_current_angle",
        std::bind(&ArmController::handleGetCurrentAngle, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Services registered successfully");
}

void ArmController::moveToInitPosition()
{
    RCLCPP_INFO(this->get_logger(), "Moving to initial position...");
    
    // Transition from current (0,0) to init_angles
    transitionToAngles(init_angles_, 
                      std::vector<double>(joint_names_.size(), init_angle_duration_),
                      SmoothingType::QUINTIC);
    
    RCLCPP_INFO(this->get_logger(), "Reached initial position: [%.2f, %.2f]", 
                current_angles_[0], current_angles_[1]);
}

// ========== Service Callbacks ==========

void ArmController::handleSetWristAngle(
    const std::shared_ptr<arm_controller::srv::SetJointAngle::Request> request,
    std::shared_ptr<arm_controller::srv::SetJointAngle::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service: set_wrist_angle called - angle: %.2f, duration: %.2f, smoothing: %s",
                request->angle, request->duration, request->smoothing_type.c_str());
    
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
    SmoothingType smoothing = parseSmoothingType(request->smoothing_type);
    
    // Set pending command
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_joint_command_.joint_index = wrist_index;
        pending_joint_command_.target_angle = request->angle;
        pending_joint_command_.duration = request->duration;
        pending_joint_command_.smoothing = smoothing;
    }
    
    // Set flag to trigger command
    command_flag_.store(CommandFlag::SET_WRIST);
    
    response->success = true;
    response->message = "Wrist angle command accepted";
}

void ArmController::handleSetShoulderAngle(
    const std::shared_ptr<arm_controller::srv::SetJointAngle::Request> request,
    std::shared_ptr<arm_controller::srv::SetJointAngle::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service: set_shoulder_angle called - angle: %.2f, duration: %.2f, smoothing: %s",
                request->angle, request->duration, request->smoothing_type.c_str());
    
    auto it = std::find(joint_names_.begin(), joint_names_.end(), "shoulder");
    if (it == joint_names_.end())
    {
        response->success = false;
        response->message = "Shoulder joint not found in configuration";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    size_t shoulder_index = std::distance(joint_names_.begin(), it);
    SmoothingType smoothing = parseSmoothingType(request->smoothing_type);
    
    // Set pending command
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_joint_command_.joint_index = shoulder_index;
        pending_joint_command_.target_angle = request->angle;
        pending_joint_command_.duration = request->duration;
        pending_joint_command_.smoothing = smoothing;
    }
    
    // Set flag to trigger command
    command_flag_.store(CommandFlag::SET_SHOULDER);
    
    response->success = true;
    response->message = "Shoulder angle command accepted";
}

void ArmController::handleSetArmState(
    const std::shared_ptr<arm_controller::srv::SetArmState::Request> request,
    std::shared_ptr<arm_controller::srv::SetArmState::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Service: set_arm_state called - state: %s, smoothing: %s",
                request->state.c_str(), request->smoothing_type.c_str());
    
    CommandFlag target_flag;
    if (request->state == "IDEAL")
        target_flag = CommandFlag::IDEAL;
    else if (request->state == "DIG")
        target_flag = CommandFlag::DIG;
    else if (request->state == "DUMP")
        target_flag = CommandFlag::DUMP;
    else
    {
        response->success = false;
        response->message = "Invalid state: " + request->state;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    // Set flag
    command_flag_.store(target_flag);
    
    response->success = true;
    response->message = "State transition started";
}

void ArmController::handleEmergencyStop(
    const std::shared_ptr<arm_controller::srv::EmergencyStop::Request> request,
    std::shared_ptr<arm_controller::srv::EmergencyStop::Response> response)
{
    (void)request;
    
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP TRIGGERED!");
    
    // Set emergency flag
    command_flag_.store(CommandFlag::EMERGENCY);
    
    response->success = true;
    response->message = "Emergency stop executed - arm frozen at current position";
    RCLCPP_INFO(this->get_logger(), "Emergency stop completed, flag cleared");
}

void ArmController::handleGetCurrentAngle(
    const std::shared_ptr<arm_controller::srv::GetCurrentAngle::Request> request,
    std::shared_ptr<arm_controller::srv::GetCurrentAngle::Response> response)
{
    (void)request;
    
    std::lock_guard<std::mutex> lock(command_mutex_);
    response->joint_names = joint_names_;
    response->angles = current_angles_;
    
    RCLCPP_INFO(this->get_logger(), "Current angles: [%.2f, %.2f]", 
                current_angles_[0], current_angles_[1]);
}

// ========== Command Processing ==========

void ArmController::commandProcessingLoop()
{
    rclcpp::Rate rate(10); // Check for commands at 10Hz
    
    while (running_.load() && rclcpp::ok())
    {
        CommandFlag current_flag = command_flag_.load();
        
        if (current_flag != CommandFlag::NULL_STATE)
        {
            RCLCPP_INFO(this->get_logger(), "Processing command: %s", 
                        commandFlagToString(current_flag).c_str());
            
            // Handle the command
            switch (current_flag)
            {
                case CommandFlag::EMERGENCY:
                    RCLCPP_WARN(this->get_logger(), "Emergency stop - freezing at current position");
                    clearFlag();
                    break;
                    
                case CommandFlag::IDEAL:
                    clearFlag();
                    idealStateTrigger();
                    break;
                    
                case CommandFlag::DIG:
                    clearFlag();
                    digStateTrigger();
                    break;
                    
                case CommandFlag::DUMP:
                    clearFlag();
                    dumpStateTrigger();
                    break;
                    
                case CommandFlag::SET_WRIST:
                case CommandFlag::SET_SHOULDER:
                {
                    SingleJointCommand cmd;
                    {
                        std::lock_guard<std::mutex> lock(command_mutex_);
                        cmd = pending_joint_command_;
                    }
                    clearFlag();
                    setJointAngle(cmd.joint_index, cmd.target_angle, cmd.duration, cmd.smoothing);
                    break;
                }
                    
                default:
                    break;
            }
        }
        
        rate.sleep();
    }
}

bool ArmController::checkAndHandleFlag()
{
    CommandFlag current_flag = command_flag_.load();
    
    if (current_flag != CommandFlag::NULL_STATE)
    {
        RCLCPP_INFO(this->get_logger(), "Flag detected during operation: %s", 
                    commandFlagToString(current_flag).c_str());
        
        if (current_flag == CommandFlag::EMERGENCY)
        {
            RCLCPP_WARN(this->get_logger(), "Emergency stop - exiting current operation");
            clearFlag();
            return true; // Stop execution
        }
        
        // For other commands, let the command processing loop handle them
        return true; // Stop current execution
    }
    
    return false; // Continue execution
}

void ArmController::clearFlag()
{
    command_flag_.store(CommandFlag::NULL_STATE);
}

// ========== Core State Execution Functions ==========

void ArmController::idealStateTrigger()
{
    RCLCPP_INFO(this->get_logger(), "Executing IDEAL state sequence");
    executeStateSequence(ideal_states_);
    RCLCPP_INFO(this->get_logger(), "IDEAL state sequence completed");
}

void ArmController::digStateTrigger()
{
    RCLCPP_INFO(this->get_logger(), "Executing DIG state sequence");
    executeStateSequence(dig_states_);
    RCLCPP_INFO(this->get_logger(), "DIG state sequence completed");
}

void ArmController::dumpStateTrigger()
{
    RCLCPP_INFO(this->get_logger(), "Executing DUMP state sequence");
    executeStateSequence(dump_states_);
    RCLCPP_INFO(this->get_logger(), "DUMP state sequence completed");
}

void ArmController::setJointAngle(size_t joint_index, double target_angle, 
                                 double duration, SmoothingType smoothing)
{
    RCLCPP_INFO(this->get_logger(), "Setting joint %zu (%s) to %.2f° over %.2fs", 
                joint_index, joint_names_[joint_index].c_str(), target_angle, duration);
    
    std::vector<double> target_angles = current_angles_;
    target_angles[joint_index] = target_angle;
    
    std::vector<double> durations(joint_names_.size(), duration);
    
    transitionToAngles(target_angles, durations, smoothing);
    
    RCLCPP_INFO(this->get_logger(), "Joint angle command completed");
}

// ========== State Sequence Execution ==========

bool ArmController::executeStateSequence(const std::map<std::string, StateConfig>& states)
{
    if (states.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No states to execute in sequence");
        return false;
    }
    
    // Iterate through states in order
    for (const auto& [state_name, config] : states)
    {
        RCLCPP_INFO(this->get_logger(), "Transitioning to %s", state_name.c_str());
        
        // Transition to this state's angles
        if (!transitionToAngles(config.angles, config.durations, SmoothingType::CUBIC))
        {
            RCLCPP_WARN(this->get_logger(), "State sequence interrupted during transition to %s", 
                        state_name.c_str());
            return false;
        }
        
        // Check flag after transition
        if (checkAndHandleFlag())
        {
            return false;
        }
        
        // Delay at this state
        if (config.delay > 0.0)
        {
            RCLCPP_INFO(this->get_logger(), "Holding at %s for %.2fs", state_name.c_str(), config.delay);
            if (!sleepWithFlagCheck(config.delay))
            {
                RCLCPP_WARN(this->get_logger(), "State sequence interrupted during delay at %s", 
                            state_name.c_str());
                return false;
            }
        }
        
        // Check flag after delay
        if (checkAndHandleFlag())
        {
            return false;
        }
    }
    
    return true;
}

bool ArmController::transitionToAngles(const std::vector<double>& target_angles, 
                                      const std::vector<double>& durations,
                                      SmoothingType smoothing)
{
    // Generate trajectories for each joint
    std::vector<std::vector<double>> trajectories;
    size_t max_samples = 0;
    
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        std::vector<double> traj;
        
        if (smoothing == SmoothingType::QUINTIC)
        {
            traj = generateSmoothTrajectory(current_angles_[i], target_angles[i], 
                                           durations[i], publish_rate_);
        }
        else
        {
            traj = generateSmoothTrajectoryCubic(current_angles_[i], target_angles[i], 
                                                durations[i], publish_rate_);
        }
        
        trajectories.push_back(traj);
        max_samples = std::max(max_samples, traj.size());
    }
    
    // Publish trajectory at publish_rate Hz
    rclcpp::Rate rate(publish_rate_);
    
    for (size_t sample = 0; sample < max_samples; ++sample)
    {
        // Check flag
        if (checkAndHandleFlag())
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory interrupted at sample %zu/%zu", 
                        sample, max_samples);
            return false;
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
            std::lock_guard<std::mutex> lock(command_mutex_);
            current_angles_ = angles;
        }
        
        // Publish
        publishJointAngles(angles);
        
        rate.sleep();
    }
    
    return true;
}

bool ArmController::sleepWithFlagCheck(double seconds)
{
    auto start = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration<double>(seconds);
    
    rclcpp::Rate rate(20); // Check flag 20 times per second during sleep
    
    while (std::chrono::steady_clock::now() - start < duration)
    {
        if (checkAndHandleFlag())
        {
            return false;
        }
        rate.sleep();
    }
    
    return true;
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

// ========== Utility ==========

SmoothingType ArmController::parseSmoothingType(const std::string& type_str)
{
    if (type_str == "QUINTIC")
    {
        return SmoothingType::QUINTIC;
    }
    else
    {
        return SmoothingType::CUBIC; // Default
    }
}

std::string ArmController::commandFlagToString(CommandFlag flag)
{
    switch (flag)
    {
        case CommandFlag::NULL_STATE: return "NULL_STATE";
        case CommandFlag::EMERGENCY: return "EMERGENCY";
        case CommandFlag::IDEAL: return "IDEAL";
        case CommandFlag::DIG: return "DIG";
        case CommandFlag::DUMP: return "DUMP";
        case CommandFlag::SET_WRIST: return "SET_WRIST";
        case CommandFlag::SET_SHOULDER: return "SET_SHOULDER";
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