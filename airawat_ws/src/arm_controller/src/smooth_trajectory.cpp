#include "arm_controller/smooth_trajectory.hpp"


std::vector<double> generateSmoothTrajectory(
    double current_angle,
    double target_angle, 
    double duration_sec,
    double publish_rate)
{
    // Calculate total number of samples
    int num_samples = static_cast<int>(std::ceil(duration_sec * publish_rate));
    
    // Handle edge cases
    if (num_samples <= 0)
    {
        return {target_angle};
    }
    
    if (num_samples == 1)
    {
        return {target_angle};
    }
    
    // Pre-allocate output vector
    std::vector<double> trajectory;
    trajectory.reserve(num_samples);
    
    // Calculate angle difference
    double angle_delta = target_angle - current_angle;
    
    // Generate trajectory using quintic polynomial
    // s(t) = 6t^5 - 15t^4 + 10t^3
    // This gives smooth ease-in/ease-out with zero velocity and acceleration at endpoints
    for (int i = 0; i < num_samples; ++i)
    {
        // Normalized time: 0.0 to 1.0
        double t = static_cast<double>(i) / (num_samples - 1);
        
        // Quintic smoothing function (5th order polynomial)
        // Provides C2 continuity (continuous position, velocity, acceleration)
        double s = 6.0 * std::pow(t, 5) - 15.0 * std::pow(t, 4) + 10.0 * std::pow(t, 3);
        
        // Interpolate angle
        double angle = current_angle + s * angle_delta;
        
        trajectory.push_back(angle);
    }
    
    // Ensure final angle is exactly the target (avoid floating point errors)
    trajectory.back() = target_angle;
    
    return trajectory;
}

std::vector<double> generateSmoothTrajectoryCubic(
    double current_angle,
    double target_angle,
    double duration_sec,
    double publish_rate)
{
    int num_samples = static_cast<int>(std::ceil(duration_sec * publish_rate));
    
    if (num_samples <= 0)
        return {target_angle};
    
    if (num_samples == 1)
        return {target_angle};
    
    std::vector<double> trajectory;
    trajectory.reserve(num_samples);
    
    double angle_delta = target_angle - current_angle;
    
    for (int i = 0; i < num_samples; ++i)
    {
        double t = static_cast<double>(i) / (num_samples - 1);
        
        // Cubic smoothstep: s(t) = 3t^2 - 2t^3
        double s = 3.0 * t * t - 2.0 * t * t * t;
        
        double angle = current_angle + s * angle_delta;
        trajectory.push_back(angle);
    }
    
    trajectory.back() = target_angle;
    
    return trajectory;
}

double getTrajectoryVelocity(
    const std::vector<double>& trajectory,
    size_t index,
    double publish_rate)
{
    if (trajectory.size() < 2 || index >= trajectory.size() - 1)
        return 0.0;
    
    double dt = 1.0 / publish_rate;
    return (trajectory[index + 1] - trajectory[index]) / dt;
}