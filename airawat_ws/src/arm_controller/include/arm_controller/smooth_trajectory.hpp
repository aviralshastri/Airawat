#ifndef SMOOTH_TRAJECTORY_HPP
#define SMOOTH_TRAJECTORY_HPP

#include <vector>
#include <cmath>

/**
 * @brief Generates a smooth trajectory from current angle to target angle
 * 
 * Uses a quintic (5th-order) polynomial for smooth ease-in/ease-out motion with:
 * - Zero velocity at start and end
 * - Zero acceleration at start and end
 * - Continuous jerk throughout
 * 
 * @param current_angle Starting angle (radians or degrees)
 * @param target_angle Ending angle (radians or degrees)
 * @param duration_sec Total time for motion (seconds)
 * @param publish_rate Frequency of angle commands (Hz), typically 100
 * @return std::vector<double> Array of angles to publish at each time step
 */
std::vector<double> generateSmoothTrajectory(
    double current_angle,
    double target_angle, 
    double duration_sec,
    double publish_rate = 100.0);

/**
 * @brief Alternative: Cubic easing (simpler, still smooth)
 * 
 * Uses a cubic polynomial for smoother motion than linear, but simpler than quintic.
 * Has zero velocity at endpoints, but non-zero acceleration.
 * 
 * @param current_angle Starting angle
 * @param target_angle Ending angle
 * @param duration_sec Duration in seconds
 * @param publish_rate Publishing frequency (Hz)
 * @return std::vector<double> Trajectory samples
 */
std::vector<double> generateSmoothTrajectoryCubic(
    double current_angle,
    double target_angle,
    double duration_sec,
    double publish_rate = 100.0);

/**
 * @brief Helper: Calculate instantaneous velocity at any point
 * 
 * Useful for debugging or monitoring motion profile
 * 
 * @param trajectory Generated trajectory
 * @param index Sample index
 * @param publish_rate Frequency (Hz)
 * @return double Velocity in angle_units/second
 */
double getTrajectoryVelocity(
    const std::vector<double>& trajectory,
    size_t index,
    double publish_rate = 100.0);

#endif // SMOOTH_TRAJECTORY_HPP