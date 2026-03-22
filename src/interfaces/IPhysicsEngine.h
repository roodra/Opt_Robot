#pragma once

#include <string>
#include <vector>

// Abstract interface for a physics engine backend.
// Concrete implementations (MuJoCo, Bullet, Pinocchio, etc.) derive from this.
// The rest of the codebase depends only on this interface, never on a specific engine.
class IPhysicsEngine {
public:
    virtual ~IPhysicsEngine() = default;

    // Load a robot model from a URDF file. Returns true on success.
    virtual bool loadModel(const std::string& urdf_path) = 0;

    // Set the full joint state of the robot.
    virtual void setState(const std::vector<double>& joint_positions,
                          const std::vector<double>& joint_velocities) = 0;

    // Get current joint positions.
    virtual std::vector<double> getJointPositions() const = 0;

    // Get the end-effector pose as [x, y, z, qx, qy, qz, qw].
    virtual std::vector<double> getEndEffectorPose() const = 0;

    // Compute the geometric Jacobian at the current state.
    // Returns a flat row-major matrix of size (6 x num_joints).
    virtual std::vector<double> computeJacobian() const = 0;

    // Step the simulation forward by dt seconds.
    virtual void step(double dt) = 0;

    // Return the number of joints (degrees of freedom).
    virtual int getNumJoints() const = 0;
};
