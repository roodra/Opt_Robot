#pragma once

#include "../../interfaces/IPhysicsEngine.h"

// MuJoCo implementation of IPhysicsEngine.
// Depends on libmujoco — only this file and its .cpp include MuJoCo headers.
// All other code interacts with this through IPhysicsEngine.
class MuJoCoEngine : public IPhysicsEngine {
public:
    MuJoCoEngine();
    ~MuJoCoEngine() override;

    bool loadModel(const std::string& urdf_path) override;

    void setState(const std::vector<double>& joint_positions,
                  const std::vector<double>& joint_velocities) override;

    std::vector<double> getJointPositions() const override;
    std::vector<double> getEndEffectorPose() const override;
    std::vector<double> computeJacobian() const override;

    void step(double dt) override;
    int getNumJoints() const override;

private:
    // MuJoCo model and data pointers will live here once libmujoco is linked.
    // Using void* as placeholders until the dependency is added to CMakeLists.txt.
    // Replace with: mjModel* m_model; mjData* m_data;
    void* m_model = nullptr;
    void* m_data  = nullptr;
};
