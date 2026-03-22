#pragma once

#include "../../interfaces/IPhysicsEngine.h"

// MuJoCo implementation of IPhysicsEngine.
// MuJoCo headers are included only in MuJoCoEngine.cpp — they never leak
// into the rest of the codebase. All interaction goes through IPhysicsEngine.
class MuJoCoEngine : public IPhysicsEngine {
public:
    MuJoCoEngine();
    ~MuJoCoEngine() override;

    bool loadModel(const std::string& model_path) override;

    void setState(const std::vector<double>& joint_positions,
                  const std::vector<double>& joint_velocities) override;

    void setControl(const std::vector<double>& control) override;
    void step(double dt) override;

    std::vector<double> getJointPositions()  const override;
    std::vector<double> getJointVelocities() const override;
    std::vector<double> getEndEffectorPose() const override;
    std::vector<double> computeJacobian()    const override;

    int getNumJoints() const override;

private:
    // Opaque pointers — cast to mjModel*/mjData* inside the .cpp only.
    void* m_model = nullptr;
    void* m_data  = nullptr;
};
