#include "MuJoCoEngine.h"

#include <stdexcept>

// TODO: replace these stubs with real MuJoCo calls once libmujoco is linked.
// Include order will be:
//   #include <mujoco/mujoco.h>
// and replace void* model/data with mjModel*/mjData*.

MuJoCoEngine::MuJoCoEngine() = default;

MuJoCoEngine::~MuJoCoEngine() {
    // TODO: mj_deleteData(m_data); mj_deleteModel(m_model);
}

bool MuJoCoEngine::loadModel(const std::string& urdf_path) {
    // TODO: MuJoCo loads MJCF natively. URDF must first be converted via
    // MuJoCo's compile tool: `compile arm.urdf arm.xml`
    // Then load with: m_model = mj_loadXML("arm.xml", nullptr, nullptr, 0);
    //                 m_data  = mj_makeData(m_model);
    (void)urdf_path;
    return false;
}

void MuJoCoEngine::setState(const std::vector<double>& joint_positions,
                             const std::vector<double>& joint_velocities) {
    // TODO: copy into m_data->qpos and m_data->qvel, then call mj_forward()
    (void)joint_positions;
    (void)joint_velocities;
}

std::vector<double> MuJoCoEngine::getJointPositions() const {
    // TODO: return {m_data->qpos, m_data->qpos + m_model->nq}
    return {};
}

std::vector<double> MuJoCoEngine::getEndEffectorPose() const {
    // TODO: read m_data->xpos and m_data->xquat for the end-effector body
    return {};
}

std::vector<double> MuJoCoEngine::computeJacobian() const {
    // TODO: use mj_jacBody() to compute the geometric Jacobian
    return {};
}

void MuJoCoEngine::step(double dt) {
    // TODO: set m_model->opt.timestep = dt; mj_step(m_model, m_data);
    (void)dt;
}

int MuJoCoEngine::getNumJoints() const {
    // TODO: return m_model->nv;
    return 0;
}
