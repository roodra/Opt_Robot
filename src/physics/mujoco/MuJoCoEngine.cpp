#include "MuJoCoEngine.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cstdio>
#include <stdexcept>

// Convenience casts — MuJoCo types are kept out of the header entirely.
static mjModel* model(void* p) { return static_cast<mjModel*>(p); }
static mjData*  data(void* p)  { return static_cast<mjData*>(p);  }

MuJoCoEngine::MuJoCoEngine() = default;

MuJoCoEngine::~MuJoCoEngine() {
    if (m_data)  mj_deleteData(data(m_data));
    if (m_model) mj_deleteModel(model(m_model));
}

bool MuJoCoEngine::loadModel(const std::string& model_path) {
    char error[1000] = {};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!m) {
        std::fprintf(stderr, "MuJoCo error: %s\n", error);
        return false;
    }
    mjData* d = mj_makeData(m);
    if (!d) { mj_deleteModel(m); return false; }
    m_model = m;
    m_data  = d;
    return true;
}

void MuJoCoEngine::setState(const std::vector<double>& joint_positions,
                             const std::vector<double>& joint_velocities) {
    mjModel* m = model(m_model);
    mjData*  d = data(m_data);
    for (int i = 0; i < m->nq && i < (int)joint_positions.size(); ++i)
        d->qpos[i] = joint_positions[i];
    for (int i = 0; i < m->nv && i < (int)joint_velocities.size(); ++i)
        d->qvel[i] = joint_velocities[i];
    mj_forward(m, d);
}

void MuJoCoEngine::setControl(const std::vector<double>& control) {
    mjModel* m = model(m_model);
    mjData*  d = data(m_data);
    for (int i = 0; i < m->nu && i < (int)control.size(); ++i)
        d->ctrl[i] = control[i];
}

void MuJoCoEngine::step(double dt) {
    mjModel* m = model(m_model);
    m->opt.timestep = dt;
    mj_step(m, data(m_data));
}

std::vector<double> MuJoCoEngine::getJointPositions() const {
    mjModel* m = model(m_model);
    mjData*  d = data(m_data);
    return {d->qpos, d->qpos + m->nq};
}

std::vector<double> MuJoCoEngine::getJointVelocities() const {
    mjModel* m = model(m_model);
    mjData*  d = data(m_data);
    return {d->qvel, d->qvel + m->nv};
}

std::vector<double> MuJoCoEngine::getEndEffectorPose() const {
    mjModel* m = model(m_model);
    mjData*  d = data(m_data);
    int site_id = mj_name2id(m, mjOBJ_SITE, "tip");
    if (site_id >= 0) {
        const double* p = d->site_xpos + 3 * site_id;
        return {p[0], p[1], p[2]};
    }
    // Fallback: last body position
    const double* p = d->xpos + 3 * (m->nbody - 1);
    return {p[0], p[1], p[2]};
}

std::vector<double> MuJoCoEngine::computeJacobian() const {
    mjModel* m = model(m_model);
    mjData*  d = data(m_data);
    int site_id = mj_name2id(m, mjOBJ_SITE, "tip");
    if (site_id < 0) return {};
    // Translational (jacp) + rotational (jacr) rows, each 3 x nv
    std::vector<double> jacp(3 * m->nv, 0.0);
    std::vector<double> jacr(3 * m->nv, 0.0);
    mj_jacSite(m, d, jacp.data(), jacr.data(), site_id);
    // Return 6 x nv Jacobian: translational rows first, then rotational
    std::vector<double> jac(6 * m->nv);
    std::copy(jacp.begin(), jacp.end(), jac.begin());
    std::copy(jacr.begin(), jacr.end(), jac.begin() + 3 * m->nv);
    return jac;
}

int MuJoCoEngine::getNumJoints() const {
    return model(m_model)->nv;
}
