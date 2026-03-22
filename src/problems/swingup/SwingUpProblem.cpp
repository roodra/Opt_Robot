#include "SwingUpProblem.h"

#include <cmath>
#include <numeric>

SwingUpProblem::SwingUpProblem(IPhysicsEngine& engine,
                               int horizon, double dt, double max_torque)
    : m_engine(engine)
    , m_horizon(horizon)
    , m_dt(dt)
    , m_max_torque(max_torque)
{}

double SwingUpProblem::evaluate(const std::vector<double>& params) {
    // Reset to hanging at rest
    m_engine.setState({0.0}, {0.0});

    // Simulate forward, applying one torque per step
    for (int i = 0; i < m_horizon; ++i) {
        m_engine.setControl({params[i]});
        m_engine.step(m_dt);
    }

    // Read final state
    double theta     = m_engine.getJointPositions()[0];
    double theta_dot = m_engine.getJointVelocities()[0];

    // Angle cost: (1 + cos(theta))^2 — equals 0 only when theta = pi (upright)
    double angle_cost = (1.0 + std::cos(theta));
    angle_cost *= angle_cost;

    // Velocity cost: penalise swinging through upright without stopping
    double vel_cost = theta_dot * theta_dot;

    // Control effort: mean squared torque
    double effort = 0.0;
    for (double tau : params) effort += tau * tau;
    effort /= m_horizon;

    return 10.0 * angle_cost + 1.0 * vel_cost + 0.01 * effort;
}

bool SwingUpProblem::isFeasible(const std::vector<double>& params) const {
    // Bounds are enforced by the optimizer; all in-bounds candidates are feasible
    (void)params;
    return true;
}

int SwingUpProblem::getDimension() const {
    return m_horizon;
}

std::vector<double> SwingUpProblem::getLowerBounds() const {
    return std::vector<double>(m_horizon, -m_max_torque);
}

std::vector<double> SwingUpProblem::getUpperBounds() const {
    return std::vector<double>(m_horizon, m_max_torque);
}
