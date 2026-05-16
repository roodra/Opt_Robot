#include "SwingUpProblem.h"

#include <cmath>

SwingUpProblem::SwingUpProblem(IPhysicsEngine& engine,
                               int horizon, int n_segments,
                               double dt, double max_torque)
    : m_engine(engine)
    , m_horizon(horizon)
    , m_n_segments(n_segments)
    , m_steps_per_segment(horizon / n_segments)
    , m_dt(dt)
    , m_max_torque(max_torque)
{}

double SwingUpProblem::evaluate(const std::vector<double>& params) {
    m_engine.setState({0.0}, {0.0});

    double dense_cost = 0.0;

    for (int seg = 0; seg < m_n_segments; ++seg) {
        double tau = params[seg];
        m_engine.setControl({tau});

        for (int s = 0; s < m_steps_per_segment; ++s) {
            m_engine.step(m_dt);

            double theta = m_engine.getJointPositions()[0];
            double a = (1.0 + std::cos(theta));
            dense_cost += a * a;              // progress signal, no large weight
            dense_cost += 0.001 * tau * tau;  // tiny effort penalty
        }
    }

    // Normalise dense term so its scale doesn't depend on horizon length.
    // Max possible: 4 * horizon. Typical good solution: < 1.0.
    dense_cost /= m_horizon;

    // --- Terminal cost (dominant) ---
    // The optimizer must end near upright with low velocity.
    // Bad (hanging):  100 * 4 + 0 = 400
    // Good (upright): 100 * 0 + 0 = 0
    double theta_f = m_engine.getJointPositions()[0];
    double omega_f = m_engine.getJointVelocities()[0];
    double ta = (1.0 + std::cos(theta_f));
    double terminal_cost = 100.0 * ta * ta + 10.0 * omega_f * omega_f;

    return dense_cost + terminal_cost;
}

bool SwingUpProblem::isFeasible(const std::vector<double>& params) const {
    (void)params;
    return true;
}

int SwingUpProblem::getDimension() const { return m_n_segments; }

std::vector<double> SwingUpProblem::getLowerBounds() const {
    return std::vector<double>(m_n_segments, -m_max_torque);
}

std::vector<double> SwingUpProblem::getUpperBounds() const {
    return std::vector<double>(m_n_segments, m_max_torque);
}
