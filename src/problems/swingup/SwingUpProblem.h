#pragma once

#include "../../interfaces/IProblem.h"
#include "../../interfaces/IPhysicsEngine.h"

// Pendulum swing-up as an optimisation problem.
//
// Decision variables: a sequence of `horizon` torque values applied at each
// simulation timestep. The optimizer searches for the torque sequence that
// brings the pendulum from hanging (theta = 0) to upright (theta = pi).
//
// Cost = 10 * (1 + cos(theta_final))^2   <- 0 only at upright
//       +  1 * theta_dot_final^2          <- penalise residual velocity
//       + 0.01 * mean(torque^2)           <- penalise control effort
//
// Note: evaluate() resets engine state before each simulation — not thread-safe.
class SwingUpProblem : public IProblem {
public:
    SwingUpProblem(IPhysicsEngine& engine,
                   int    horizon   = 100,
                   double dt        = 0.02,
                   double max_torque = 5.0);

    double evaluate(const std::vector<double>& params) override;
    bool   isFeasible(const std::vector<double>& params) const override;
    int    getDimension() const override;
    std::vector<double> getLowerBounds() const override;
    std::vector<double> getUpperBounds() const override;

private:
    IPhysicsEngine& m_engine;
    int    m_horizon;
    double m_dt;
    double m_max_torque;
};
