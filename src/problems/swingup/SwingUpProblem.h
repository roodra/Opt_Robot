#pragma once

#include "../../interfaces/IProblem.h"
#include "../../interfaces/IPhysicsEngine.h"

// Pendulum swing-up as an optimisation problem.
//
// Decision variables: `n_segments` piecewise-constant torque values. Each torque
// is held for (horizon / n_segments) simulation steps. This reduces the search
// space from 100D to n_segments-D, making it tractable for population-based methods.
//
// Cost = sum over all steps of:
//          w_angle  * (1 + cos(theta))^2   <- 0 only at upright (theta = pi)
//        + w_vel    * theta_dot^2           <- penalise velocity throughout
//        + w_effort * tau^2                 <- penalise control effort
//
// Using a dense (per-step) cost rather than a sparse (final-only) cost gives the
// optimizer a meaningful signal at every timestep, not just at the end.
//
// Note: evaluate() resets engine state before each simulation — not thread-safe.
class SwingUpProblem : public IProblem {
public:
    SwingUpProblem(IPhysicsEngine& engine,
                   int    horizon    = 100,
                   int    n_segments = 10,
                   double dt         = 0.02,
                   double max_torque = 5.0);

    double evaluate(const std::vector<double>& params) override;
    bool   isFeasible(const std::vector<double>& params) const override;
    int    getDimension() const override;   // = n_segments
    std::vector<double> getLowerBounds() const override;
    std::vector<double> getUpperBounds() const override;

private:
    IPhysicsEngine& m_engine;
    int    m_horizon;
    int    m_n_segments;
    int    m_steps_per_segment;
    double m_dt;
    double m_max_torque;
};
