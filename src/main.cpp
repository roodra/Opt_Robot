#include <iostream>

#include "physics/mujoco/MuJoCoEngine.h"
#include "problems/swingup/SwingUpProblem.h"
#include "optimization/random_search/RandomSearch.h"
#include "optimization/simulated_annealing/SimulatedAnnealing.h"

static void report(const OptimizationResult& r, MuJoCoEngine& engine,
                   int horizon, double dt) {
    std::cout << "\n--- Result ---\n"
              << "Best cost : " << r.cost       << "\n"
              << "Converged : " << std::boolalpha << r.converged << "\n"
              << "Iterations: " << r.iterations  << "\n";

    engine.setState({0.0}, {0.0});
    for (int i = 0; i < horizon; ++i) {
        engine.setControl({r.solution[i]});
        engine.step(dt);
    }
    double theta = engine.getJointPositions()[0];
    std::cout << "Final theta (rad): " << theta
              << "  (target pi = 3.14159)\n\n";
}

int main() {
    MuJoCoEngine engine;
    if (!engine.loadModel("robots/pendulum.xml")) {
        std::cerr << "Failed to load pendulum model.\n";
        return 1;
    }

    const int    horizon    = 100;
    const double dt         = 0.02;
    const double max_torque = 5.0;

    SwingUpProblem problem(engine, horizon, dt, max_torque);

    // --- Baseline: Random Search ---
    std::cout << "=== Random Search (baseline) ===\n";
    RandomSearch rs(/*max_iterations=*/500, /*seed=*/42);
    report(rs.optimize(problem), engine, horizon, dt);

    // --- Simulated Annealing ---
    std::cout << "=== Simulated Annealing ===\n";
    SimulatedAnnealing sa(/*max_iterations=*/5000,
                          /*initial_temp=*/5.0,
                          /*cooling_rate=*/0.995,
                          /*perturbation_scale=*/1.0,
                          /*seed=*/42);
    report(sa.optimize(problem), engine, horizon, dt);

    return 0;
}
