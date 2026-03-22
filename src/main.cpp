#include <iostream>

#include "physics/mujoco/MuJoCoEngine.h"
#include "problems/swingup/SwingUpProblem.h"
#include "optimization/random_search/RandomSearch.h"

int main() {
    // --- Physics engine ---
    MuJoCoEngine engine;
    if (!engine.loadModel("robots/pendulum.xml")) {
        std::cerr << "Failed to load pendulum model.\n";
        return 1;
    }

    // --- Problem: swing the pendulum from hanging (theta=0) to upright (theta=pi) ---
    // horizon=100 steps x dt=0.02s = 2 seconds of simulation per evaluation
    SwingUpProblem problem(engine, /*horizon=*/100, /*dt=*/0.02, /*max_torque=*/5.0);

    // --- Optimiser: random search (baseline) ---
    RandomSearch optimizer(/*max_iterations=*/500, /*seed=*/42);

    std::cout << "Running Random Search on pendulum swing-up...\n";
    auto result = optimizer.optimize(problem);

    std::cout << "\n--- Result ---\n"
              << "Best cost : " << result.cost      << "\n"
              << "Converged : " << std::boolalpha << result.converged << "\n"
              << "Iterations: " << result.iterations << "\n";

    // Report the final state achieved by the best torque sequence
    engine.setState({0.0}, {0.0});
    for (int i = 0; i < (int)result.solution.size(); ++i) {
        engine.setControl({result.solution[i]});
        engine.step(0.02);
    }
    double theta_final = engine.getJointPositions()[0];
    std::cout << "Final theta (rad): " << theta_final
              << "  (pi = " << 3.14159 << ")\n";

    return 0;
}
