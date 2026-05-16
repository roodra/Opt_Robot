#include <iostream>

#include "physics/mujoco/MuJoCoEngine.h"
#include "problems/swingup/SwingUpProblem.h"
#include "optimization/random_search/RandomSearch.h"
#include "optimization/simulated_annealing/SimulatedAnnealing.h"
#include "optimization/genetic/GeneticAlgorithm.h"
#include "viewer/MuJoCoViewer.h"

static void report(const std::string& label, const OptimizationResult& r,
                   MuJoCoEngine& engine, int horizon, int n_segments, double dt) {
    int steps_per_seg = horizon / n_segments;
    engine.setState({0.0}, {0.0});
    for (int seg = 0; seg < n_segments; ++seg)
        for (int s = 0; s < steps_per_seg; ++s) {
            engine.setControl({r.solution[seg]});
            engine.step(dt);
        }
    double theta = engine.getJointPositions()[0];
    std::cout << "\n--- " << label << " Result ---\n"
              << "Best cost : " << r.cost                        << "\n"
              << "Converged : " << std::boolalpha << r.converged << "\n"
              << "Iterations: " << r.iterations                  << "\n"
              << "Final theta (rad): " << theta
              << "  (target ±pi = ±3.14159)\n\n";
}

static void launch_viewer(const OptimizationResult& r, MuJoCoEngine& engine,
                           int horizon, int n_segments, double dt) {
    int steps_per_seg = horizon / n_segments;
    std::vector<std::vector<double>> controls;
    controls.reserve(horizon);
    for (int seg = 0; seg < n_segments; ++seg)
        for (int s = 0; s < steps_per_seg; ++s)
            controls.push_back({r.solution[seg]});

    std::cout << "Opening viewer — scroll to zoom, ESC to close.\n";
    MuJoCoViewer viewer(engine);
    viewer.replay(controls, dt);
}

int main() {
    MuJoCoEngine engine;
    if (!engine.loadModel("robots/pendulum.xml")) {
        std::cerr << "Failed to load pendulum model.\n";
        return 1;
    }

    const int    horizon     = 200;   // total simulation steps (4 seconds at dt=0.02)
    const int    n_segments  = 10;    // decision variables: 10 piecewise-constant torques
    const double dt          = 0.02;
    const double max_torque  = 5.0;

    SwingUpProblem problem(engine, horizon, n_segments, dt, max_torque);

    // --- Baseline: Random Search ---
    std::cout << "=== Random Search (baseline) ===\n";
    RandomSearch rs(500, 42);
    auto rs_result = rs.optimize(problem);
    report("Random Search", rs_result, engine, horizon, n_segments, dt);

    // --- Simulated Annealing ---
    std::cout << "=== Simulated Annealing ===\n";
    SimulatedAnnealing sa(5000, 5.0, 0.995, 1.0, 42);
    auto sa_result = sa.optimize(problem);
    report("Simulated Annealing", sa_result, engine, horizon, n_segments, dt);

    // --- Genetic Algorithm ---
    std::cout << "=== Genetic Algorithm ===\n";
    GeneticAlgorithm ga(/*population=*/100, /*generations=*/200,
                        /*crossover_rate=*/0.8, /*mutation_rate=*/0.1,
                        /*mutation_scale=*/0.5, /*tournament_size=*/5,
                        /*elite_count=*/5, /*seed=*/42);
    auto ga_result = ga.optimize(problem);
    report("Genetic Algorithm", ga_result, engine, horizon, n_segments, dt);

    // Replay whichever algorithm found the lowest cost
    const OptimizationResult* best_result = &rs_result;
    std::string best_label = "Random Search";
    if (sa_result.cost < best_result->cost) { best_result = &sa_result; best_label = "Simulated Annealing"; }
    if (ga_result.cost < best_result->cost) { best_result = &ga_result; best_label = "Genetic Algorithm"; }

    std::cout << "Replaying best overall result: " << best_label
              << "  (cost " << best_result->cost << ")\n";
    launch_viewer(*best_result, engine, horizon, n_segments, dt);

    return 0;
}
