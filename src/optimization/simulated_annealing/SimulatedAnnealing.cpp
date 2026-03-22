#include "SimulatedAnnealing.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>

SimulatedAnnealing::SimulatedAnnealing(int max_iterations, double initial_temp,
                                       double cooling_rate, double perturbation_scale,
                                       unsigned int seed)
    : m_max_iterations(max_iterations)
    , m_initial_temp(initial_temp)
    , m_cooling_rate(cooling_rate)
    , m_perturbation_scale(perturbation_scale)
    , m_seed(seed)
{}

OptimizationResult SimulatedAnnealing::optimize(IProblem& problem) {
    const int dim = problem.getDimension();
    const auto lo = problem.getLowerBounds();
    const auto hi = problem.getUpperBounds();

    std::mt19937 rng(m_seed);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double>       gauss(0.0, m_perturbation_scale);
    std::uniform_int_distribution<int>     idx_dist(0, dim - 1);
    std::uniform_int_distribution<int>     num_perturb_dist(1, std::min(5, dim));

    // Start from a random feasible solution
    std::vector<double> current(dim);
    for (int i = 0; i < dim; ++i) {
        std::uniform_real_distribution<double> d(lo[i], hi[i]);
        current[i] = d(rng);
    }
    double current_cost = problem.evaluate(current);

    auto   best      = current;
    double best_cost = current_cost;

    double T = m_initial_temp;

    for (int iter = 0; iter < m_max_iterations; ++iter) {
        // Generate neighbour: perturb a random subset of dimensions
        auto neighbor = current;
        int  n_perturb = num_perturb_dist(rng);
        for (int k = 0; k < n_perturb; ++k) {
            int i = idx_dist(rng);
            neighbor[i] = std::clamp(neighbor[i] + gauss(rng), lo[i], hi[i]);
        }

        double neighbor_cost = problem.evaluate(neighbor);
        double delta         = neighbor_cost - current_cost;

        // Accept unconditionally if better; otherwise accept with Boltzmann probability
        if (delta < 0.0 || uniform(rng) < std::exp(-delta / T)) {
            current      = neighbor;
            current_cost = neighbor_cost;
        }

        if (current_cost < best_cost) {
            best      = current;
            best_cost = current_cost;
        }

        // Geometric cooling
        T *= m_cooling_rate;

        if (iter % 500 == 0)
            std::cout << "[SA] iter " << iter
                      << "  T: "    << T
                      << "  best: " << best_cost << "\n";
    }

    return {best, best_cost, m_max_iterations, best_cost < 0.1};
}
