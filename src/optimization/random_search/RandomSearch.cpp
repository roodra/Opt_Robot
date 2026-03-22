#include "RandomSearch.h"

#include <iostream>
#include <limits>
#include <random>

RandomSearch::RandomSearch(int max_iterations, unsigned int seed)
    : m_max_iterations(max_iterations)
    , m_seed(seed)
{}

OptimizationResult RandomSearch::optimize(IProblem& problem) {
    const int dim = problem.getDimension();
    const auto lo = problem.getLowerBounds();
    const auto hi = problem.getUpperBounds();

    std::mt19937 rng(m_seed);
    std::vector<std::uniform_real_distribution<double>> dists;
    dists.reserve(dim);
    for (int i = 0; i < dim; ++i)
        dists.emplace_back(lo[i], hi[i]);

    std::vector<double> best_params(dim, 0.0);
    double best_cost = std::numeric_limits<double>::max();

    for (int iter = 0; iter < m_max_iterations; ++iter) {
        std::vector<double> candidate(dim);
        for (int i = 0; i < dim; ++i)
            candidate[i] = dists[i](rng);

        double cost = problem.evaluate(candidate);

        if (cost < best_cost) {
            best_cost   = cost;
            best_params = candidate;
        }

        if (iter % 100 == 0)
            std::cout << "[RandomSearch] iter " << iter
                      << "  best cost: " << best_cost << "\n";
    }

    return {best_params, best_cost, m_max_iterations, best_cost < 0.1};
}
