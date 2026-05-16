#include "GeneticAlgorithm.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <numeric>

GeneticAlgorithm::GeneticAlgorithm(int population_size, int generations,
                                   double crossover_rate, double mutation_rate,
                                   double mutation_scale, int tournament_size,
                                   int elite_count, unsigned int seed)
    : m_pop_size(population_size)
    , m_generations(generations)
    , m_crossover_rate(crossover_rate)
    , m_mutation_rate(mutation_rate)
    , m_mutation_scale(mutation_scale)
    , m_tournament_size(tournament_size)
    , m_elite_count(elite_count)
    , m_seed(seed)
{}

// --- Private helpers ---

int GeneticAlgorithm::tournament_select(const std::vector<double>& costs,
                                        std::mt19937& rng) const {
    std::uniform_int_distribution<int> pick(0, static_cast<int>(costs.size()) - 1);
    int best = pick(rng);
    for (int i = 1; i < m_tournament_size; ++i) {
        int candidate = pick(rng);
        if (costs[candidate] < costs[best])
            best = candidate;
    }
    return best;
}

std::vector<double> GeneticAlgorithm::crossover(const std::vector<double>& p1,
                                                 const std::vector<double>& p2,
                                                 std::mt19937& rng) const {
    std::uniform_int_distribution<int> coin(0, 1);
    std::vector<double> child(p1.size());
    for (std::size_t i = 0; i < p1.size(); ++i)
        child[i] = coin(rng) ? p1[i] : p2[i];
    return child;
}

void GeneticAlgorithm::mutate(std::vector<double>& individual,
                               const std::vector<double>& lo,
                               const std::vector<double>& hi,
                               std::mt19937& rng) const {
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double>       gauss(0.0, m_mutation_scale);
    for (std::size_t i = 0; i < individual.size(); ++i) {
        if (uniform(rng) < m_mutation_rate)
            individual[i] = std::clamp(individual[i] + gauss(rng), lo[i], hi[i]);
    }
}

// --- Main optimisation loop ---

OptimizationResult GeneticAlgorithm::optimize(IProblem& problem) {
    const int dim = problem.getDimension();
    const auto lo = problem.getLowerBounds();
    const auto hi = problem.getUpperBounds();

    std::mt19937 rng(m_seed);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    // Initialise population randomly within bounds
    std::vector<std::vector<double>> pop(m_pop_size, std::vector<double>(dim));
    for (auto& ind : pop)
        for (int i = 0; i < dim; ++i) {
            std::uniform_real_distribution<double> d(lo[i], hi[i]);
            ind[i] = d(rng);
        }

    // Evaluate initial population
    std::vector<double> costs(m_pop_size);
    for (int i = 0; i < m_pop_size; ++i)
        costs[i] = problem.evaluate(pop[i]);

    // Track global best
    int best_idx = static_cast<int>(
        std::min_element(costs.begin(), costs.end()) - costs.begin());
    auto   best      = pop[best_idx];
    double best_cost = costs[best_idx];

    std::cout << "[GA] initial population best: " << best_cost << "\n";

    for (int gen = 0; gen < m_generations; ++gen) {
        // Sort indices by cost (ascending = better first)
        std::vector<int> ranked(m_pop_size);
        std::iota(ranked.begin(), ranked.end(), 0);
        std::sort(ranked.begin(), ranked.end(),
                  [&](int a, int b) { return costs[a] < costs[b]; });

        std::vector<std::vector<double>> new_pop(m_pop_size);
        std::vector<double>              new_costs(m_pop_size);

        // Elitism: best individuals survive unchanged
        for (int i = 0; i < m_elite_count; ++i) {
            new_pop[i]   = pop[ranked[i]];
            new_costs[i] = costs[ranked[i]];
        }

        // Fill remaining slots via selection, crossover, mutation
        for (int i = m_elite_count; i < m_pop_size; ++i) {
            int p1 = tournament_select(costs, rng);
            std::vector<double> child;

            if (uniform(rng) < m_crossover_rate) {
                int p2 = tournament_select(costs, rng);
                child = crossover(pop[p1], pop[p2], rng);
            } else {
                child = pop[p1];
            }

            mutate(child, lo, hi, rng);
            new_pop[i]   = child;
            new_costs[i] = problem.evaluate(child);

            if (new_costs[i] < best_cost) {
                best_cost = new_costs[i];
                best      = new_pop[i];
            }
        }

        pop   = std::move(new_pop);
        costs = std::move(new_costs);

        if (gen % 50 == 0)
            std::cout << "[GA] gen " << gen
                      << "  best: " << best_cost << "\n";
    }

    return {best, best_cost, m_generations, best_cost < 10.0};
}
