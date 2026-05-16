#pragma once

#include "../../interfaces/IOptimizer.h"

#include <random>
#include <vector>

// Genetic Algorithm optimiser (real-valued, steady-state with elitism).
//
// Each individual is a real-valued vector within the problem bounds.
// Each generation:
//   1. Elite individuals are carried over unchanged.
//   2. Remaining slots are filled by tournament selection + uniform crossover + Gaussian mutation.
//
// Uniform crossover: each gene is independently drawn from parent1 or parent2
// with equal probability. This is better than single-point for high-dimensional
// problems because it can combine good sub-sequences from both parents.
class GeneticAlgorithm : public IOptimizer {
public:
    GeneticAlgorithm(int    population_size = 100,
                     int    generations     = 200,
                     double crossover_rate  = 0.8,
                     double mutation_rate   = 0.1,
                     double mutation_scale  = 0.5,
                     int    tournament_size = 5,
                     int    elite_count     = 5,
                     unsigned int seed      = 42);

    OptimizationResult optimize(IProblem& problem) override;

private:
    int tournament_select(const std::vector<double>& costs, std::mt19937& rng) const;

    std::vector<double> crossover(const std::vector<double>& p1,
                                  const std::vector<double>& p2,
                                  std::mt19937& rng) const;

    void mutate(std::vector<double>& individual,
                const std::vector<double>& lo,
                const std::vector<double>& hi,
                std::mt19937& rng) const;

    int          m_pop_size;
    int          m_generations;
    double       m_crossover_rate;
    double       m_mutation_rate;
    double       m_mutation_scale;
    int          m_tournament_size;
    int          m_elite_count;
    unsigned int m_seed;
};
