#pragma once

#include "../../interfaces/IOptimizer.h"

// Random Search — baseline optimizer.
//
// Samples candidate solutions uniformly at random within the problem bounds
// and keeps the best one found. Gradient-free and assumption-free.
//
// Purpose: establish the IOptimizer interface and provide a performance
// baseline that all future algorithms (evolutionary, PSO, etc.) must beat.
class RandomSearch : public IOptimizer {
public:
    explicit RandomSearch(int max_iterations = 1000, unsigned int seed = 42);

    OptimizationResult optimize(IProblem& problem) override;

private:
    int          m_max_iterations;
    unsigned int m_seed;
};
