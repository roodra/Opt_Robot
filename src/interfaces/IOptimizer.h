#pragma once

#include <vector>
#include "IProblem.h"

// Result returned by any optimizer.
struct OptimizationResult {
    std::vector<double> solution;   // Best parameter vector found
    double cost;                    // Cost at the best solution
    int iterations;                 // Number of iterations run
    bool converged;                 // Whether a stopping criterion was met
};

// Abstract interface for an optimization algorithm.
// Concrete implementations (evolutionary, backtracking, PSO, etc.) derive from this.
// Algorithms depend only on IProblem — they have no knowledge of robot physics.
class IOptimizer {
public:
    virtual ~IOptimizer() = default;

    // Run the optimizer on the given problem and return the best solution found.
    virtual OptimizationResult optimize(IProblem& problem) = 0;
};
