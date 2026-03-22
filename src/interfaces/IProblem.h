#pragma once

#include <vector>

// Abstract interface for an optimization problem.
// Defines the search space and how to evaluate a candidate solution.
// Optimizers depend only on this interface — they have no knowledge of
// what the problem is (IK, trajectory planning, etc.).
class IProblem {
public:
    virtual ~IProblem() = default;

    // Evaluate the cost of a candidate solution. Lower is better.
    virtual double evaluate(const std::vector<double>& params) = 0;

    // Return true if the candidate satisfies all constraints.
    virtual bool isFeasible(const std::vector<double>& params) const = 0;

    // Number of decision variables (e.g. number of joints for IK).
    virtual int getDimension() const = 0;

    // Per-variable lower bounds.
    virtual std::vector<double> getLowerBounds() const = 0;

    // Per-variable upper bounds.
    virtual std::vector<double> getUpperBounds() const = 0;
};
