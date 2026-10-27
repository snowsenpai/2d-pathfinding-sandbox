#pragma once

#include "Grid.h"   // Cell

#include <algorithm>
#include <cmath>

// Heuristics for A*: pure functions, no SFML, no Grid queries.
//
// h(a, b) estimates the cost of travelling from cell a to cell b, measured in CELLS on an
// open field (diagonal step = sqrt(2), orthogonal step = 1). The search multiplies it by the
// cheapest possible terrain cost so the estimate stays in the same units as the real costs.
//
// ADMISSIBLE  = never overestimates the true remaining cost  =>  A* returns an optimal path.
// CONSISTENT  = h(a) <= stepCost(a,b) + h(b)                  =>  a cell is final when first popped.
//
//   heuristic   4-way movement     8-way movement (diagonal = sqrt2)
//   ---------   ----------------   ---------------------------------
//   Zero        admissible         admissible   (= Dijkstra, expands everything)
//   Manhattan   admissible, exact  NOT admissible (counts a diagonal as 2)  -> can return a worse path
//   Euclidean   admissible         admissible   (a straight line is never longer than the grid path)
//   Chebyshev   admissible         admissible   (counts a diagonal as 1: too optimistic, so weak)
//   Octile      admissible         admissible, EXACT on an open field (the best choice for 8-way)
enum class Heuristic { Zero, Manhattan, Euclidean, Chebyshev, Octile };

inline const char* toString(Heuristic h)
{
	switch (h)
	{
	case Heuristic::Zero:      return "Zero";
	case Heuristic::Manhattan: return "Manhattan";
	case Heuristic::Euclidean: return "Euclidean";
	case Heuristic::Chebyshev: return "Chebyshev";
	case Heuristic::Octile:    return "Octile";
	}
	return "?";
}

// Estimated remaining cost from `a` to `b`, in cells (cost units of an open-field move).
inline float heuristic(Heuristic kind, Cell a, Cell b)
{
	constexpr float Sqrt2 = 1.41421356237f;

	const float dx = static_cast<float>(std::abs(a.x - b.x));
	const float dy = static_cast<float>(std::abs(a.y - b.y));

	switch (kind)
	{
	case Heuristic::Zero:
		return 0.f;
	case Heuristic::Manhattan:
		return dx + dy;
	case Heuristic::Euclidean:
		return std::sqrt(dx * dx + dy * dy);
	case Heuristic::Chebyshev:
		return std::max(dx, dy);
	case Heuristic::Octile:
		// Move diagonally for min(dx,dy) steps (each costs sqrt2 instead of the 2 that going
		// orthogonally would cost), then straight for the remainder.
		return (dx + dy) + (Sqrt2 - 2.f) * std::min(dx, dy);
	}
	return 0.f;
}
