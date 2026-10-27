#pragma once

#include "Grid.h"
#include "Heuristic.h"

#include <cstdint>
#include <deque>
#include <vector>

// Resumable grid search: BFS, Dijkstra and A*.  No SFML, no ImGui, no entities.
//
// REUSING THIS IN A REAL GAME
//   The search only asks the map these questions (all in PathSearch.cpp, marked "MAP QUERY"):
//     grid.cols()/rows()     -> how big is the map
//     grid.inBounds(cell)    -> is this cell inside the map
//     grid.isWalkable(cell)  -> can I stand here at all (not a wall)
//     grid.moveCost(cell)    -> what does it cost to ENTER this cell (>= a small positive minimum)
//   Swap `Grid` for your game's tile map by providing those four calls.
//
// USAGE
//   PathSearch s;
//   SearchOptions opt;  opt.algorithm = Algorithm::Dijkstra;  opt.diagonal = true;
//   s.start(grid, startCell, goalCell, opt);     // seeds the frontier; status = Running
//   while (s.status() == SearchStatus::Running)  // or call s.step(grid) once per UI click
//       s.step(grid);                            // ONE real expansion per call
//   if (s.status() == SearchStatus::Found) use(s.path(), s.pathCost());
//   // one-liner for gameplay code: s.runToEnd(grid);
//
// COSTS
//   Entering a cell costs  grid.moveCost(cell) * (diagonal step ? sqrt(2) : 1).
//   BFS ignores costs while searching (it minimises STEPS) but still reports the true cost of the
//   path it found, so BFS, Dijkstra and A* results can be compared fairly.
//
// A* IS DIJKSTRA WITH A DIFFERENT PRIORITY
//   Dijkstra pops the smallest g (cost so far). A* pops the smallest f = g + w * scale * h(cell, goal).
//   Dijkstra is literally A* with the Zero heuristic, and shares one code path (stepBestFirst).
//
// If the Grid (walls, terrain, costs) changes while a search is Running, the answer is stale: call reset().

enum class SearchStatus { Idle, Running, Found, NoPath };

// Unseen   : never discovered
// Frontier : discovered, waiting to be expanded (its cost may still improve under Dijkstra)
// Visited  : expanded; its cost is final
enum class NodeState : std::uint8_t { Unseen, Frontier, Visited };

enum class Algorithm { BFS, Dijkstra, AStar };

struct SearchOptions
{
	Algorithm algorithm = Algorithm::BFS;   // defaults reproduce Checkpoint 2 behaviour exactly
	bool diagonal = false;                  // false: 4-way, true: 8-way
	bool allowCornerCutting = false;        // false: a diagonal needs BOTH orthogonal neighbours walkable

	// --- A* only (Dijkstra always uses the Zero heuristic) ---
	Heuristic heuristic = Heuristic::Octile;
	float weight = 1.0f;                    // 1 = optimal A*. >1 = weighted A*: faster, cost up to weight x optimal.
	bool tieBreakLargerG = true;            // among equal f, expand the cell with the larger g (closer to the goal)
};

inline const char* toString(SearchStatus s)
{
	switch (s)
	{
	case SearchStatus::Idle:    return "Idle";
	case SearchStatus::Running: return "Running";
	case SearchStatus::Found:   return "Found";
	case SearchStatus::NoPath:  return "No path";
	}
	return "?";
}

inline const char* toString(Algorithm a)
{
	switch (a)
	{
	case Algorithm::BFS:      return "BFS";
	case Algorithm::Dijkstra: return "Dijkstra";
	case Algorithm::AStar:    return "A*";
	}
	return "?";
}

class PathSearch
{
	// One entry of the best-first frontier (Dijkstra / A*).
	//   f = g + estimate: the priority.   g: the cost-so-far this entry was pushed with.
	// An entry is STALE when a cheaper route to its cell was found later (g > m_cost[idx]).
	// Stale entries are skipped lazily when they reach the top, because a binary heap cannot
	// decrease a key. The stale test compares g, NOT f: f also contains the heuristic.
	struct HeapEntry
	{
		float f;
		float g;
		std::uint32_t seq;   // insertion order: makes ties deterministic
		int idx;
	};

	// "Lower priority" for std::push_heap/pop_heap, which keep the LARGEST element on top.
	// We want the SMALLEST f on top, so a is lower priority than b when a.f is larger.
	// This must be a strict weak ordering or the heap is corrupted (see the tie-break rules).
	struct HeapOrder
	{
		bool preferLargerG = false;

		bool operator()(const HeapEntry& a, const HeapEntry& b) const
		{
			if (a.f != b.f) return a.f > b.f;                          // 1. smaller f first
			if (preferLargerG && a.g != b.g) return a.g < b.g;         // 2. then larger g (nearer the goal)
			return a.seq > b.seq;                                      // 3. then older entry (deterministic)
		}
	};

	SearchStatus  m_status = SearchStatus::Idle;
	SearchOptions m_options;
	Cell m_start;
	Cell m_goal;
	int  m_cols = 0;
	int  m_rows = 0;

	std::vector<NodeState> m_state;   // cols*rows, row-major: index = y * cols + x
	std::vector<int>       m_parent;  // index of the cell that discovered/improved this one, -1 = none
	std::vector<float>     m_cost;    // best known cost from start; +infinity = unseen
	std::deque<int>        m_queue;   // BFS frontier (FIFO)
	std::vector<HeapEntry> m_heap;    // Dijkstra frontier (binary min-heap via std::push_heap/pop_heap)
	std::uint32_t          m_seq = 0; // next HeapEntry::seq

	// Resolved in start(): what priority function this search uses.
	Heuristic m_heuristic = Heuristic::Zero;  // Dijkstra and BFS: Zero
	float m_hScale = 1.f;                     // grid.minMoveCost(): keeps h in the same units as real costs
	float m_weight = 1.f;
	std::vector<Cell>      m_path;    // start -> goal, filled when Found
	float                  m_pathCost = 0.f;

	int m_expanded = 0;               // cells expanded so far (stale heap pops do NOT count)
	int m_frontierCount = 0;          // cells currently in state Frontier
	int m_maxFrontier = 0;            // largest frontier seen (memory-use indicator)

	int  toIndex(Cell c) const { return c.y * m_cols + c.x; }
	Cell toCell(int i) const { return Cell{ i % m_cols, i / m_cols }; }

	bool canStep(const Grid& grid, Cell from, const Cell& dir) const; // walkable target + corner rule
	float stepCost(const Grid& grid, Cell to, const Cell& dir) const; // cost of making this step

	float estimate(Cell c) const;          // weight * scale * h(c, goal): the "remaining cost" guess
	HeapOrder heapOrder() const { return HeapOrder{ m_options.tieBreakLargerG }; }
	void pushHeap(float g, int idx);       // pushes { g + estimate(cell), g, seq, idx }
	void stepBfs(const Grid& grid);
	void stepBestFirst(const Grid& grid);  // Dijkstra and A*
	void buildPath(const Grid& grid);

public:
	// Back to Idle; drops all overlays and results.
	void reset();

	// Seeds a new search. Returns false (and stays Idle) if start or goal is out of bounds or not walkable.
	// start == goal is legal: the first step() pops it and reports Found with a one-cell path.
	bool start(const Grid& grid, Cell start, Cell goal, SearchOptions options = {});

	// Exactly ONE real expansion. No-op unless Running.
	void step(const Grid& grid);

	// Convenience for gameplay code and tests: step until finished. Returns the final status.
	SearchStatus runToEnd(const Grid& grid);

	SearchStatus status() const { return m_status; }
	const SearchOptions& options() const { return m_options; }
	NodeState stateAt(Cell c) const;                  // Unseen for out-of-range cells or before start()
	float costAt(Cell c) const;                       // best known cost; +infinity if unseen / out of range
	const std::vector<Cell>& path() const { return m_path; }
	float pathCost() const { return m_pathCost; }     // true cost of path(); 0 unless Found
	int expanded() const { return m_expanded; }
	int frontierSize() const { return m_frontierCount; }
	int maxFrontier() const { return m_maxFrontier; }
};
