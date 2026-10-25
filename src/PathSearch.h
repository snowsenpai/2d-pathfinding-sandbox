#pragma once

#include "Grid.h"

#include <cstdint>
#include <deque>
#include <vector>

// Resumable breadth-first search on a Grid.  No SFML, no ImGui, no entities.
//
// REUSING THIS IN A REAL GAME
//   The search only asks the map three questions (all in PathSearch.cpp, marked "MAP QUERY"):
//     grid.cols()/rows()   -> how big is the map
//     grid.inBounds(cell)  -> is this cell inside the map
//     grid.get(cell)       -> is it a Wall
//   Swap `Grid` for your game's tile map by changing only those calls.
//
// USAGE
//   PathSearch s;
//   s.start(grid, startCell, goalCell);          // seeds the queue; status = Running
//   while (s.status() == SearchStatus::Running)  // or call s.step(grid) once per UI click
//       s.step(grid);                            // ONE expansion per call
//   if (s.status() == SearchStatus::Found) use(s.path());   // start -> goal, inclusive
//   // one-liner for gameplay code: s.runToEnd(grid);
//
// The search stores its own per-cell arrays, so it does not touch the Grid.
// If the Grid's walls change while a search is Running, the answer is stale: call reset().

enum class SearchStatus { Idle, Running, Found, NoPath };

// Unseen   : never queued
// Frontier : discovered and queued, not yet expanded
// Visited  : expanded (popped from the queue)
enum class NodeState : std::uint8_t { Unseen, Frontier, Visited };

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

class PathSearch
{
	SearchStatus m_status = SearchStatus::Idle;
	Cell m_start;
	Cell m_goal;
	int  m_cols = 0;
	int  m_rows = 0;

	std::vector<NodeState> m_state;   // cols*rows, row-major: index = y * cols + x
	std::vector<int>       m_parent;  // index of the cell that discovered this one, -1 = none
	std::deque<int>        m_queue;   // FIFO of cell indices (push_back / pop_front)
	std::vector<Cell>      m_path;    // start -> goal, filled when Found

	int m_expanded = 0;               // cells popped so far
	int m_maxFrontier = 0;            // largest queue size seen (memory-use indicator)

	int  toIndex(Cell c) const { return c.y * m_cols + c.x; }
	Cell toCell(int i) const { return Cell{ i % m_cols, i / m_cols }; }
	void buildPath();

public:
	// Back to Idle; drops all overlays and results.
	void reset();

	// Seeds a new search. Returns false (and stays Idle) if start or goal is out of bounds or a wall.
	// start == goal is legal: the first step() pops it and reports Found with a one-cell path.
	bool start(const Grid& grid, Cell start, Cell goal);

	// Exactly ONE expansion. No-op unless Running.
	void step(const Grid& grid);

	// Convenience for gameplay code and tests: step until finished. Returns the final status.
	SearchStatus runToEnd(const Grid& grid);

	SearchStatus status() const { return m_status; }
	NodeState stateAt(Cell c) const;                  // Unseen for out-of-range cells or before start()
	const std::vector<Cell>& path() const { return m_path; }
	int expanded() const { return m_expanded; }
	int frontierSize() const { return static_cast<int>(m_queue.size()); }
	int maxFrontier() const { return m_maxFrontier; }
};