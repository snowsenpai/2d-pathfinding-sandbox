#include "PathSearch.h"

#include <algorithm>

namespace
{
	// 4-connected neighbours: up, right, down, left (screen space: up is -y).
	// The ORDER decides which of several equally short paths you get, so keep it fixed.
	constexpr Cell Dirs4[] = { {0, -1}, {1, 0}, {0, 1}, {-1, 0} };
}

void PathSearch::reset()
{
	m_status = SearchStatus::Idle;
	m_cols = 0;
	m_rows = 0;
	m_state.clear();
	m_parent.clear();
	m_queue.clear();
	m_path.clear();
	m_expanded = 0;
	m_maxFrontier = 0;
}

bool PathSearch::start(const Grid& grid, Cell start, Cell goal)
{
	reset();

	// MAP QUERY: bounds + wall checks on both endpoints.
	if (!grid.inBounds(start) || !grid.inBounds(goal)) return false;
	if (grid.get(start) == Tile::Wall || grid.get(goal) == Tile::Wall) return false;

	m_start = start;
	m_goal = goal;
	m_cols = grid.cols();   // MAP QUERY
	m_rows = grid.rows();   // MAP QUERY

	const std::size_t cellCount = static_cast<std::size_t>(m_cols) * static_cast<std::size_t>(m_rows);
	m_state.assign(cellCount, NodeState::Unseen);
	m_parent.assign(cellCount, -1);

	// Seed. Mark Frontier at ENQUEUE time (not at pop time), so no cell is ever queued twice.
	const int startIdx = toIndex(start);
	m_state[static_cast<std::size_t>(startIdx)] = NodeState::Frontier;
	m_queue.push_back(startIdx);
	m_maxFrontier = 1;

	m_status = SearchStatus::Running;
	return true;
}

void PathSearch::step(const Grid& grid)
{
	if (m_status != SearchStatus::Running) return;

	// Defensive: if the map was swapped for a different size, our arrays are meaningless.
	if (grid.cols() != m_cols || grid.rows() != m_rows)
	{
		reset();
		return;
	}

	// 1. POP the oldest discovered cell (FIFO => cells come out in order of distance from start).
	const int curIdx = m_queue.front();
	m_queue.pop_front();
	m_state[static_cast<std::size_t>(curIdx)] = NodeState::Visited;
	++m_expanded;

	const Cell cur = toCell(curIdx);

	// 2. GOAL TEST at pop (not at push). Same structure Dijkstra and A* will need.
	if (cur == m_goal)
	{
		buildPath();
		m_status = SearchStatus::Found;
		return;
	}

	// 3. DISCOVER neighbours.
	for (const Cell& d : Dirs4)
	{
		const Cell n{ cur.x + d.x, cur.y + d.y };

		if (!grid.inBounds(n)) continue;                // MAP QUERY
		if (grid.get(n) == Tile::Wall) continue;        // MAP QUERY

		const std::size_t nIdx = static_cast<std::size_t>(toIndex(n));
		if (m_state[nIdx] != NodeState::Unseen) continue; // already queued or expanded

		m_state[nIdx] = NodeState::Frontier;
		m_parent[nIdx] = curIdx;                        // remember who found us (for the path)
		m_queue.push_back(static_cast<int>(nIdx));
	}

	m_maxFrontier = std::max(m_maxFrontier, static_cast<int>(m_queue.size()));

	// 4. Nothing left to explore and the goal was never popped: it is unreachable.
	if (m_queue.empty())
	{
		m_status = SearchStatus::NoPath;
	}
}

SearchStatus PathSearch::runToEnd(const Grid& grid)
{
	while (m_status == SearchStatus::Running)
	{
		step(grid);
	}
	return m_status;
}

NodeState PathSearch::stateAt(Cell c) const
{
	if (m_state.empty() || c.x < 0 || c.y < 0 || c.x >= m_cols || c.y >= m_rows)
	{
		return NodeState::Unseen;
	}
	return m_state[static_cast<std::size_t>(toIndex(c))];
}

void PathSearch::buildPath()
{
	m_path.clear();

	// Walk parent links goal -> start (the start has parent -1), then flip to start -> goal.
	for (int i = toIndex(m_goal); i != -1; i = m_parent[static_cast<std::size_t>(i)])
	{
		m_path.push_back(toCell(i));
	}
	std::reverse(m_path.begin(), m_path.end());
}