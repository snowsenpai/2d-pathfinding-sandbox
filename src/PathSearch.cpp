#include "PathSearch.h"

#include <algorithm>
#include <limits>

namespace
{
	// Orthogonal first, then diagonal. The ORDER decides which of several equally cheap paths
	// you get, so keep it fixed. (screen space: up is -y)
	constexpr Cell Dirs8[] = {
		{0, -1}, {1, 0}, {0, 1}, {-1, 0},      // up, right, down, left
		{1, -1}, {1, 1}, {-1, 1}, {-1, -1}     // up-right, down-right, down-left, up-left
	};
	constexpr int OrthogonalCount = 4;
	constexpr int AllDirectionCount = 8;

	constexpr float Sqrt2 = 1.41421356237f;
	constexpr float Infinity = std::numeric_limits<float>::infinity();

	bool isDiagonal(const Cell& d) { return d.x != 0 && d.y != 0; }
}

void PathSearch::reset()
{
	m_status = SearchStatus::Idle;
	m_options = SearchOptions{};
	m_cols = 0;
	m_rows = 0;
	m_state.clear();
	m_parent.clear();
	m_cost.clear();
	m_queue.clear();
	m_heap.clear();
	m_seq = 0;
	m_path.clear();
	m_pathCost = 0.f;
	m_expanded = 0;
	m_frontierCount = 0;
	m_maxFrontier = 0;
}

bool PathSearch::start(const Grid& grid, Cell start, Cell goal, SearchOptions options)
{
	reset();

	// MAP QUERY: bounds + walkability of both endpoints.
	if (!grid.inBounds(start) || !grid.inBounds(goal)) return false;
	if (!grid.isWalkable(start) || !grid.isWalkable(goal)) return false;

	m_options = options;
	m_start = start;
	m_goal = goal;
	m_cols = grid.cols();   // MAP QUERY
	m_rows = grid.rows();   // MAP QUERY

	const std::size_t cellCount = static_cast<std::size_t>(m_cols) * static_cast<std::size_t>(m_rows);
	m_state.assign(cellCount, NodeState::Unseen);
	m_parent.assign(cellCount, -1);
	m_cost.assign(cellCount, Infinity);

	// Seed the frontier. A cell becomes Frontier the first time it is DISCOVERED.
	const int startIdx = toIndex(start);
	m_state[static_cast<std::size_t>(startIdx)] = NodeState::Frontier;
	m_cost[static_cast<std::size_t>(startIdx)] = 0.f;
	m_frontierCount = 1;
	m_maxFrontier = 1;

	if (m_options.algorithm == Algorithm::BFS) m_queue.push_back(startIdx);
	else                                       pushHeap(0.f, startIdx);

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

	if (m_options.algorithm == Algorithm::BFS) stepBfs(grid);
	else                                       stepDijkstra(grid);
}

SearchStatus PathSearch::runToEnd(const Grid& grid)
{
	while (m_status == SearchStatus::Running)
	{
		step(grid);
	}
	return m_status;
}

// --------------------------------------------------------------------------------------
// Shared helpers
// --------------------------------------------------------------------------------------

// Can we step from `from` in direction `dir`?  Target must be walkable. A diagonal must also not
// squeeze between two wall corners, unless corner cutting is allowed.
bool PathSearch::canStep(const Grid& grid, Cell from, const Cell& dir) const
{
	const Cell to{ from.x + dir.x, from.y + dir.y };

	if (!grid.isWalkable(to)) return false;                          // MAP QUERY (also bounds)

	if (isDiagonal(dir) && !m_options.allowCornerCutting)
	{
		// BOTH orthogonal neighbours of the diagonal must be open.
		if (!grid.isWalkable(Cell{ from.x + dir.x, from.y })) return false;   // MAP QUERY
		if (!grid.isWalkable(Cell{ from.x, from.y + dir.y })) return false;   // MAP QUERY
	}
	return true;
}

// Cost of ENTERING `to` by moving along `dir`: terrain cost, stretched by sqrt(2) on a diagonal.
float PathSearch::stepCost(const Grid& grid, Cell to, const Cell& dir) const
{
	return grid.moveCost(to) * (isDiagonal(dir) ? Sqrt2 : 1.f);       // MAP QUERY
}

void PathSearch::pushHeap(float cost, int idx)
{
	m_heap.push_back(HeapEntry{ cost, m_seq++, idx });
	std::push_heap(m_heap.begin(), m_heap.end(), HeapOrder{});
}

// --------------------------------------------------------------------------------------
// BFS: minimises the NUMBER OF STEPS. Costs are ignored while searching, but recorded so the
// cost of the path BFS found can be compared with Dijkstra's.
// --------------------------------------------------------------------------------------
void PathSearch::stepBfs(const Grid& grid)
{
	// 1. POP the oldest discovered cell (FIFO => cells come out in order of step count).
	const int curIdx = m_queue.front();
	m_queue.pop_front();
	m_state[static_cast<std::size_t>(curIdx)] = NodeState::Visited;
	--m_frontierCount;
	++m_expanded;

	const Cell cur = toCell(curIdx);

	// 2. GOAL TEST at pop.
	if (cur == m_goal)
	{
		buildPath(grid);
		m_status = SearchStatus::Found;
		return;
	}

	// 3. DISCOVER neighbours (4 or 8 directions).
	const int dirCount = m_options.diagonal ? AllDirectionCount : OrthogonalCount;
	for (int i = 0; i < dirCount; ++i)
	{
		const Cell& d = Dirs8[i];
		if (!canStep(grid, cur, d)) continue;

		const Cell n{ cur.x + d.x, cur.y + d.y };
		const std::size_t nIdx = static_cast<std::size_t>(toIndex(n));
		if (m_state[nIdx] != NodeState::Unseen) continue;       // already queued or expanded

		m_state[nIdx] = NodeState::Frontier;
		m_parent[nIdx] = curIdx;
		m_cost[nIdx] = m_cost[static_cast<std::size_t>(curIdx)] + stepCost(grid, n, d);
		m_queue.push_back(static_cast<int>(nIdx));
		++m_frontierCount;
	}

	m_maxFrontier = std::max(m_maxFrontier, m_frontierCount);

	// 4. Nothing left to explore and the goal was never popped: it is unreachable.
	if (m_frontierCount == 0)
	{
		m_status = SearchStatus::NoPath;
	}
}

// --------------------------------------------------------------------------------------
// Dijkstra: minimises TOTAL COST. Always expands the cheapest known unexpanded cell.
// A cell may be improved (relaxed) several times while it is Frontier; once it is popped
// (Visited) its cost is final, provided every step cost is positive.
// --------------------------------------------------------------------------------------
void PathSearch::stepDijkstra(const Grid& grid)
{
	// 1. POP the cheapest entry, silently discarding STALE ones (a better route to that cell was
	//    found after this entry was pushed, or the cell was already expanded). Only a real
	//    expansion counts as one step() / one `expanded`.
	int curIdx = -1;
	while (!m_heap.empty())
	{
		std::pop_heap(m_heap.begin(), m_heap.end(), HeapOrder{});
		const HeapEntry e = m_heap.back();
		m_heap.pop_back();

		const std::size_t eIdx = static_cast<std::size_t>(e.idx);
		if (m_state[eIdx] == NodeState::Visited) continue;     // already expanded
		if (e.cost > m_cost[eIdx]) continue;                   // stale: a cheaper entry exists

		curIdx = e.idx;
		break;
	}

	if (curIdx < 0)
	{
		// Heap exhausted without reaching the goal.
		m_status = SearchStatus::NoPath;
		return;
	}

	m_state[static_cast<std::size_t>(curIdx)] = NodeState::Visited;
	--m_frontierCount;
	++m_expanded;

	const Cell cur = toCell(curIdx);

	// 2. GOAL TEST at pop (NOT at push): only now is the goal's cost guaranteed to be the cheapest.
	if (cur == m_goal)
	{
		buildPath(grid);
		m_status = SearchStatus::Found;
		return;
	}

	// 3. RELAX neighbours.
	const float curCost = m_cost[static_cast<std::size_t>(curIdx)];
	const int dirCount = m_options.diagonal ? AllDirectionCount : OrthogonalCount;

	for (int i = 0; i < dirCount; ++i)
	{
		const Cell& d = Dirs8[i];
		if (!canStep(grid, cur, d)) continue;

		const Cell n{ cur.x + d.x, cur.y + d.y };
		const std::size_t nIdx = static_cast<std::size_t>(toIndex(n));
		if (m_state[nIdx] == NodeState::Visited) continue;     // final already

		const float newCost = curCost + stepCost(grid, n, d);
		if (newCost < m_cost[nIdx])                            // found a cheaper way in
		{
			if (m_state[nIdx] == NodeState::Unseen)
			{
				m_state[nIdx] = NodeState::Frontier;
				++m_frontierCount;
			}
			m_cost[nIdx] = newCost;
			m_parent[nIdx] = curIdx;
			pushHeap(newCost, static_cast<int>(nIdx));          // old entry becomes stale
		}
	}

	m_maxFrontier = std::max(m_maxFrontier, m_frontierCount);

	// 4. No unexpanded cells left (any remaining heap entries are stale): unreachable.
	if (m_frontierCount == 0)
	{
		m_status = SearchStatus::NoPath;
	}
}

// --------------------------------------------------------------------------------------
// Results
// --------------------------------------------------------------------------------------
NodeState PathSearch::stateAt(Cell c) const
{
	if (m_state.empty() || c.x < 0 || c.y < 0 || c.x >= m_cols || c.y >= m_rows)
	{
		return NodeState::Unseen;
	}
	return m_state[static_cast<std::size_t>(toIndex(c))];
}

float PathSearch::costAt(Cell c) const
{
	if (m_cost.empty() || c.x < 0 || c.y < 0 || c.x >= m_cols || c.y >= m_rows)
	{
		return Infinity;
	}
	return m_cost[static_cast<std::size_t>(toIndex(c))];
}

void PathSearch::buildPath(const Grid& grid)
{
	m_path.clear();

	// Walk parent links goal -> start (the start has parent -1), then flip to start -> goal.
	for (int i = toIndex(m_goal); i != -1; i = m_parent[static_cast<std::size_t>(i)])
	{
		m_path.push_back(toCell(i));
	}
	std::reverse(m_path.begin(), m_path.end());

	// True cost of the path, summed in travel order (the same order Dijkstra accumulated it,
	// so for Dijkstra this matches costAt(goal) exactly).
	m_pathCost = 0.f;
	for (std::size_t i = 1; i < m_path.size(); ++i)
	{
		const Cell dir{ m_path[i].x - m_path[i - 1].x, m_path[i].y - m_path[i - 1].y };
		m_pathCost += stepCost(grid, m_path[i], dir);
	}
}