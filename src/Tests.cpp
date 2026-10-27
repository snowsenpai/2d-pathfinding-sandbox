// Checkpoint 1 + 2 + 3 + 4 (A*) + path-following tests. No SFML / ImGui needed: links Vec2.cpp, Grid.cpp, PathSearch.cpp.
// Build target: sandbox_tests.  Exit code 0 = all passed.

#include "FixedStep.h"
#include "Heuristic.h"
#include "Grid.h"
#include "Movement.h"
#include "PathFollow.h"
#include "PathSearch.h"
#include "Vec2.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

static int g_failures = 0;

#define CHECK(cond)                                                     \
	do {                                                                \
		if (!(cond)) {                                                  \
			std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
			++g_failures;                                               \
		}                                                               \
	} while (0)

static bool near(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

// Runs the exact Game-loop recipe (stepper -> N ticks of integratePosition)
// for `seconds` of real time at a given render FPS. Returns final position.
static Vec2 simulate(float speedPxPerSec, float fps, float seconds)
{
	FixedStepper stepper;
	Vec2 pos{ 0.f, 0.f };
	const Vec2 vel{ speedPxPerSec, 0.f };

	const int frames = static_cast<int>(std::lround(fps * seconds));
	for (int f = 0; f < frames; ++f)
	{
		const int ticks = stepper.advance(1.0f / fps);
		for (int t = 0; t < ticks; ++t)
		{
			pos = movement::integratePosition(pos, vel, stepper.fixedDt);
		}
	}
	return pos;
}


// ---- Checkpoint 2 helpers ----

// start->goal list is valid: right endpoints, 4-connected steps, no wall cells
static bool pathIsValid(const Grid& g, const std::vector<Cell>& path, Cell s, Cell goal)
{
	if (path.empty()) return false;
	if (path.front() != s || path.back() != goal) return false;
	for (std::size_t i = 0; i < path.size(); ++i)
	{
		if (!g.inBounds(path[i]) || g.get(path[i]) == Tile::Wall) return false;
		if (i > 0)
		{
			const int dx = std::abs(path[i].x - path[i - 1].x);
			const int dy = std::abs(path[i].y - path[i - 1].y);
			if (dx + dy != 1) return false;   // exactly one orthogonal step
		}
	}
	return true;
}

static int countOpen(const Grid& g)
{
	int n = 0;
	for (int y = 0; y < g.rows(); ++y)
		for (int x = 0; x < g.cols(); ++x)
			if (g.get(Cell{ x, y }) == Tile::Open) ++n;
	return n;
}

// Walks a route at `speed` px/s using fixed steps of `dt`, exactly like Game::sPathFollow does.
// Returns the number of ticks until arrival (or -1 if it never arrives within maxTicks).
static int ticksToArrive(const std::vector<Vec2>& wps, float speed, float dt, Vec2* finalPos = nullptr)
{
	Vec2 pos = wps.front();
	std::size_t next = 1;
	for (int tick = 1; tick <= 100000; ++tick)
	{
		const pathfollow::StepResult r = pathfollow::advance(pos, wps, next, speed * dt);
		pos = r.pos;
		next = r.next;
		if (r.arrived)
		{
			if (finalPos) *finalPos = pos;
			return tick;
		}
	}
	return -1;
}

// ---- Checkpoint 3 helpers ----

static SearchOptions options(Algorithm a, bool diagonal, bool cornerCutting = false)
{
	SearchOptions o;
	o.algorithm = a;
	o.diagonal = diagonal;
	o.allowCornerCutting = cornerCutting;
	return o;
}

// Re-computes a path's cost from the grid ALONE (independent of PathSearch internals).
static float recomputeCost(const Grid& g, const std::vector<Cell>& path)
{
	float total = 0.f;
	for (std::size_t i = 1; i < path.size(); ++i)
	{
		const bool diag = path[i].x != path[i - 1].x && path[i].y != path[i - 1].y;
		total += g.moveCost(path[i]) * (diag ? 1.41421356237f : 1.f);
	}
	return total;
}

// 8-way-aware validity: right endpoints, one king-move per step, walkable cells,
// and (unless corner cutting is allowed) no diagonal between two wall corners.
static bool pathIsValid8(const Grid& g, const std::vector<Cell>& path, Cell s, Cell goal, bool diagonal, bool cut)
{
	if (path.empty() || path.front() != s || path.back() != goal) return false;
	for (std::size_t i = 0; i < path.size(); ++i)
	{
		if (!g.isWalkable(path[i])) return false;
		if (i == 0) continue;
		const int dx = path[i].x - path[i - 1].x;
		const int dy = path[i].y - path[i - 1].y;
		if (dx == 0 && dy == 0) return false;
		if (std::abs(dx) > 1 || std::abs(dy) > 1) return false;
		const bool diag = dx != 0 && dy != 0;
		if (diag && !diagonal) return false;
		if (diag && !cut)
		{
			if (!g.isWalkable(Cell{ path[i - 1].x + dx, path[i - 1].y })) return false;
			if (!g.isWalkable(Cell{ path[i - 1].x, path[i - 1].y + dy })) return false;
		}
	}
	return true;
}

static int countWalkable(const Grid& g)
{
	int n = 0;
	for (int y = 0; y < g.rows(); ++y)
		for (int x = 0; x < g.cols(); ++x)
			if (g.isWalkable(Cell{ x, y })) ++n;
	return n;
}

static int countMud(const Grid& g, const std::vector<Cell>& path)
{
	int n = 0;
	for (const Cell& c : path) if (g.get(c) == Tile::Mud) ++n;
	return n;
}

// 7x3 map, mud strip on (2,1),(3,1),(4,1): the cheap-looking straight line vs a detour.
static Grid mudMap(float mudCost)
{
	Grid g(7, 3, 20.f);
	for (int x = 2; x <= 4; ++x) g.set(Cell{ x, 1 }, Tile::Mud);
	g.costs.mud = mudCost;
	return g;
}

// Tiny deterministic PRNG so the property test is reproducible without <random> engine differences.
static std::uint32_t nextRand(std::uint32_t& s)
{
	s ^= s << 13; s ^= s >> 17; s ^= s << 5;
	return s;
}

// ---- Checkpoint 4 helpers ----

static SearchOptions astar(Heuristic h, bool diagonal, bool cornerCutting = false,
                           float weight = 1.f, bool tieBreakLargerG = true)
{
	SearchOptions o;
	o.algorithm = Algorithm::AStar;
	o.heuristic = h;
	o.diagonal = diagonal;
	o.allowCornerCutting = cornerCutting;
	o.weight = weight;
	o.tieBreakLargerG = tieBreakLargerG;
	return o;
}

// '.' open   '#' wall   '~' mud   'S' start   'G' goal      (rows[y][x])
struct AsciiMap
{
	Grid grid;
	Cell start;
	Cell goal;
};

static AsciiMap parseMap(const std::vector<std::string>& rows)
{
	const int h = static_cast<int>(rows.size());
	const int w = static_cast<int>(rows[0].size());

	AsciiMap m{ Grid(w, h, 20.f), Cell{ -1, -1 }, Cell{ -1, -1 } };
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			switch (rows[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)])
			{
			case '#': m.grid.set(Cell{ x, y }, Tile::Wall); break;
			case '~': m.grid.set(Cell{ x, y }, Tile::Mud);  break;
			case 'S': m.start = Cell{ x, y };               break;
			case 'G': m.goal = Cell{ x, y };                break;
			default: break;
			}
		}
	}
	return m;
}

struct RunResult
{
	SearchStatus status;
	float cost;
	int cells;
	int expanded;
	std::vector<Cell> path;
};

static RunResult run(const Grid& g, Cell s, Cell goal, const SearchOptions& o)
{
	PathSearch ps;
	ps.start(g, s, goal, o);
	ps.runToEnd(g);
	return RunResult{ ps.status(), ps.pathCost(), static_cast<int>(ps.path().size()), ps.expanded(), ps.path() };
}

int main()
{
	// ---- Vec2 ----
	{
		const Vec2 z{ 0.f, 0.f };
		const Vec2 n = z.normalized();
		CHECK(n.x == 0.f && n.y == 0.f);             // zero in, zero out, no NaN
		CHECK(!std::isnan(n.x) && !std::isnan(n.y));

		CHECK(near(Vec2(3.f, 4.f).length(), 5.f, 1e-5f));
		CHECK(near(Vec2(3.f, 4.f).normalized().length(), 1.f, 1e-5f));

		const Vec2 lim = Vec2(30.f, 40.f).limited(10.f);  // length 50 -> 10
		CHECK(near(lim.length(), 10.f, 1e-4f));
		CHECK(near(lim.x, 6.f, 1e-4f) && near(lim.y, 8.f, 1e-4f)); // direction preserved

		const Vec2 small = Vec2(1.f, 1.f).limited(10.f);   // never scales up
		CHECK(small.x == 1.f && small.y == 1.f);

		CHECK(near(Vec2(1.f, 0.f).dot(Vec2(0.f, 1.f)), 0.f, 1e-6f));
	}

	// ---- zero velocity => no movement ----
	{
		const Vec2 p = movement::integratePosition(Vec2(10.f, 20.f), Vec2(0.f, 0.f), 1.0f / 60.0f);
		CHECK(p.x == 10.f && p.y == 20.f);
	}

	// ---- one second of movement, different render rates (expected: ~240 px) ----
	{
		const Vec2 at60  = simulate(240.f, 60.f,  1.f);
		const Vec2 at30  = simulate(240.f, 30.f,  1.f);
		const Vec2 at144 = simulate(240.f, 144.f, 1.f);
		std::printf("1s @240px/s -> 60fps: %.3f  30fps: %.3f  144fps: %.3f\n", at60.x, at30.x, at144.x);

		CHECK(near(at60.x, 240.f, 0.05f));
		CHECK(near(at30.x, 240.f, 0.05f));
		CHECK(near(at144.x, 240.f, 4.1f)); // within one tick (4 px): float accumulation may drop/add a tick
		CHECK(at60.y == 0.f);
	}

	// ---- diagonal input is normalized (speed stays 240, not ~339) ----
	{
		const Vec2 d = movement::directionFromFlags(true, false, false, true) * 240.f;
		CHECK(near(d.length(), 240.f, 1e-3f));
		CHECK(d.y < 0.f && d.x > 0.f);              // up is -y, right is +x
		const Vec2 cancel = movement::directionFromFlags(true, true, false, false);
		CHECK(cancel.x == 0.f && cancel.y == 0.f);  // opposite keys cancel
	}

	// ---- stepper: one tick per 1/60 s frame, two per 1/30 s frame ----
	{
		FixedStepper s;
		CHECK(s.advance(s.fixedDt) == 1);
		FixedStepper s2;
		CHECK(s2.advance(2.f * s2.fixedDt) == 2);
		FixedStepper s3;
		CHECK(s3.advance(0.5f * s3.fixedDt) == 0);  // too short: no tick yet...
		CHECK(s3.advance(0.5f * s3.fixedDt) == 1);  // ...but time is remembered
	}

	// ---- stepper: giant spike is clamped, capped and the backlog dropped ----
	{
		FixedStepper s;
		CHECK(s.advance(5.0f) == s.maxTicksPerFrame);  // 5 s spike -> at most 5 ticks
		CHECK(s.advance(0.0f) == 0);                   // backlog was dropped, no catch-up
		CHECK(s.advance(-1.0f) == 0);                  // negative frame time is ignored
	}

	// ---- grid coordinate conversion (800x600 window, 20 px cells => 40x30) ----
	{
		Grid g(40, 30, 20.f);
		const Cell origin = g.worldToCell(Vec2(0.f, 0.f));
		CHECK(origin.x == 0 && origin.y == 0);

		const Cell last = g.worldToCell(Vec2(799.f, 599.f));
		CHECK(last.x == 39 && last.y == 29 && g.inBounds(last));

		const Cell past = g.worldToCell(Vec2(800.f, 600.f));
		CHECK(past.x == 40 && past.y == 30 && !g.inBounds(past));

		const Cell neg = g.worldToCell(Vec2(-5.f, -5.f));
		CHECK(neg.x == -1 && neg.y == -1 && !g.inBounds(neg)); // floor, not truncation

		const Vec2 c = g.cellCenter(Cell{ 0, 0 });
		CHECK(c.x == 10.f && c.y == 10.f);

		// edits and out-of-bounds safety
		g.set(Cell{ 3, 4 }, Tile::Wall);
		CHECK(g.get(Cell{ 3, 4 }) == Tile::Wall);
		g.set(Cell{ -1, 0 }, Tile::Wall);   // must not crash or corrupt
		g.set(Cell{ 40, 30 }, Tile::Wall);
		CHECK(g.get(Cell{ -1, 0 }) == Tile::Wall); // outside reads as blocked
		g.clearWalls();
		CHECK(g.get(Cell{ 3, 4 }) == Tile::Open);
	}

	// ================= Checkpoint 2: BFS =================

	// ---- open field, opposite corners ----
	{
		Grid g(40, 30, 20.f);
		PathSearch ps;
		CHECK(ps.start(g, Cell{ 0, 0 }, Cell{ 39, 29 }));
		CHECK(ps.status() == SearchStatus::Running);
		CHECK(ps.runToEnd(g) == SearchStatus::Found);
		CHECK(ps.path().size() == 69);            // 39 + 29 moves = 68 moves = 69 cells
		CHECK(pathIsValid(g, ps.path(), Cell{ 0, 0 }, Cell{ 39, 29 }));
		CHECK(ps.expanded() == 1200);             // goal is the unique farthest cell: every cell is popped
		std::printf("BFS open field: path=%zu cells, expanded=%d, maxFrontier=%d\n",
			ps.path().size(), ps.expanded(), ps.maxFrontier());
	}

	// ---- start == goal ----
	{
		Grid g(5, 5, 20.f);
		PathSearch ps;
		CHECK(ps.start(g, Cell{ 2, 2 }, Cell{ 2, 2 }));
		CHECK(ps.runToEnd(g) == SearchStatus::Found);
		CHECK(ps.path().size() == 1);
		CHECK(ps.expanded() == 1);
	}

	// ---- wall with a gap: shortest path must go through the gap ----
	{
		Grid g(5, 5, 20.f);
		for (int y = 0; y <= 3; ++y) g.set(Cell{ 2, y }, Tile::Wall);   // gap at (2,4)
		PathSearch ps;
		CHECK(ps.start(g, Cell{ 0, 0 }, Cell{ 4, 0 }));
		CHECK(ps.runToEnd(g) == SearchStatus::Found);
		CHECK(ps.path().size() == 13);            // 12 moves: 6 to (2,4), 6 to (4,0)
		CHECK(pathIsValid(g, ps.path(), Cell{ 0, 0 }, Cell{ 4, 0 }));
		bool throughGap = false;
		for (const Cell& c : ps.path()) if (c == Cell{ 2, 4 }) throughGap = true;
		CHECK(throughGap);

		// a path cell is always Visited; walls are never Visited or Frontier
		for (const Cell& c : ps.path()) CHECK(ps.stateAt(c) == NodeState::Visited);
		CHECK(ps.stateAt(Cell{ 2, 0 }) == NodeState::Unseen);
	}

	// ---- unreachable goal ----
	{
		Grid g(5, 5, 20.f);
		g.set(Cell{ 3, 4 }, Tile::Wall);
		g.set(Cell{ 4, 3 }, Tile::Wall);          // goal (4,4) is sealed in
		PathSearch ps;
		CHECK(ps.start(g, Cell{ 0, 0 }, Cell{ 4, 4 }));
		CHECK(ps.runToEnd(g) == SearchStatus::NoPath);
		CHECK(ps.path().empty());
		CHECK(ps.expanded() == 22);               // 25 cells - 2 walls - the sealed goal
		CHECK(ps.frontierSize() == 0);
		CHECK(ps.expanded() <= countOpen(g));     // no cell is ever expanded twice
	}

	// ---- invalid inputs ----
	{
		Grid g(5, 5, 20.f);
		g.set(Cell{ 1, 1 }, Tile::Wall);
		PathSearch ps;
		CHECK(!ps.start(g, Cell{ 1, 1 }, Cell{ 4, 4 }));    // start on a wall
		CHECK(ps.status() == SearchStatus::Idle);
		CHECK(!ps.start(g, Cell{ 0, 0 }, Cell{ 1, 1 }));    // goal on a wall
		CHECK(!ps.start(g, Cell{ -1, 0 }, Cell{ 4, 4 }));   // out of bounds
		CHECK(!ps.start(g, Cell{ 0, 0 }, Cell{ 5, 5 }));
		ps.step(g);                                         // step while Idle must be a harmless no-op
		CHECK(ps.status() == SearchStatus::Idle);
		CHECK(ps.stateAt(Cell{ 0, 0 }) == NodeState::Unseen);
		CHECK(ps.stateAt(Cell{ -3, 99 }) == NodeState::Unseen);
	}

	// ---- stepping: exactly one expansion per step(); reset clears everything ----
	{
		Grid g(5, 5, 20.f);
		PathSearch ps;
		CHECK(ps.start(g, Cell{ 0, 0 }, Cell{ 4, 4 }));
		CHECK(ps.expanded() == 0);
		CHECK(ps.stateAt(Cell{ 0, 0 }) == NodeState::Frontier);   // seeded start is Frontier, not Visited
		ps.step(g);
		CHECK(ps.expanded() == 1);
		CHECK(ps.stateAt(Cell{ 0, 0 }) == NodeState::Visited);
		CHECK(ps.frontierSize() == 2);                            // (1,0) and (0,1)
		ps.step(g);
		CHECK(ps.expanded() == 2);
		ps.reset();
		CHECK(ps.status() == SearchStatus::Idle);
		CHECK(ps.expanded() == 0 && ps.path().empty());
		CHECK(ps.stateAt(Cell{ 0, 0 }) == NodeState::Unseen);
	}

	// ---- determinism + re-use of one PathSearch object ----
	{
		Grid g(40, 30, 20.f);
		for (int x = 5; x < 35; ++x) g.set(Cell{ x, 15 }, Tile::Wall);
		PathSearch a, b;
		a.start(g, Cell{ 2, 2 }, Cell{ 37, 27 });
		b.start(g, Cell{ 2, 2 }, Cell{ 37, 27 });
		a.runToEnd(g);
		b.runToEnd(g);
		CHECK(a.status() == SearchStatus::Found);
		CHECK(a.path() == b.path());
		CHECK(a.expanded() == b.expanded());
		CHECK(pathIsValid(g, a.path(), Cell{ 2, 2 }, Cell{ 37, 27 }));

		// second search on the same object must not inherit stale state
		a.start(g, Cell{ 0, 0 }, Cell{ 3, 0 });
		CHECK(a.runToEnd(g) == SearchStatus::Found);
		CHECK(a.path().size() == 4);
		CHECK(a.expanded() < 40);
	}

	// ================= Path following =================

	// ---- single leg, partial move ----
	{
		const std::vector<Vec2> w{ Vec2(10.f, 0.f) };
		const auto r = pathfollow::advance(Vec2(0.f, 0.f), w, 0, 4.f);
		CHECK(near(r.pos.x, 4.f, 1e-5f) && near(r.pos.y, 0.f, 1e-5f));
		CHECK(r.next == 0 && !r.arrived);
		CHECK(near(r.dir.x, 1.f, 1e-5f) && near(r.dir.y, 0.f, 1e-5f));
	}

	// ---- exact arrival and overshoot are clamped onto the waypoint ----
	{
		const std::vector<Vec2> w{ Vec2(10.f, 0.f) };
		const auto exact = pathfollow::advance(Vec2(0.f, 0.f), w, 0, 10.f);
		CHECK(exact.arrived && exact.next == 1 && exact.pos == Vec2(10.f, 0.f));
		const auto over = pathfollow::advance(Vec2(0.f, 0.f), w, 0, 1000.f);
		CHECK(over.arrived && over.pos == Vec2(10.f, 0.f));      // never flies past the end
	}

	// ---- corner: leftover distance carries into the next leg ----
	{
		const std::vector<Vec2> w{ Vec2(10.f, 0.f), Vec2(10.f, 10.f) };
		const auto r = pathfollow::advance(Vec2(0.f, 0.f), w, 0, 15.f);   // 10 along x, then 5 along y
		CHECK(near(r.pos.x, 10.f, 1e-4f) && near(r.pos.y, 5.f, 1e-4f));
		CHECK(r.next == 1 && !r.arrived);
		CHECK(near(r.dir.x, 0.f, 1e-5f) && near(r.dir.y, 1.f, 1e-5f));   // direction of the LAST leg
		const auto done = pathfollow::advance(Vec2(0.f, 0.f), w, 0, 20.f);
		CHECK(done.arrived && done.pos == Vec2(10.f, 10.f));
	}

	// ---- degenerate input is safe: empty route, zero/negative distance, duplicate waypoints ----
	{
		const std::vector<Vec2> none;
		const auto e = pathfollow::advance(Vec2(3.f, 4.f), none, 0, 50.f);
		CHECK(e.arrived && e.pos == Vec2(3.f, 4.f));

		const std::vector<Vec2> w{ Vec2(10.f, 0.f) };
		const auto z = pathfollow::advance(Vec2(0.f, 0.f), w, 0, 0.f);
		CHECK(z.pos == Vec2(0.f, 0.f) && !z.arrived && z.dir == Vec2(0.f, 0.f));
		const auto neg = pathfollow::advance(Vec2(0.f, 0.f), w, 0, -5.f);
		CHECK(neg.pos == Vec2(0.f, 0.f));

		const std::vector<Vec2> dup{ Vec2(5.f, 0.f), Vec2(5.f, 0.f), Vec2(5.f, 5.f) };
		const auto d = pathfollow::advance(Vec2(0.f, 0.f), dup, 0, 7.f);
		CHECK(near(d.pos.x, 5.f, 1e-4f) && near(d.pos.y, 2.f, 1e-4f));
		CHECK(!std::isnan(d.pos.x) && !std::isnan(d.pos.y) && !std::isnan(d.dir.x));
	}

	// ---- end to end: BFS path -> waypoints -> walk. 68 moves * 20 px = 1360 px ----
	{
		Grid g(40, 30, 20.f);
		PathSearch ps;
		ps.start(g, Cell{ 0, 0 }, Cell{ 39, 29 });
		CHECK(ps.runToEnd(g) == SearchStatus::Found);

		std::vector<Vec2> wps;
		for (const Cell& c : ps.path()) wps.push_back(g.cellCenter(c));
		CHECK(wps.size() == 69);

		Vec2 end60, end30;
		const int t60 = ticksToArrive(wps, 160.f, 1.f / 60.f, &end60);
		const int t30 = ticksToArrive(wps, 160.f, 1.f / 30.f, &end30);
		std::printf("Walk 1360 px @160 px/s -> 60Hz: %d ticks (%.3fs)   30Hz: %d ticks (%.3fs)\n",
			t60, static_cast<double>(t60) / 60.0, t30, static_cast<double>(t30) / 30.0);

		// 1360 / 160 = 8.5 s exactly: 510 ticks at 60 Hz, 255 at 30 Hz (+-1 for float rounding)
		CHECK(t60 >= 509 && t60 <= 511);
		CHECK(t30 >= 254 && t30 <= 256);
		CHECK(end60 == g.cellCenter(Cell{ 39, 29 }));   // lands EXACTLY on the goal centre
		CHECK(end30 == g.cellCenter(Cell{ 39, 29 }));
	}

	// ---- the path never leaves open cells when sampled finely (agent cannot clip a wall) ----
	{
		Grid g(5, 5, 20.f);
		for (int y = 0; y <= 3; ++y) g.set(Cell{ 2, y }, Tile::Wall);
		PathSearch ps;
		ps.start(g, Cell{ 0, 0 }, Cell{ 4, 0 });
		ps.runToEnd(g);

		std::vector<Vec2> wps;
		for (const Cell& c : ps.path()) wps.push_back(g.cellCenter(c));

		Vec2 pos = wps.front();
		std::size_t next = 1;
		bool clipped = false;
		for (int i = 0; i < 2000 && next < wps.size(); ++i)
		{
			const auto r = pathfollow::advance(pos, wps, next, 0.7f);    // tiny steps, many corners
			pos = r.pos;
			next = r.next;
			if (g.get(g.worldToCell(pos)) == Tile::Wall) clipped = true;
		}
		CHECK(!clipped);
		CHECK(next == wps.size());
	}

	// ================= Checkpoint 3: terrain, 8-way, Dijkstra =================

	// ---- grid terrain basics ----
	{
		Grid g(5, 5, 20.f);
		g.set(Cell{ 1, 1 }, Tile::Mud);
		g.set(Cell{ 2, 2 }, Tile::Wall);
		CHECK(g.isWalkable(Cell{ 1, 1 }) && g.isWalkable(Cell{ 0, 0 }));
		CHECK(!g.isWalkable(Cell{ 2, 2 }) && !g.isWalkable(Cell{ -1, 0 }) && !g.isWalkable(Cell{ 5, 5 }));
		CHECK(near(g.moveCost(Cell{ 0, 0 }), 1.f, 1e-6f));
		CHECK(near(g.moveCost(Cell{ 1, 1 }), 3.f, 1e-6f));
		CHECK(std::isinf(g.moveCost(Cell{ 2, 2 })) && std::isinf(g.moveCost(Cell{ -1, -1 })));
		g.costs.mud = 0.f;                                   // a slider dragged to 0 must not break the search
		CHECK(g.moveCost(Cell{ 1, 1 }) >= Grid::MinMoveCost);
		g.costs.mud = -5.f;
		CHECK(g.moveCost(Cell{ 1, 1 }) >= Grid::MinMoveCost);
	}

	// ---- open field: Dijkstra 4-way equals BFS ----
	{
		Grid g(40, 30, 20.f);
		PathSearch ps;
		CHECK(ps.start(g, Cell{ 0, 0 }, Cell{ 39, 29 }, options(Algorithm::Dijkstra, false)));
		CHECK(ps.runToEnd(g) == SearchStatus::Found);
		CHECK(ps.path().size() == 69);
		CHECK(near(ps.pathCost(), 68.f, 1e-3f));
		CHECK(pathIsValid8(g, ps.path(), Cell{ 0, 0 }, Cell{ 39, 29 }, false, false));
	}

	// ---- open field: Dijkstra 8-way ----
	{
		Grid g(40, 30, 20.f);
		PathSearch ps;
		CHECK(ps.start(g, Cell{ 0, 0 }, Cell{ 39, 29 }, options(Algorithm::Dijkstra, true)));
		CHECK(ps.runToEnd(g) == SearchStatus::Found);
		CHECK(ps.path().size() == 40);                       // 39 moves: 29 diagonal + 10 straight
		CHECK(near(ps.pathCost(), 51.012193f, 1e-3f));       // 29*sqrt2 + 10
		CHECK(ps.expanded() == 1200);                        // goal is the unique farthest cell
		CHECK(near(ps.costAt(Cell{ 39, 29 }), ps.pathCost(), 1e-4f));
		std::printf("Dijkstra 8-way open field: cost=%.3f cells=%zu expanded=%d\n",
			static_cast<double>(ps.pathCost()), ps.path().size(), ps.expanded());
	}

	// ---- small diagonal cases ----
	{
		Grid g(5, 5, 20.f);
		PathSearch ps;
		ps.start(g, Cell{ 0, 0 }, Cell{ 4, 4 }, options(Algorithm::Dijkstra, true));
		CHECK(ps.runToEnd(g) == SearchStatus::Found);
		CHECK(ps.path().size() == 5 && near(ps.pathCost(), 5.656854f, 1e-3f));    // 4*sqrt2

		ps.start(g, Cell{ 0, 0 }, Cell{ 4, 2 }, options(Algorithm::Dijkstra, true));
		CHECK(ps.runToEnd(g) == SearchStatus::Found);
		CHECK(ps.path().size() == 5 && near(ps.pathCost(), 4.828427f, 1e-3f));    // 2*sqrt2 + 2
	}

	// ---- terrain cost vs detour (4-way). Threshold: straight = 3 + 3m, detour = 8  =>  m = 5/3 ----
	{
		for (float m : { 5.f, 2.f })
		{
			Grid g = mudMap(m);
			PathSearch ps;
			ps.start(g, Cell{ 0, 1 }, Cell{ 6, 1 }, options(Algorithm::Dijkstra, false));
			CHECK(ps.runToEnd(g) == SearchStatus::Found);
			CHECK(near(ps.pathCost(), 8.f, 1e-3f));          // detour wins
			CHECK(ps.path().size() == 9);
			CHECK(countMud(g, ps.path()) == 0);
		}

		Grid cheap = mudMap(1.5f);
		PathSearch ps;
		ps.start(cheap, Cell{ 0, 1 }, Cell{ 6, 1 }, options(Algorithm::Dijkstra, false));
		CHECK(ps.runToEnd(cheap) == SearchStatus::Found);
		CHECK(near(ps.pathCost(), 7.5f, 1e-3f));             // straight through the mud: 1 + 3*1.5 + 1 + 1
		CHECK(ps.path().size() == 7);
		CHECK(countMud(cheap, ps.path()) == 3);

		// BFS minimises steps, so it ignores mud entirely and pays for it
		Grid dear = mudMap(5.f);
		PathSearch bfs;
		bfs.start(dear, Cell{ 0, 1 }, Cell{ 6, 1 }, options(Algorithm::BFS, false));
		CHECK(bfs.runToEnd(dear) == SearchStatus::Found);
		CHECK(bfs.path().size() == 7);                       // fewest steps...
		CHECK(near(bfs.pathCost(), 18.f, 1e-3f));            // ...but 1 + 5*3 + 1 + 1 = 18 vs Dijkstra's 8
	}

	// ---- same map, 8-way: a diagonal detour is cheap for any mud cost >= 1.5 ----
	{
		for (float m : { 5.f, 2.f, 1.5f })
		{
			Grid g = mudMap(m);
			PathSearch ps;
			ps.start(g, Cell{ 0, 1 }, Cell{ 6, 1 }, options(Algorithm::Dijkstra, true));
			CHECK(ps.runToEnd(g) == SearchStatus::Found);
			CHECK(near(ps.pathCost(), 6.828427f, 1e-3f));    // 4 + 2*sqrt2
			CHECK(ps.path().size() == 7);
			CHECK(countMud(g, ps.path()) == 0);
		}
	}

	// ---- corner cutting ----
	{
		// two walls box the start in diagonally
		Grid boxed(3, 3, 20.f);
		boxed.set(Cell{ 1, 0 }, Tile::Wall);
		boxed.set(Cell{ 0, 1 }, Tile::Wall);
		PathSearch ps;
		ps.start(boxed, Cell{ 0, 0 }, Cell{ 1, 1 }, options(Algorithm::Dijkstra, true, false));
		CHECK(ps.runToEnd(boxed) == SearchStatus::NoPath);
		CHECK(ps.expanded() == 1);
		CHECK(ps.frontierSize() == 0);

		ps.start(boxed, Cell{ 0, 0 }, Cell{ 1, 1 }, options(Algorithm::Dijkstra, true, true));
		CHECK(ps.runToEnd(boxed) == SearchStatus::Found);
		CHECK(ps.path().size() == 2 && near(ps.pathCost(), 1.414214f, 1e-3f));

		// one wall: forbidden diagonal forces the 2-step way round
		Grid one(3, 3, 20.f);
		one.set(Cell{ 1, 0 }, Tile::Wall);
		ps.start(one, Cell{ 0, 0 }, Cell{ 1, 1 }, options(Algorithm::Dijkstra, true, false));
		CHECK(ps.runToEnd(one) == SearchStatus::Found);
		CHECK(near(ps.pathCost(), 2.f, 1e-3f) && ps.path().size() == 3);

		ps.start(one, Cell{ 0, 0 }, Cell{ 1, 1 }, options(Algorithm::Dijkstra, true, true));
		CHECK(ps.runToEnd(one) == SearchStatus::Found);
		CHECK(near(ps.pathCost(), 1.414214f, 1e-3f) && ps.path().size() == 2);
	}

	// ---- BFS keeps its Checkpoint 2 behaviour with default options, and works 8-way ----
	{
		Grid g(40, 30, 20.f);
		PathSearch ps;
		ps.start(g, Cell{ 0, 0 }, Cell{ 39, 29 });                        // default options
		CHECK(ps.runToEnd(g) == SearchStatus::Found);
		CHECK(ps.path().size() == 69 && ps.expanded() == 1200);
		CHECK(near(ps.pathCost(), 68.f, 1e-3f));

		ps.start(g, Cell{ 0, 0 }, Cell{ 39, 29 }, options(Algorithm::BFS, true));
		CHECK(ps.runToEnd(g) == SearchStatus::Found);
		CHECK(ps.path().size() == 40);                                    // fewest steps = 39 moves
	}

	// ---- unreachable under Dijkstra, and start on mud is fine ----
	{
		Grid g(5, 5, 20.f);
		g.set(Cell{ 3, 4 }, Tile::Wall);
		g.set(Cell{ 4, 3 }, Tile::Wall);
		g.set(Cell{ 0, 0 }, Tile::Mud);
		PathSearch ps;
		CHECK(ps.start(g, Cell{ 0, 0 }, Cell{ 4, 4 }, options(Algorithm::Dijkstra, false)));  // mud start is walkable
		CHECK(ps.runToEnd(g) == SearchStatus::NoPath);
		CHECK(ps.expanded() == 22);                                       // 25 - 2 walls - sealed goal
		CHECK(ps.path().empty());
		CHECK(!ps.start(g, Cell{ 3, 4 }, Cell{ 0, 1 }, options(Algorithm::Dijkstra, false)));  // wall start rejected
	}

	// ---- property test: random maps, Dijkstra never loses to BFS, all invariants hold ----
	{
		std::uint32_t seed = 12345u;
		int compared = 0;
		for (int map = 0; map < 40; ++map)
		{
			Grid g(12, 12, 20.f);
			g.costs.mud = 3.f;
			for (int y = 0; y < 12; ++y)
			{
				for (int x = 0; x < 12; ++x)
				{
					const std::uint32_t r = nextRand(seed) % 100;
					if (r < 22)      g.set(Cell{ x, y }, Tile::Wall);
					else if (r < 40) g.set(Cell{ x, y }, Tile::Mud);
				}
			}

			Cell a{ static_cast<int>(nextRand(seed) % 12), static_cast<int>(nextRand(seed) % 12) };
			Cell b{ static_cast<int>(nextRand(seed) % 12), static_cast<int>(nextRand(seed) % 12) };
			if (!g.isWalkable(a) || !g.isWalkable(b)) continue;

			for (int variant = 0; variant < 4; ++variant)
			{
				const bool diag = (variant & 1) != 0;
				const bool cut = (variant & 2) != 0;
				if (!diag && cut) continue;                                  // cut only matters with diagonals

				PathSearch dj, bf;
				dj.start(g, a, b, options(Algorithm::Dijkstra, diag, cut));
				bf.start(g, a, b, options(Algorithm::BFS, diag, cut));

				// Dijkstra: record the cost of each newly Visited cell, in expansion order
				std::vector<float> popCosts;
				std::vector<char> seen(144, 0);
				while (dj.status() == SearchStatus::Running)
				{
					dj.step(g);
					for (int y = 0; y < 12; ++y)
						for (int x = 0; x < 12; ++x)
						{
							const std::size_t i = static_cast<std::size_t>(y * 12 + x);
							if (!seen[i] && dj.stateAt(Cell{ x, y }) == NodeState::Visited)
							{
								seen[i] = 1;
								popCosts.push_back(dj.costAt(Cell{ x, y }));
							}
						}
				}
				bf.runToEnd(g);

				// both algorithms must agree on reachability
				CHECK((dj.status() == SearchStatus::Found) == (bf.status() == SearchStatus::Found));
				CHECK(dj.expanded() <= countWalkable(g));                     // no cell is expanded twice
				CHECK(std::is_sorted(popCosts.begin(), popCosts.end()));      // cheapest-first order

				if (dj.status() == SearchStatus::Found)
				{
					++compared;
					CHECK(pathIsValid8(g, dj.path(), a, b, diag, cut));
					CHECK(pathIsValid8(g, bf.path(), a, b, diag, cut));
					CHECK(near(dj.pathCost(), recomputeCost(g, dj.path()), 1e-3f));
					CHECK(near(bf.pathCost(), recomputeCost(g, bf.path()), 1e-3f));
					CHECK(near(dj.costAt(b), dj.pathCost(), 1e-4f));
					CHECK(dj.pathCost() <= bf.pathCost() + 1e-3f);            // Dijkstra is optimal
					CHECK(bf.path().size() <= dj.path().size());              // BFS has the fewest steps
				}
			}
		}
		std::printf("Property test: %d Found comparisons\n", compared);
		CHECK(compared > 20);                                                 // the test really exercised something
	}

	// ================= Checkpoint 4: A* =================

	// ---- heuristic values: h((0,0),(4,2)) ----
	{
		const Cell a{ 0, 0 }, b{ 4, 2 };
		CHECK(heuristic(Heuristic::Zero, a, b) == 0.f);
		CHECK(near(heuristic(Heuristic::Chebyshev, a, b), 4.f, 1e-5f));
		CHECK(near(heuristic(Heuristic::Euclidean, a, b), 4.472136f, 1e-4f));
		CHECK(near(heuristic(Heuristic::Octile, a, b), 4.828427f, 1e-4f));         // 2*sqrt2 + 2
		CHECK(near(heuristic(Heuristic::Manhattan, a, b), 6.f, 1e-5f));

		// symmetry, zero at the goal, and the ordering Chebyshev <= Euclidean <= Octile <= Manhattan
		for (int dx = -5; dx <= 5; ++dx)
		{
			for (int dy = -5; dy <= 5; ++dy)
			{
				const Cell p{ 3 + dx, 3 + dy }, q{ 3, 3 };
				const float ch = heuristic(Heuristic::Chebyshev, p, q);
				const float eu = heuristic(Heuristic::Euclidean, p, q);
				const float oc = heuristic(Heuristic::Octile, p, q);
				const float ma = heuristic(Heuristic::Manhattan, p, q);
				CHECK(ch <= eu + 1e-4f && eu <= oc + 1e-4f && oc <= ma + 1e-4f);
				CHECK(near(oc, heuristic(Heuristic::Octile, q, p), 1e-5f));
			}
		}
		CHECK(heuristic(Heuristic::Octile, Cell{ 2, 2 }, Cell{ 2, 2 }) == 0.f);
	}

	// ---- minMoveCost follows the terrain table and never reaches zero ----
	{
		Grid g(3, 3, 20.f);
		CHECK(near(g.minMoveCost(), 1.f, 1e-6f));
		g.costs.open = 2.f; g.costs.mud = 5.f;
		CHECK(near(g.minMoveCost(), 2.f, 1e-6f));
		g.costs.open = 0.f; g.costs.mud = -1.f;
		CHECK(g.minMoveCost() >= Grid::MinMoveCost);
	}

	// ---- A* + Zero heuristic IS Dijkstra: same cost, same expansions, same path ----
	{
		std::uint32_t seed = 777u;
		int compared = 0;
		for (int map = 0; map < 30; ++map)
		{
			Grid g(12, 12, 20.f);
			g.costs.mud = 3.f;
			for (int y = 0; y < 12; ++y)
				for (int x = 0; x < 12; ++x)
				{
					const std::uint32_t r = nextRand(seed) % 100;
					if (r < 22)      g.set(Cell{ x, y }, Tile::Wall);
					else if (r < 40) g.set(Cell{ x, y }, Tile::Mud);
				}
			const Cell a{ static_cast<int>(nextRand(seed) % 12), static_cast<int>(nextRand(seed) % 12) };
			const Cell b{ static_cast<int>(nextRand(seed) % 12), static_cast<int>(nextRand(seed) % 12) };
			if (!g.isWalkable(a) || !g.isWalkable(b)) continue;

			const RunResult d  = run(g, a, b, options(Algorithm::Dijkstra, true));
			const RunResult a0 = run(g, a, b, astar(Heuristic::Zero, true));
			CHECK(d.status == a0.status);
			CHECK(d.expanded == a0.expanded);
			CHECK(d.path == a0.path);
			if (d.status == SearchStatus::Found) { CHECK(near(d.cost, a0.cost, 1e-5f)); ++compared; }
		}
		CHECK(compared > 10);
	}

	// ---- open field, 8-way, Octile: optimal cost, a narrow search ----
	{
		Grid g(40, 30, 20.f);
		const RunResult a = run(g, Cell{ 0, 0 }, Cell{ 39, 29 }, astar(Heuristic::Octile, true));
		CHECK(a.status == SearchStatus::Found);
		CHECK(near(a.cost, 51.012193f, 1e-3f));
		CHECK(a.cells == 40);
		CHECK(a.expanded > 0 && a.expanded < 400);                // reference run: ~180 (varies with tie-break/order)
		std::printf("A* octile 8-way open field: cost=%.3f cells=%d expanded=%d (Dijkstra: 1200)\n",
			static_cast<double>(a.cost), a.cells, a.expanded);
	}

	// ---- tie-breaking: 4-way Manhattan on an open field, every cell in the rectangle has f = 68 ----
	{
		Grid g(40, 30, 20.f);
		const RunResult on  = run(g, Cell{ 0, 0 }, Cell{ 39, 29 }, astar(Heuristic::Manhattan, false, false, 1.f, true));
		const RunResult off = run(g, Cell{ 0, 0 }, Cell{ 39, 29 }, astar(Heuristic::Manhattan, false, false, 1.f, false));
		std::printf("A* manhattan 4-way open field: tie-break ON expanded=%d   OFF expanded=%d\n", on.expanded, off.expanded);
		CHECK(near(on.cost, 68.f, 1e-3f) && near(off.cost, 68.f, 1e-3f));   // both optimal...
		CHECK(on.expanded <= 300);                                           // ...but ON is ~69
		CHECK(off.expanded == 1200);                                         // OFF degrades to Dijkstra's 1200
	}

	// ---- mud strip (7x3) ----
	{
		const float mudCosts[] = { 5.f, 2.f, 1.5f };
		const float want4[]    = { 8.f, 8.f, 7.5f };
		for (int i = 0; i < 3; ++i)
		{
			Grid g = mudMap(mudCosts[i]);
			const RunResult a = run(g, Cell{ 0, 1 }, Cell{ 6, 1 }, astar(Heuristic::Manhattan, false));
			const RunResult d = run(g, Cell{ 0, 1 }, Cell{ 6, 1 }, options(Algorithm::Dijkstra, false));
			CHECK(a.status == SearchStatus::Found);
			CHECK(near(a.cost, want4[i], 1e-3f));
			CHECK(near(a.cost, d.cost, 1e-4f));
			CHECK(a.expanded <= d.expanded);

			const RunResult a8 = run(g, Cell{ 0, 1 }, Cell{ 6, 1 }, astar(Heuristic::Octile, true));
			CHECK(near(a8.cost, 6.828427f, 1e-3f));                          // 4 + 2*sqrt2 for any mud >= 1.5
		}
	}

	// ---- breaking A* on purpose: Manhattan overestimates when diagonals are allowed ----
	{
		const AsciiMap m = parseMap({
			"S....#.#",
			"........",
			"##....#.",
			"....#...",
			".#..##.#",
			".#......",
			"....#...",
			"...###.G" });
		CHECK((m.start == Cell{ 0, 0 }));      // extra parentheses: the macro would split the braces at the comma
		CHECK((m.goal == Cell{ 7, 7 }));

		const RunResult dij  = run(m.grid, m.start, m.goal, options(Algorithm::Dijkstra, true));
		const RunResult oct  = run(m.grid, m.start, m.goal, astar(Heuristic::Octile, true));
		const RunResult euc  = run(m.grid, m.start, m.goal, astar(Heuristic::Euclidean, true));
		const RunResult man  = run(m.grid, m.start, m.goal, astar(Heuristic::Manhattan, true));
		std::printf("8x8 map  Dijkstra %.3f (%d)  Octile %.3f (%d)  Euclid %.3f (%d)  Manhattan %.3f (%d)\n",
			static_cast<double>(dij.cost), dij.expanded, static_cast<double>(oct.cost), oct.expanded,
			static_cast<double>(euc.cost), euc.expanded, static_cast<double>(man.cost), man.expanded);

		CHECK(near(dij.cost, 11.656854f, 1e-3f));
		CHECK(near(oct.cost, 11.656854f, 1e-3f));
		CHECK(near(euc.cost, 11.656854f, 1e-3f));
		CHECK(near(man.cost, 12.242641f, 1e-3f));                 // WRONG: costlier than the optimum...
		CHECK(man.expanded < dij.expanded);                       // ...found faster. Inadmissible = fast but wrong.
		CHECK(oct.expanded < dij.expanded);

		// the same Manhattan heuristic is perfectly fine when movement is 4-way
		const RunResult dij4 = run(m.grid, m.start, m.goal, options(Algorithm::Dijkstra, false));
		const RunResult man4 = run(m.grid, m.start, m.goal, astar(Heuristic::Manhattan, false));
		CHECK(near(dij4.cost, man4.cost, 1e-4f));
	}

	// ---- weighted A*: faster, never worse than weight x optimal ----
	{
		Grid g(40, 30, 20.f);
		const RunResult w1 = run(g, Cell{ 0, 0 }, Cell{ 39, 29 }, astar(Heuristic::Octile, true, false, 1.f));
		const RunResult w2 = run(g, Cell{ 0, 0 }, Cell{ 39, 29 }, astar(Heuristic::Octile, true, false, 2.f));
		const RunResult w5 = run(g, Cell{ 0, 0 }, Cell{ 39, 29 }, astar(Heuristic::Octile, true, false, 5.f));
		std::printf("weighted open field: w=1 expanded=%d  w=2 expanded=%d  w=5 expanded=%d\n", w1.expanded, w2.expanded, w5.expanded);
		CHECK(w2.expanded <= w1.expanded && w5.expanded <= w2.expanded);
		CHECK(w2.cost <= 2.f * w1.cost + 1e-3f);
		CHECK(w5.cost <= 5.f * w1.cost + 1e-3f);

		const AsciiMap m = parseMap({
			"S....#.#", "........", "##....#.", "....#...",
			".#..##.#", ".#......", "....#...", "...###.G" });
		const float opt = run(m.grid, m.start, m.goal, options(Algorithm::Dijkstra, true)).cost;
		for (float w : { 1.f, 1.5f, 2.f, 3.f, 10.f })
		{
			const RunResult r = run(m.grid, m.start, m.goal, astar(Heuristic::Octile, true, false, w));
			CHECK(r.status == SearchStatus::Found);
			CHECK(r.cost >= opt - 1e-3f);                        // never BETTER than optimal
			CHECK(r.cost <= w * opt + 1e-3f);                    // and bounded by weight x optimal
		}
	}

	// ---- property test: admissible heuristics always match Dijkstra's cost ----
	{
		std::uint32_t seed = 2468u;
		int compared = 0;
		long long totalDijkstra = 0, totalOctile = 0, totalManhattan4 = 0, totalDijkstra4 = 0;
		for (int map = 0; map < 60; ++map)
		{
			Grid g(12, 12, 20.f);
			g.costs.mud = 3.f;
			for (int y = 0; y < 12; ++y)
				for (int x = 0; x < 12; ++x)
				{
					const std::uint32_t r = nextRand(seed) % 100;
					if (r < 22)      g.set(Cell{ x, y }, Tile::Wall);
					else if (r < 40) g.set(Cell{ x, y }, Tile::Mud);
				}
			const Cell a{ static_cast<int>(nextRand(seed) % 12), static_cast<int>(nextRand(seed) % 12) };
			const Cell b{ static_cast<int>(nextRand(seed) % 12), static_cast<int>(nextRand(seed) % 12) };
			if (!g.isWalkable(a) || !g.isWalkable(b)) continue;

			for (int cut = 0; cut < 2; ++cut)
			{
				const RunResult d = run(g, a, b, options(Algorithm::Dijkstra, true, cut != 0));
				for (Heuristic h : { Heuristic::Octile, Heuristic::Euclidean, Heuristic::Chebyshev, Heuristic::Zero })
				{
					const RunResult r = run(g, a, b, astar(h, true, cut != 0));
					CHECK(r.status == d.status);
					if (d.status == SearchStatus::Found)
					{
						CHECK(near(r.cost, d.cost, 1e-3f));
						CHECK(pathIsValid8(g, r.path, a, b, true, cut != 0));
					}
					if (h == Heuristic::Octile && d.status == SearchStatus::Found)
					{
						++compared;
						totalDijkstra += d.expanded;
						totalOctile += r.expanded;
					}
				}
			}

			// 4-way: Manhattan is exact and admissible
			const RunResult d4 = run(g, a, b, options(Algorithm::Dijkstra, false));
			const RunResult m4 = run(g, a, b, astar(Heuristic::Manhattan, false));
			CHECK(d4.status == m4.status);
			if (d4.status == SearchStatus::Found)
			{
				CHECK(near(d4.cost, m4.cost, 1e-3f));
				totalDijkstra4 += d4.expanded;
				totalManhattan4 += m4.expanded;
			}
		}
		std::printf("Property test: %d Found comparisons. Expansions  8-way: Dijkstra %lld vs A* octile %lld   4-way: Dijkstra %lld vs A* manhattan %lld\n",
			compared, totalDijkstra, totalOctile, totalDijkstra4, totalManhattan4);
		CHECK(compared > 30);
		CHECK(totalOctile < totalDijkstra);
		CHECK(totalManhattan4 < totalDijkstra4);
	}

	if (g_failures == 0) std::printf("All tests passed.\n");
	else                 std::printf("%d check(s) failed.\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}