// Checkpoint 1 + 2 + path-following tests. No SFML / ImGui needed: links Vec2.cpp, Grid.cpp, PathSearch.cpp.
// Build target: sandbox_tests.  Exit code 0 = all passed.

#include "FixedStep.h"
#include "Grid.h"
#include "Movement.h"
#include "PathFollow.h"
#include "PathSearch.h"
#include "Vec2.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
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
		const Vec2 at60 = simulate(240.f, 60.f, 1.f);
		const Vec2 at30 = simulate(240.f, 30.f, 1.f);
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

	if (g_failures == 0) std::printf("All tests passed.\n");
	else                 std::printf("%d check(s) failed.\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}