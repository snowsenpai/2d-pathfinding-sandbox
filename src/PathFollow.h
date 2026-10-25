#pragma once

#include "Vec2.h"

#include <algorithm>
#include <cstddef>
#include <vector>

// Pure path-following math: no entities, no SFML, no Grid.  Waypoints are plain world-space
// positions (pixels), so this works for ANY route source (BFS now, A*/flow fields later).
//
// advance() answers: "if I travel `distance` pixels along this route, where do I end up?"
//
//   pos ---> waypoint[next] ---> waypoint[next+1] ---> ...
//   distance budget is spent leg by leg. When a waypoint is reached with budget left, the
//   agent snaps onto it and CONTINUES toward the next one in the same call.
//
// Why a distance budget instead of "velocity * dt, then check a radius":
//   * no overshoot/orbiting when speed*dt is larger than any arrival radius
//   * corners cost no speed: leftover movement carries into the next leg
//   * the agent lands exactly on every waypoint, so it never cuts a corner into a wall
namespace pathfollow
{
	struct StepResult
	{
		Vec2 pos;              // position after moving
		std::size_t next;      // index of the waypoint we are now heading to
		Vec2 dir;              // unit direction of the LAST leg travelled; (0,0) if we did not move
		bool arrived;          // true once the final waypoint has been reached
	};

	// Preconditions: none. An empty route, next >= size, zero or negative distance are all safe.
	inline StepResult advance(const Vec2& pos,
		const std::vector<Vec2>& waypoints,
		std::size_t next,
		float distance)
	{
		StepResult r{ pos, next, Vec2(0.f, 0.f), next >= waypoints.size() };

		float remaining = std::max(distance, 0.f);

		while (!r.arrived && remaining > 0.f)
		{
			const Vec2 toTarget = waypoints[r.next] - r.pos;
			const float dist = toTarget.length();

			if (dist <= remaining)
			{
				// Reach this waypoint this call: land exactly on it, spend the distance, move on.
				r.pos = waypoints[r.next];
				remaining -= dist;
				if (dist > 0.f) r.dir = toTarget / dist;   // dist > 0, so the division is safe
				++r.next;
				r.arrived = r.next >= waypoints.size();
			}
			else
			{
				// Not enough budget to reach it: move part of the way and stop.
				// Here dist > remaining > 0, so the division is safe.
				r.dir = toTarget / dist;
				r.pos += r.dir * remaining;
				remaining = 0.f;
			}
		}

		return r;
	}
}