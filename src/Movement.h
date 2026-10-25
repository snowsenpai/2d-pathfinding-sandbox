#pragma once

#include "Vec2.h"

// Pure movement math (no entities, no SFML) so it can be tested directly.
namespace movement
{
	// Opposite keys cancel. Diagonals are normalized so W+D is not ~1.41x faster.
	// Returns a unit vector (or (0,0) when nothing / cancelling keys are held).
	// Screen space: up is -y.
	inline Vec2 directionFromFlags(bool up, bool down, bool left, bool right)
	{
		Vec2 dir{ 0.f, 0.f };
		if (up)    dir.y -= 1.f;
		if (down)  dir.y += 1.f;
		if (left)  dir.x -= 1.f;
		if (right) dir.x += 1.f;
		return dir.normalized();
	}

	// position (px) += velocity (px/s) * dt (s).  dt is applied exactly once, here.
	// Acceleration enters in later checkpoints as: velocity += accel * dt, THEN this call.
	inline Vec2 integratePosition(const Vec2& pos, const Vec2& velocity, float dt)
	{
		return pos + velocity * dt;
	}
}
