#pragma once

#include "Vec2.h"

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>

#include <cstddef>
#include <vector>

class CTransform
{
public:
	Vec2 pos = { 0.0f, 0.0f };
	Vec2 prevPos = { 0.0f, 0.0f };
	Vec2 velocity = { 0.0f, 0.0f };
	float angle = 0.0f;

	CTransform(Vec2 p) : pos(p) {}

	CTransform(Vec2 p, Vec2 v, float a)
		: pos(p), velocity(v), angle(a) {}
};

class CBoundingBox
{
public:
	Vec2 size = { 0.0f, 0.0f };

	CBoundingBox(Vec2 size)
		: size(size) {}
};

class CBoundingCircle
{
public:
	float radius = 0.0f;

	CBoundingCircle(float r)
		: radius(r) {}
};

class CShape
{
public:
	sf::CircleShape circle;

	CShape(float radius, int points, const sf::Color& fill, const sf::Color& outline, float thickness)
		: circle(radius, points)
	{
		circle.setFillColor(fill);
		circle.setOutlineColor(outline);
		circle.setOutlineThickness(thickness);
		circle.setOrigin({ radius, radius });
	}
};

// Where a path follower is in its life.
enum class FollowState { Idle, Following, Arrived };

// Marks an entity as driven by a route instead of the keyboard.
// CONTROL OWNERSHIP: an entity is moved by exactly ONE of CInput (sMovement) or CPathFollower
// (sPathFollow). Never both, or two systems would fight over velocity.
// Waypoints are world-space pixels (not grid cells) so this component knows nothing about Grid.
class CPathFollower
{
public:
	std::vector<Vec2> waypoints;
	std::size_t next = 0;                 // index of the waypoint currently being approached
	float speed = 160.0f;                 // px/s
	FollowState state = FollowState::Idle;

	CPathFollower() = default;
	explicit CPathFollower(float speedPxPerSec) : speed(speedPxPerSec) {}
};

class CInput
{
public:
	bool up = false;
	bool down = false;
	bool left = false;
	bool right = false;
	bool shoot = false;

	CInput() {}
};