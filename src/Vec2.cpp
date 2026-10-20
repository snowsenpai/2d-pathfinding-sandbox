#include "Vec2.h"
#include "MathUtils.h"
#include <cmath>

Vec2::Vec2(float xIn, float yIn) : x(xIn), y(yIn) {}

bool Vec2::operator == (const Vec2& rhs) const
{
	return (x == rhs.x && y == rhs.y);
}

bool Vec2::operator != (const Vec2& rhs) const
{
	return !(*this == rhs);
}

Vec2 Vec2::operator + (const Vec2& rhs) const
{
	return Vec2(x + rhs.x, y + rhs.y);
}

Vec2 Vec2::operator - (const Vec2& rhs) const
{
	return Vec2(x - rhs.x, y - rhs.y);
}

Vec2 Vec2::operator * (const float val) const
{
	return Vec2(x * val, y * val);
}

Vec2 Vec2::operator / (const float val) const
{
	return Vec2(x / val, y / val);
}

void Vec2::operator += (const Vec2& rhs)
{
	x += rhs.x;
	y += rhs.y;
}

void Vec2::operator -= (const Vec2& rhs)
{
	x -= rhs.x;
	y -= rhs.y;
}

void Vec2::operator *= (const float val)
{
	x *= val;
	y *= val;
}

void Vec2::operator /= (const float val)
{
	x /= val;
	y /= val;
}

float Vec2::distance(const Vec2& destination) const
{
	// difference vector
	Vec2 diffVec = destination - *this;

	float distance = std::sqrt(diffVec.x * diffVec.x + diffVec.y * diffVec.y);

	return distance;
}

float Vec2::angleRadians() const { return std::atan2(y, x); }
float Vec2::angleDegrees() const { return math::toDegrees(angleRadians()); }

Vec2 Vec2::fromAngleRadians(float length, float radians)
{
	return Vec2(length * std::cos(radians), length * std::sin(radians));
}

Vec2 Vec2::fromAngleDegrees(float length, float degrees)
{
	return fromAngleRadians(length, math::toRadians(degrees));
}