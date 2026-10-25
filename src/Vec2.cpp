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

Vec2& Vec2::operator += (const Vec2& rhs)
{
	x += rhs.x;
	y += rhs.y;
	return *this;
}

Vec2& Vec2::operator -= (const Vec2& rhs)
{
	x -= rhs.x;
	y -= rhs.y;
	return *this;
}

Vec2& Vec2::operator *= (const float val)
{
	x *= val;
	y *= val;
	return *this;
}

Vec2& Vec2::operator /= (const float val)
{
	x /= val;
	y /= val;
	return *this;
}

float Vec2::lengthSquared() const
{
	return x * x + y * y;
}

float Vec2::length() const
{
	return std::sqrt(lengthSquared());
}

float Vec2::dot(const Vec2& rhs) const
{
	return x * rhs.x + y * rhs.y;
}

Vec2 Vec2::normalized() const
{
	const float lenSq = lengthSquared();

	// Compare squared length to avoid a sqrt just to reject tiny vectors.
	// 1e-12 on lenSq == 1e-6 on length.
	if (lenSq < 1e-12f)
	{
		return Vec2(0.f, 0.f);
	}

	return *this / std::sqrt(lenSq);
}

Vec2 Vec2::limited(float maxLength) const
{
	if (maxLength <= 0.f)
	{
		return Vec2(0.f, 0.f);
	}

	const float lenSq = lengthSquared();

	if (lenSq > maxLength * maxLength)
	{
		// lenSq > maxLength^2 > 0, so the division inside normalized() is safe
		return normalized() * maxLength;
	}

	return *this;
}

float Vec2::distance(const Vec2& destination) const
{
	// difference vector
	Vec2 diffVec = destination - *this;
	return diffVec.length();
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
