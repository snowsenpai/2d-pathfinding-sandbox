#include "Vec2.h"
#include <cmath>

Vec2::Vec2() {}

Vec2::Vec2(float xIn, float yIn) : x(xIn), y(yIn) {}

bool Vec2::operator == (const Vec2& rhs) const
{
	return (x == rhs.x && y == rhs.y);
}

bool Vec2::operator != (const Vec2& rhs) const
{
	return (x != rhs.x && y != rhs.y);
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

constexpr double pi = 3.14159265358979323846;

float Vec2::angle() const
{
	float rad = std::atan2(y, x);
	float deg = static_cast<float>(rad * 180 / pi);

	return deg;
}

Vec2 Vec2::velocity(float speed, float angle)
{
	double rad = angle * pi / 180;
	return Vec2(speed * static_cast<float>(std::cos(rad)), speed * static_cast<float>(std::sin(rad)));
}