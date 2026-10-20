#pragma once

class Vec2
{
public:
	float x = 0.f;
	float y = 0.f;

	Vec2() = default;
	Vec2(float xIn, float yIn);

	bool operator == (const Vec2& rhs) const;
	bool operator != (const Vec2& rhs) const;

	Vec2 operator + (const Vec2& rhs) const;
	Vec2 operator - (const Vec2& rhs) const;
	Vec2 operator * (const float val) const;
	Vec2 operator / (const float val) const;

	void operator += (const Vec2& rhs);
	void operator -= (const Vec2& rhs);
	void operator *= (const float val);
	void operator /= (const float val);

	// angle of this vector measured from the +x axis, via atan2(y, x)
	// range: (-Pi, Pi]. NOTE: screen y points DOWN, so positive angles rotate clockwise on screen
	float angleRadians() const;
	float angleDegrees() const;

	// vector of the given length pointing at the given angle
	static Vec2 fromAngleRadians(float length, float radians);
	static Vec2 fromAngleDegrees(float length, float degrees);

	// calculates the distance between destination and origin(this*)
	float distance(const Vec2& destination) const;
};