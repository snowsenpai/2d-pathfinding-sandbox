#pragma once

// Units convention for this project (keep it everywhere):
//   position      -> pixels
//   velocity      -> pixels / second
//   acceleration  -> pixels / second^2   (later checkpoints)
//   dt            -> seconds
// SFML screen space: +x is right, +y is DOWN (origin top-left).

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
	Vec2 operator / (const float val) const; // NOTE: no zero guard; callers must check

	// CHANGED: return Vec2& (was void) so they can chain: a += b += c
	Vec2& operator += (const Vec2& rhs);
	Vec2& operator -= (const Vec2& rhs);
	Vec2& operator *= (const float val);
	Vec2& operator /= (const float val);

	// --- new in Checkpoint 1 ---
	float lengthSquared() const;                 // cheap: no sqrt, use for comparisons
	float length() const;                        // pixels (or px/s, px/s^2 depending on what the vector is)
	float dot(const Vec2& rhs) const;

	// Unit vector in the same direction.
	// Policy: a (near) zero-length vector returns (0,0) instead of NaN.
	// The caller decides what "no direction" means.
	Vec2 normalized() const;

	// Same direction, length clamped to at most maxLength. Never scales up.
	Vec2 limited(float maxLength) const;

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
