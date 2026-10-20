#pragma once

class Vec2
{
public:
	float x = 0;
	float y = 0;
	
	Vec2();
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

	// calculates the distance between destination and origin (this*)
	float distance(const Vec2& destination) const;

	// calculates the angle of this* using its x,y components
	// this* must be a difference vector(distance) between two position vector (Vec2 destination - Vec2 origin)
	// returns angle in degrees
	float angle() const;
	
	// calculates the velocity vector
	// angle must be in degrees
	static Vec2 velocity(float speed, float angle);
};