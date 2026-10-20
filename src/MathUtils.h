#pragma once

namespace math
{
	inline constexpr float Pi = 3.14159265358979323846f;
	inline constexpr float TwoPi = 2.0f * Pi;

	constexpr float toRadians(float degrees) { return degrees * Pi / 180.0f; }
	constexpr float toDegrees(float radians) { return radians * 180.0f / Pi; }
}