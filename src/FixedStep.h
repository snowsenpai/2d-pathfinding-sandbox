#pragma once

#include <algorithm>

// Fixed-timestep accumulator. No SFML, no ImGui: pure logic, so Tests.cpp can drive it.
//
// DATA FLOW
//   real frame time (seconds, from sf::Clock)
//        |  clamp to [0, maxFrameTime]            <- kills huge spikes (window drag, breakpoint)
//        v
//   accumulator += frameTime
//        |  while (accumulator >= fixedDt && ticks < maxTicksPerFrame)
//        v
//   returns N = how many fixed simulation ticks Game must run THIS frame
//
// Rendering and ImGui run once per frame regardless of N (N can be 0, 1, 2, ...).
struct FixedStepper
{
	float fixedDt = 1.0f / 60.0f;  // seconds per simulation tick
	float maxFrameTime = 0.25f;    // spike clamp (seconds)
	int   maxTicksPerFrame = 5;    // spiral-of-death guard
	float accumulator = 0.0f;      // leftover real time not yet simulated (seconds)

	int advance(float frameTime)
	{
		// negative or huge frame times must not poison the accumulator
		accumulator += std::min(std::max(frameTime, 0.0f), maxFrameTime);

		int ticks = 0;
		while (accumulator >= fixedDt && ticks < maxTicksPerFrame)
		{
			accumulator -= fixedDt;
			++ticks;
		}

		// If we hit the tick cap and still owe time, DROP the backlog.
		// Otherwise a slow frame makes the next frame run even more ticks, which makes it
		// slower still (the "spiral of death"). Dropping means the sim runs in slow motion
		// instead of freezing. This is a design choice; the alternative is to keep the backlog.
		if (accumulator >= fixedDt)
		{
			accumulator = 0.0f;
		}

		return ticks;
	}

	// 0..1: how far we are between the last tick and the next one.
	// Unused in Checkpoint 1. Rendering could draw lerp(prevPos, pos, alpha()) to smooth
	// 60 Hz ticks on a 144 Hz display (CTransform already has prevPos).
	float alpha() const { return accumulator / fixedDt; }
};
