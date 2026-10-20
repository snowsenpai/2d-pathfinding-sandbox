#pragma once

#include "EntityManager.h"
#include "Entity.h"

#include <SFML/Graphics.hpp>

#include <memory>
#include <type_traits>
#include <random>

class Game
{
	EntityManager m_entityManager;
	sf::RenderWindow m_window;
	std::shared_ptr<Entity> m_player;
	
	std::random_device m_randomDevice;
	mutable std::mt19937 m_engine{ m_randomDevice() }; // seeded once; rng() draws from this, not a fresh engine per call

	bool m_running = false;
	bool m_drawCollisison = false;

	void init();
	void update();
	void quit();
	void spawnPlayer();
	
	void sRender();
	void sPlayerInput();
	void sMovement();
	void sCollision();

	template <typename T>
	T rng(T min, T max) const;

public:
	Game() = default;

	void run();
};

template <typename T>
T Game::rng(T min, T max) const
{
	static_assert(std::is_arithmetic_v<T>, "rng<T>: T must be an arithmetic type");
	static_assert(!std::is_same_v<T, bool>, "rng<T>: bool is not supported by uniform_int_distribution");
	static_assert(!std::is_same_v<T, char> && !std::is_same_v<T, signed char> && !std::is_same_v<T, unsigned char>,
		"rng<T>: char types are not supported by uniform_int_distribution; use int instead");

	using Dist = std::conditional_t<std::is_integral_v<T>,
		std::uniform_int_distribution<T>,
		std::uniform_real_distribution<T>>;

	Dist dist(min, max);
	return dist(m_engine); // m_engine is seeded once at construction, not rebuilt per call
}