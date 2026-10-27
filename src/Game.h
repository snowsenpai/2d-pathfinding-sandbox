#pragma once

#include "Entity.h"
#include "EntityManager.h"
#include "FixedStep.h"
#include "Grid.h"
#include "PathSearch.h"

#include <SFML/Graphics/PrimitiveType.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Graphics/VertexArray.hpp>
#include <SFML/Graphics/View.hpp>
#include <SFML/System/Clock.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <type_traits>
#include <vector>

enum class Tool { Wall, Erase, Start, Goal, Mud };

// What the UI asks the search system to do next frame. The UI only REQUESTS; sSearch executes.
enum class SearchCommand { None, Start, Step, Reset, Compare };

// Same pattern for the agent: the UI requests, sAgentControl executes.
enum class AgentCommand { None, Run, Remove };

// One line of the "Compare all" table: the same start/goal/options run with each algorithm.
struct CompareRow
{
	Algorithm algo = Algorithm::BFS;
	SearchStatus status = SearchStatus::Idle;
	int expanded = 0;
	int maxFrontier = 0;
	float cost = 0.f;
	int cells = 0;
	double microseconds = 0.0;   // wall-clock time; at this map size it is mostly noise, expansions are the real signal
};

class Game
{
	// ---------- core (existing) ----------
	EntityManager m_entityManager;
	sf::RenderWindow m_window;
	std::shared_ptr<Entity> m_player;

	std::random_device m_randomDevice;
	mutable std::mt19937 m_engine{ m_randomDevice() }; // seeded once; rng() draws from this, not a fresh engine per call

	bool m_running = false;
	bool m_drawCollision = false;

	bool m_fullscreen = false;
	bool m_toggleFullscreen = false;   // request flag; applied at the top of the next frame
	sf::View m_worldView;              // world -> window mapping (letterboxed)

	// ---------- map (decision: Grid is owned by Game; it is data, not an entity) ----------
	Grid m_grid{ 40, 30, 20.f };          // 40*20 = 800 px, 30*20 = 600 px  (window is sized from this)
	std::optional<Cell> m_start;          // decision: search parameters live on Game, not on Grid
	std::optional<Cell> m_goal;
	Tool m_tool = Tool::Wall;

	// ---------- search (Checkpoint 2) ----------
	PathSearch m_search;                                   // all search state lives here, not in ImGui
	SearchCommand m_searchCommand = SearchCommand::None;   // set by sImGui, consumed by sSearch
	bool  m_autoRun = false;
	float m_expansionsPerSecond = 30.f;                    // search speed, independent of the 60 Hz sim ticks
	float m_searchAccum = 0.f;                             // fractional expansions owed (carries between frames)
	bool  m_mapChanged = false;                            // set by any edit; invalidates a running/finished search
	bool  m_showSearchOverlay = true;
	bool  m_showHeatmap = false;                           // colour visited cells by their cost
	SearchOptions m_searchOptions;                         // algorithm, 4/8-way, corner rule (edited in ImGui)
	std::vector<CompareRow> m_compareRows;                 // filled by runComparison(), cleared when the map or options change
	bool  m_searchOptionsChanged = false;                  // options changed: reset the search, but the map itself is fine

	// ---------- agent (path-following simulation) ----------
	std::shared_ptr<Entity> m_agent;                       // null when no agent exists
	AgentCommand m_agentCommand = AgentCommand::None;      // set by sImGui, consumed by sAgentControl
	float m_agentSpeed = 160.f;                            // px/s, copied into CPathFollower every frame

	// ---------- time ----------
	sf::Clock m_clock;                    // measures REAL frame time
	FixedStepper m_stepper;               // turns real time into N fixed ticks
	bool m_paused = false;
	bool m_stepRequested = false;         // consumed once per frame, only honoured while paused
	std::uint64_t m_tickCount = 0;
	double m_simTime = 0.0;               // seconds of SIMULATED time (tickCount * fixedDt)
	int m_lastFrameTicks = 0;             // ticks run in the most recent frame (debug display)
	int m_fpsLimitIndex = 1;              // index into the frame-limit table in Game.cpp (60)

	// ---------- tuning ----------
	float m_playerSpeed = 240.f;          // px/s  (old code: 8 px/frame * 60 fps = 480 px/s, which felt far too fast for 800 px)

	// ---------- render caches (built once, reused every frame: no per-frame heap churn) ----------
	sf::VertexArray m_gridLines{ sf::PrimitiveType::Lines };
	sf::RectangleShape m_cellShape;
	sf::VertexArray m_pathLine{ sf::PrimitiveType::LineStrip }; // rebuilt only when a path is Found

	void init();
	void update();
	void quit();
	void spawnPlayer();
	void buildGridLines();
	void applyFramerateLimit();

	void createWindow();               // (re)creates m_window for the current mode
	void updateView();                 // rebuilds m_worldView for the current window size
	void applyDisplayMode();           // full recreate cycle incl. ImGui
	void runComparison();              // BFS vs Dijkstra vs A* on private PathSearch copies
	void runAgent();                   // spawn an agent on the start cell and send it down the found path
	void removeAgent();
	void syncPathLine();               // keeps m_pathLine matching m_search (build on Found, clear otherwise)
	Vec2 worldSize() const;            // pixels, from the grid: the truth about map size

	// per-FRAME systems (run once per rendered frame)
	void sUserInput();
	void sImGui();
	void sEditTools();
	void sAgentControl();
	void sSearch(float frameTime);
	void sRender();

	// per-TICK systems (run 0..N times per frame, always with the fixed dt)
	void sMovement(float dt);
	void sPathFollow(float dt);
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