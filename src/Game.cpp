#include "Components.h"
#include "Game.h"
#include "Grid.h"
#include "Movement.h"
#include "PathFollow.h"
#include "Vec2.h"

#include <imgui.h>
#include <imgui-SFML.h>

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/Vertex.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/Window/VideoMode.hpp>
#include <SFML/Window/WindowEnums.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>

namespace
{
	// Index into these two arrays is what the ImGui combo stores (m_fpsLimitIndex).
	// 0 means "no limit" for sf::Window::setFramerateLimit.
	constexpr unsigned int FrameLimits[] = { 30, 60, 144, 0 };
	const char* FrameLimitLabels[] = { "30", "60", "144", "Unlimited" };

	const sf::Color BackgroundColor(25, 25, 30);
	const sf::Color GridLineColor(60, 60, 70);
	const sf::Color WallColor(150, 150, 165);
	const sf::Color StartColor(60, 200, 90);
	const sf::Color GoalColor(220, 70, 70);
	const sf::Color VisitedColor(70, 90, 160, 170);   // expanded cells
	const sf::Color FrontierColor(230, 190, 60, 210); // queued, waiting to be expanded
	const sf::Color PathColor(0, 230, 230);           // final path through cell centres
}

// ======================================================================================
// FRAME LOOP  --  who produces what, who consumes it
//
//   sf::Clock ---------> frameTime (real seconds)
//   window events -----> sUserInput:  ImGui::SFML::ProcessEvent (ImGui sees every event)
//                                     CInput flags (W/A/S/D)  [only if ImGui doesn't want keyboard]
//   frameTime ---------> ImGui::SFML::Update   (ONCE per frame; starts the ImGui frame)
//   ImGui widgets -----> sImGui:      writes m_paused, m_tool, m_playerSpeed, ... (UI is a VIEW,
//                                     the values live in Game, not inside the widgets)
//   mouse + m_tool ----> sEditTools:  writes m_grid tiles, m_start, m_goal, m_mapChanged
//                                     [skipped if ImGui wants the mouse]
//   m_mapChanged, ----> sSearch:      reset on edit; run the UI command (start/step/reset);
//   m_searchCommand,                  auto-run: accumulate frameTime * expansions/s, do N steps;
//   frameTime                         then syncPathLine().  Writes m_search, m_pathLine.
//                                     (frame-time driven, NOT part of the fixed ticks)
//   m_agentCommand, --> sAgentControl: stop the agent if the map changed; run/remove on request;
//   m_mapChanged,                     copies m_agentSpeed into CPathFollower.
//   m_search.path()                   Runs BEFORE sSearch (it needs to see the edit, and the
//                                     search result from before it). update() clears m_mapChanged
//                                     after both have looked at it.
//   frameTime ---------> FixedStepper -> N ticks.  For each tick (always dt = 1/60 s):
//        CInput flags --> sMovement(dt):  velocity (px/s), then pos += velocity * dt
//        CPathFollower --> sPathFollow(dt): spend speed*dt of distance along the waypoints,
//                                          writes CTransform.pos/velocity (agents only)
//        CTransform ----> sCollision():   clamp pos inside the window
//   m_grid, CTransform -> sRender:    grid, entities, then ImGui::SFML::Render on top
// ======================================================================================

void Game::run()
{
	init();

	while (m_running)
	{
		update();
	}

	ImGui::SFML::Shutdown();
	m_window.close();
}

void Game::update()
{
	// Must happen OUTSIDE an ImGui frame (between Update() and Render() is too late).
	if (m_toggleFullscreen)
	{
		m_toggleFullscreen = false;
		m_fullscreen = !m_fullscreen;
		applyDisplayMode();
		if (!m_running) return;
	}

	// Real time since the previous frame. Taken first so every system this frame agrees on it.
	const float frameTime = m_clock.restart().asSeconds();

	m_entityManager.update();

	sUserInput();                                       // events: once per frame
	ImGui::SFML::Update(m_window, sf::seconds(frameTime)); // ImGui frame start: once per frame, NEVER per tick
	sImGui();                                           // build widgets (may toggle pause / request a step)
	sEditTools();                                       // paint walls / place start+goal
	sAgentControl();                                    // agent: react to edits / UI commands
	sSearch(frameTime);                                 // pathfinding: once per frame, own accumulator
	m_mapChanged = false;                               // every per-frame system above has now seen the edit

	// ---- how many fixed ticks to run this frame? ----
	int ticks = 0;
	if (m_paused)
	{
		// Paused: the accumulator is NOT fed, so unpausing never "catches up".
		// A step request runs exactly one tick and does not touch the accumulator.
		if (m_stepRequested) ticks = 1;
	}
	else
	{
		ticks = m_stepper.advance(frameTime);           // 0, 1, 2... depending on frame time
	}
	m_stepRequested = false;                            // a step request is valid for one frame only

	for (int i = 0; i < ticks; ++i)
	{
		sMovement(m_stepper.fixedDt);                   // dt is the FIXED step, never the frame time
		sPathFollow(m_stepper.fixedDt);                 // agents walk their routes (frozen while paused)
		sCollision();
		++m_tickCount;
		m_simTime += static_cast<double>(m_stepper.fixedDt);
	}
	m_lastFrameTicks = ticks;

	sRender();                                          // once per frame
}

void Game::quit()
{
	m_running = false;
}

void Game::init()
{
	// Window is sized from the grid so 1 grid cell == cellSize real pixels (no scaling).
	// Decision: not resizable, so the pixel <-> cell mapping can never silently break.
	createWindow();   // also applies the frame limit and builds the view

	// ImGui-SFML owns the ImGui context. Initialise exactly once, right after the window exists.
	if (!ImGui::SFML::Init(m_window))
	{
		return; // m_running stays false -> run() exits
	}
	ImGui::GetIO().IniFilename = nullptr; // don't write imgui.ini next to the exe

	m_cellShape.setSize({ m_grid.cellSize(), m_grid.cellSize() });
	buildGridLines();

	spawnPlayer();
	m_running = true;

	// Loading time must not become the first frame's dt.
	m_clock.restart();
}

void Game::applyFramerateLimit()
{
	m_window.setFramerateLimit(FrameLimits[m_fpsLimitIndex]);
}

void Game::buildGridLines()
{
	// Built once. Lines are thin so SFML's Lines primitive is enough.
	m_gridLines.clear();

	const float w = static_cast<float>(m_grid.cols()) * m_grid.cellSize();
	const float h = static_cast<float>(m_grid.rows()) * m_grid.cellSize();

	for (int cx = 0; cx <= m_grid.cols(); ++cx)
	{
		const float x = static_cast<float>(cx) * m_grid.cellSize();
		m_gridLines.append(sf::Vertex{ { x, 0.f }, GridLineColor });
		m_gridLines.append(sf::Vertex{ { x, h },   GridLineColor });
	}
	for (int cy = 0; cy <= m_grid.rows(); ++cy)
	{
		const float y = static_cast<float>(cy) * m_grid.cellSize();
		m_gridLines.append(sf::Vertex{ { 0.f, y }, GridLineColor });
		m_gridLines.append(sf::Vertex{ { w, y },   GridLineColor });
	}
}

void Game::spawnPlayer()
{
	auto player = m_entityManager.addEntity("player");

	float playerRadius = m_grid.cellSize() * 0.5f;
	int shapePoints = 10;
	sf::Color fillColor(sf::Color::Transparent);
	sf::Color outlineColor(sf::Color::Red);
	float thickness = 3.f;

	player->addComponent<CShape>(playerRadius, shapePoints, fillColor, outlineColor, thickness);

	// spawn player at a random position within the world
	const Vec2 ws = worldSize();
	float maxPosX = ws.x - playerRadius;
	float maxPosY = ws.y - playerRadius;

	float posX = rng(playerRadius, maxPosX);
	float posY = rng(playerRadius, maxPosY);

	player->addComponent<CTransform>(Vec2(posX, posY));

	float boxSize = playerRadius * 2;
	player->addComponent<CBoundingBox>(Vec2(boxSize, boxSize));

	player->addComponent<CInput>();

	m_player = player;
}

// --------------------------------------------------------------------------------------
// Input. Runs once per frame. Produces: CInput flags (keyboard) and ImGui's event stream.
// --------------------------------------------------------------------------------------
void Game::sUserInput()
{
	auto& input = m_player->getComponent<CInput>().value();
	const ImGuiIO& io = ImGui::GetIO();

	while (const std::optional event = m_window.pollEvent())
	{
		// ImGui must see EVERY event (including releases) or its internal state goes stale.
		ImGui::SFML::ProcessEvent(m_window, *event);

		if (event->is<sf::Event::Closed>())
		{
			quit();
		}
		else if (event->is<sf::Event::FocusLost>())
		{
			// We will never receive the key-release events that happen while unfocused,
			// so clear the flags or the player keeps walking when you alt-tab back.
			input = CInput{};
		}
		else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
		{
			// Typing into an ImGui text box must not steer the player.
			if (io.WantCaptureKeyboard) continue;

			switch (keyPressed->code)
			{
			case sf::Keyboard::Key::W: input.up = true;    break;
			case sf::Keyboard::Key::S: input.down = true;  break;
			case sf::Keyboard::Key::A: input.left = true;  break;
			case sf::Keyboard::Key::D: input.right = true; break;
			case sf::Keyboard::Key::C: m_drawCollision = !m_drawCollision; break;
			case sf::Keyboard::Key::P: m_paused = !m_paused; break;
			case sf::Keyboard::Key::N: if (m_paused) m_stepRequested = true; break;
			case sf::Keyboard::Key::F11:    m_toggleFullscreen = true; break;
			case sf::Keyboard::Key::Escape: if (m_fullscreen) m_toggleFullscreen = true; break;
			default: break;
			}
		}
		else if (const auto* keyReleased = event->getIf<sf::Event::KeyReleased>())
		{
			// Releases are NEVER gated by WantCaptureKeyboard: otherwise pressing W, clicking an
			// ImGui box, then releasing W would leave the flag stuck on forever.
			switch (keyReleased->code)
			{
			case sf::Keyboard::Key::W: input.up = false;    break;
			case sf::Keyboard::Key::S: input.down = false;  break;
			case sf::Keyboard::Key::A: input.left = false;  break;
			case sf::Keyboard::Key::D: input.right = false; break;
			default: break;
			}
		}
	}
}

// --------------------------------------------------------------------------------------
// Map editing. Runs once per frame AFTER the ImGui widgets exist, so WantCaptureMouse is current.
// Reads: mouse, m_tool.  Writes: m_grid, m_start, m_goal.
// Mouse pixel -> world position -> grid cell. (Window == world here, but mapPixelToCoords
// stays correct if a camera/view is added later.)
// --------------------------------------------------------------------------------------
void Game::sEditTools()
{
	// ImGui owns the mouse while it is hovering/dragging an ImGui window.
	// Also ignore the mouse while our window is unfocused (isButtonPressed is global state).
	if (ImGui::GetIO().WantCaptureMouse || !m_window.hasFocus()) return;

	const bool paintButton = sf::Mouse::isButtonPressed(sf::Mouse::Button::Left);
	const bool eraseButton = sf::Mouse::isButtonPressed(sf::Mouse::Button::Right);
	if (!paintButton && !eraseButton) return;

	const sf::Vector2f world = m_window.mapPixelToCoords(sf::Mouse::getPosition(m_window), m_worldView);
	const Cell cell = m_grid.worldToCell(Vec2(world.x, world.y));
	if (!m_grid.inBounds(cell)) return; // mouse dragged outside the window: ignore

	const bool onStart = m_start && *m_start == cell;
	const bool onGoal = m_goal && *m_goal == cell;

	// Right mouse button is always "erase", whatever tool is selected.
	const Tool tool = eraseButton ? Tool::Erase : m_tool;

	switch (tool)
	{
		// The mouse button stays held for many frames. Only flag m_mapChanged when something REALLY
		// changed, otherwise holding the button would reset the search every frame.
	case Tool::Wall:
		// Rule (change it if you disagree): you cannot paint a wall under start or goal.
		if (!onStart && !onGoal && m_grid.get(cell) != Tile::Wall)
		{
			m_grid.set(cell, Tile::Wall);
			m_mapChanged = true;
		}
		break;
	case Tool::Erase:
		if (m_grid.get(cell) != Tile::Open)
		{
			m_grid.set(cell, Tile::Open);
			m_mapChanged = true;
		}
		break;
	case Tool::Start:
		// Rule: start must be on an open cell and must differ from goal.
		if (m_grid.get(cell) == Tile::Open && !onGoal && !onStart)
		{
			m_start = cell;
			m_mapChanged = true;
		}
		break;
	case Tool::Goal:
		if (m_grid.get(cell) == Tile::Open && !onStart && !onGoal)
		{
			m_goal = cell;
			m_mapChanged = true;
		}
		break;
	}
}

// --------------------------------------------------------------------------------------
// ImGui. A VIEW over Game state: widgets read/write members of Game, they own nothing.
// --------------------------------------------------------------------------------------
void Game::sImGui()
{
	ImGui::SetNextWindowPos(ImVec2(10.f, 10.f), ImGuiCond_FirstUseEver);
	ImGui::Begin("Sandbox");

	if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::Checkbox("Paused (P)", &m_paused);

		bool fs = m_fullscreen;
		if (ImGui::Checkbox("Fullscreen (F11)", &fs)) m_toggleFullscreen = true;

		ImGui::BeginDisabled(!m_paused);
		if (ImGui::Button("Step one tick (N)"))
		{
			m_stepRequested = true;
		}
		ImGui::EndDisabled();

		ImGui::Text("FPS: %.1f", static_cast<double>(ImGui::GetIO().Framerate));
		ImGui::Text("Fixed dt: %.5f s (%.0f Hz)",
			static_cast<double>(m_stepper.fixedDt), static_cast<double>(1.0f / m_stepper.fixedDt));
		ImGui::Text("Ticks total: %llu", static_cast<unsigned long long>(m_tickCount));
		ImGui::Text("Ticks last frame: %d", m_lastFrameTicks);
		ImGui::Text("Sim time: %.3f s", m_simTime);

		if (ImGui::Combo("Frame limit", &m_fpsLimitIndex, FrameLimitLabels, IM_ARRAYSIZE(FrameLimitLabels)))
		{
			applyFramerateLimit();
		}
	}

	if (ImGui::CollapsingHeader("Editing", ImGuiTreeNodeFlags_DefaultOpen))
	{
		int tool = static_cast<int>(m_tool);
		ImGui::RadioButton("Wall", &tool, static_cast<int>(Tool::Wall));   ImGui::SameLine();
		ImGui::RadioButton("Erase", &tool, static_cast<int>(Tool::Erase)); ImGui::SameLine();
		ImGui::RadioButton("Start", &tool, static_cast<int>(Tool::Start)); ImGui::SameLine();
		ImGui::RadioButton("Goal", &tool, static_cast<int>(Tool::Goal));
		m_tool = static_cast<Tool>(tool);

		ImGui::TextUnformatted("Left drag: use tool.  Right drag: erase.");

		if (ImGui::Button("Clear walls"))
		{
			m_grid.clearWalls();
			m_mapChanged = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("Clear start/goal"))
		{
			m_start.reset();
			m_goal.reset();
			m_mapChanged = true;
		}

		if (m_start) ImGui::Text("Start: (%d, %d)", m_start->x, m_start->y);
		else         ImGui::TextUnformatted("Start: not set");
		if (m_goal)  ImGui::Text("Goal:  (%d, %d)", m_goal->x, m_goal->y);
		else         ImGui::TextUnformatted("Goal: not set");
	}

	if (ImGui::CollapsingHeader("Search (BFS)", ImGuiTreeNodeFlags_DefaultOpen))
	{
		// The UI only REQUESTS actions (m_searchCommand); sSearch performs them next.
		const bool ready = m_start.has_value() && m_goal.has_value();
		const SearchStatus status = m_search.status();

		ImGui::BeginDisabled(!ready);
		if (ImGui::Button("Start / restart")) m_searchCommand = SearchCommand::Start;
		ImGui::EndDisabled();

		ImGui::SameLine();
		ImGui::BeginDisabled(status != SearchStatus::Running);
		if (ImGui::Button("Step")) m_searchCommand = SearchCommand::Step;
		ImGui::EndDisabled();

		ImGui::SameLine();
		if (ImGui::Button("Reset")) m_searchCommand = SearchCommand::Reset;

		if (!ready) ImGui::TextUnformatted("Place a start and a goal first.");

		ImGui::Checkbox("Auto-run", &m_autoRun);
		ImGui::SliderFloat("Expansions/s", &m_expansionsPerSecond, 1.f, 2000.f, "%.0f",
			ImGuiSliderFlags_Logarithmic);
		ImGui::Checkbox("Show visited/frontier", &m_showSearchOverlay);

		ImGui::Text("Status: %s", toString(status));
		ImGui::Text("Expanded: %d", m_search.expanded());
		ImGui::Text("Frontier: %d (max %d)", m_search.frontierSize(), m_search.maxFrontier());
		if (status == SearchStatus::Found)
		{
			const int cells = static_cast<int>(m_search.path().size());
			ImGui::Text("Path: %d cells (%d moves)", cells, cells - 1);
		}
	}

	if (ImGui::CollapsingHeader("Agent (path follower)", ImGuiTreeNodeFlags_DefaultOpen))
	{
		const bool canRun = m_search.status() == SearchStatus::Found;

		ImGui::BeginDisabled(!canRun);
		if (ImGui::Button("Run agent on path")) m_agentCommand = AgentCommand::Run;
		ImGui::EndDisabled();

		ImGui::SameLine();
		ImGui::BeginDisabled(!m_agent);
		if (ImGui::Button("Remove agent")) m_agentCommand = AgentCommand::Remove;
		ImGui::EndDisabled();

		if (!canRun) ImGui::TextUnformatted("Find a path first (Search section).");

		ImGui::SliderFloat("Agent speed (px/s)", &m_agentSpeed, 20.f, 600.f);

		if (m_agent)
		{
			const auto& f = m_agent->getComponent<CPathFollower>().value();
			const auto& t = m_agent->getComponent<CTransform>().value();

			const char* stateLabel = "Idle (stopped)";
			if (f.state == FollowState::Following) stateLabel = "Following";
			else if (f.state == FollowState::Arrived) stateLabel = "Arrived";

			ImGui::Text("State: %s", stateLabel);
			ImGui::Text("Waypoint: %d / %d", static_cast<int>(f.next), static_cast<int>(f.waypoints.size()));
			ImGui::Text("Pos: (%.1f, %.1f) px", static_cast<double>(t.pos.x), static_cast<double>(t.pos.y));
			ImGui::Text("Speed: %.1f px/s", static_cast<double>(t.velocity.length()));
		}
	}

	if (ImGui::CollapsingHeader("Player", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::SliderFloat("Speed (px/s)", &m_playerSpeed, 0.f, 600.f);

		const auto& t = m_player->getComponent<CTransform>().value();
		ImGui::Text("Pos: (%.1f, %.1f) px", static_cast<double>(t.pos.x), static_cast<double>(t.pos.y));
		ImGui::Text("Vel: (%.1f, %.1f) px/s", static_cast<double>(t.velocity.x), static_cast<double>(t.velocity.y));
		ImGui::Checkbox("Draw collision boxes (C)", &m_drawCollision);
	}

	ImGui::End();
}

// --------------------------------------------------------------------------------------
// Render. Reads state only (grid, transforms). No dt, no simulation here.
// Order: background -> walls -> start/goal -> grid lines -> entities -> ImGui on top.
// --------------------------------------------------------------------------------------
void Game::sRender()
{
	m_window.clear(BackgroundColor);
	m_window.setView(m_worldView);

	// One reusable shape for every cell (no allocation per cell per frame).
	for (int cy = 0; cy < m_grid.rows(); ++cy)
	{
		for (int cx = 0; cx < m_grid.cols(); ++cx)
		{
			const Cell cell{ cx, cy };
			if (m_grid.get(cell) != Tile::Wall) continue;

			const Vec2 tl = m_grid.cellTopLeft(cell);
			m_cellShape.setPosition({ tl.x, tl.y });
			m_cellShape.setFillColor(WallColor);
			m_window.draw(m_cellShape);
		}
	}

	// Search overlay: visited (blue) and frontier (yellow). Walls are never queued, so no overlap.
	if (m_showSearchOverlay && m_search.status() != SearchStatus::Idle)
	{
		for (int cy = 0; cy < m_grid.rows(); ++cy)
		{
			for (int cx = 0; cx < m_grid.cols(); ++cx)
			{
				const Cell cell{ cx, cy };
				const NodeState s = m_search.stateAt(cell);
				if (s == NodeState::Unseen) continue;

				const Vec2 tl = m_grid.cellTopLeft(cell);
				m_cellShape.setPosition({ tl.x, tl.y });
				m_cellShape.setFillColor(s == NodeState::Visited ? VisitedColor : FrontierColor);
				m_window.draw(m_cellShape);
			}
		}
	}

	auto drawMarker = [&](const std::optional<Cell>& cell, const sf::Color& color)
		{
			if (!cell) return;
			const Vec2 tl = m_grid.cellTopLeft(*cell);
			m_cellShape.setPosition({ tl.x, tl.y });
			m_cellShape.setFillColor(color);
			m_window.draw(m_cellShape);
		};
	drawMarker(m_start, StartColor);
	drawMarker(m_goal, GoalColor);

	m_window.draw(m_gridLines);
	m_window.draw(m_pathLine);   // empty unless a path was Found

	for (auto& e : m_entityManager.getEntities())
	{
		if (!e->isActive() || !e->hasComponent<CTransform>() || !e->hasComponent<CShape>()) continue;

		auto& entityCircle = e->getComponent<CShape>().value().circle;
		auto& entityPos = e->getComponent<CTransform>().value().pos;

		entityCircle.setPosition({ entityPos.x, entityPos.y });

		m_window.draw(entityCircle);

		if (m_drawCollision)
		{
			if (e->hasComponent<CBoundingBox>())
			{
				auto& eBox = e->getComponent<CBoundingBox>().value().size;

				sf::RectangleShape box({ eBox.x, eBox.y });

				box.setOrigin({ eBox.x / 2.f, eBox.y / 2.f });
				box.setPosition({ entityPos.x, entityPos.y });
				box.setOutlineColor(sf::Color::Green);
				box.setFillColor(sf::Color::Transparent);
				box.setOutlineThickness(1.f);

				m_window.draw(box);
			}
			else if (e->hasComponent<CBoundingCircle>())
			{
				auto& eCircle = e->getComponent<CBoundingCircle>().value();

				sf::CircleShape circle(eCircle.radius);

				circle.setOrigin({ eCircle.radius, eCircle.radius });
				circle.setPosition({ entityPos.x, entityPos.y });
				circle.setOutlineColor(sf::Color::Green);
				circle.setFillColor(sf::Color::Transparent);
				circle.setOutlineThickness(1.f);

				m_window.draw(circle);
			}
		}
	}

	m_window.setView(m_window.getDefaultView()); // ImGui is screen-space, not world-space
	ImGui::SFML::Render(m_window); // draws what sImGui() built this frame; must be last before display()
	m_window.display();
}

// --------------------------------------------------------------------------------------
// Per-tick systems. dt is ALWAYS the fixed step (1/60 s).
// --------------------------------------------------------------------------------------

// Reads: CInput flags, m_playerSpeed.  Writes: CTransform.velocity (px/s), prevPos, pos (px).
// This is "kinematic" movement: velocity is set directly from input. There is no acceleration
// yet; when steering arrives, velocity += accel * dt goes here BEFORE the position update
// (semi-implicit Euler).
void Game::sMovement(float dt)
{
	auto& t = m_player->getComponent<CTransform>().value();
	const auto& input = m_player->getComponent<CInput>().value();

	const Vec2 dir = movement::directionFromFlags(input.up, input.down, input.left, input.right);

	t.velocity = dir * m_playerSpeed;                              // px/s
	t.prevPos = t.pos;                                             // kept for collision revert / future render interpolation
	t.pos = movement::integratePosition(t.pos, t.velocity, dt);    // px; dt applied exactly once
}

// Reads/Writes: CTransform.pos.
// Changed from "revert to prevPos" to clamp: with a revert the player stops up to one tick of
// distance (4 px at 240 px/s) short of the wall; clamping puts it exactly against the wall.
// To go back to your old behaviour, restore the prevPos revert per axis.
void Game::sCollision()
{
	const float radius = m_player->getComponent<CShape>().value().circle.getRadius();
	auto& t = m_player->getComponent<CTransform>().value();

	const Vec2 ws = worldSize();
	t.pos.x = std::clamp(t.pos.x, radius, ws.x - radius);
	t.pos.y = std::clamp(t.pos.y, radius, ws.y - radius);
}

Vec2 Game::worldSize() const
{
	return Vec2(static_cast<float>(m_grid.cols()) * m_grid.cellSize(),
		static_cast<float>(m_grid.rows()) * m_grid.cellSize());
}

void Game::createWindow()
{
	const Vec2 ws = worldSize();

	if (m_fullscreen)
	{
		// TODO: verify this overload exists in your SFML 3.1 headers:
		//   create(VideoMode, const String& title, State state)
		m_window.create(sf::VideoMode::getDesktopMode(), "Pathfinding Sandbox", sf::State::Fullscreen);
	}
	else
	{
		m_window.create(
			sf::VideoMode({ static_cast<unsigned int>(ws.x), static_cast<unsigned int>(ws.y) }),
			"Pathfinding Sandbox",
			sf::Style::Titlebar | sf::Style::Close);
	}

	applyFramerateLimit();   // create() resets it, so reapply
	updateView();
}

void Game::updateView()
{
	const Vec2 ws = worldSize();
	m_worldView = sf::View(sf::FloatRect({ 0.f, 0.f }, { ws.x, ws.y }));

	// Letterbox: keep the world's aspect ratio, centre it, leave bars on the long axis.
	const sf::Vector2u win = m_window.getSize();
	const float windowRatio = static_cast<float>(win.x) / static_cast<float>(win.y);
	const float worldRatio = ws.x / ws.y;

	float w = 1.f, h = 1.f;   // viewport size as a fraction of the window
	if (windowRatio > worldRatio) w = worldRatio / windowRatio;   // window wider: bars left/right
	else                          h = windowRatio / worldRatio;   // window taller: bars top/bottom

	m_worldView.setViewport(sf::FloatRect({ (1.f - w) * 0.5f, (1.f - h) * 0.5f }, { w, h }));
}

void Game::applyDisplayMode()
{
	// Recreating the window destroys the GL context, which owns ImGui's font texture.
	// So ImGui-SFML must be shut down before and initialised again after.
	ImGui::SFML::Shutdown(m_window);   // TODO: if this overload doesn't compile, use Shutdown()
	createWindow();

	if (!ImGui::SFML::Init(m_window)) { quit(); return; }
	ImGui::GetIO().IniFilename = nullptr;

	// Held keys' release events may be lost across the switch; window creation can also take a while.
	m_player->getComponent<CInput>().value() = CInput{};
	m_clock.restart();
}
// --------------------------------------------------------------------------------------
// Pathfinding system. Runs once per FRAME, driven by real frame time, not by the fixed ticks:
// the 60 Hz ticks exist for physical movement, a search has no physics.
//
// Reads : m_mapChanged, m_searchCommand, m_autoRun, m_expansionsPerSecond, m_start/m_goal, m_grid
// Writes: m_search (the only owner of search state), m_searchAccum, m_pathLine
// --------------------------------------------------------------------------------------
void Game::sSearch(float frameTime)
{
	// 1. Any map edit makes the search (and its overlays) wrong. Throw it away.
	if (m_mapChanged)
	{
		m_search.reset();
		m_searchAccum = 0.f;
		// NOTE: m_mapChanged is cleared once per frame in update(), after sAgentControl AND sSearch.
	}

	// 2. Execute whatever the UI asked for. exchange() reads the command AND clears it,
	//    so a button press is handled exactly once.
	switch (std::exchange(m_searchCommand, SearchCommand::None))
	{
	case SearchCommand::Start:
		if (m_start && m_goal)
		{
			m_search.start(m_grid, *m_start, *m_goal);
			m_searchAccum = 0.f;
		}
		break;
	case SearchCommand::Step:
		m_search.step(m_grid);          // exactly one expansion
		break;
	case SearchCommand::Reset:
		m_search.reset();
		m_searchAccum = 0.f;
		break;
	case SearchCommand::None:
		break;
	}

	// 3. Auto-run: turn real time into "expansions owed". The fractional part carries over, so
	//    1 expansion/s works (it fires once every ~60 frames) and 2000/s does not skip any.
	//    frameTime is clamped so dragging the window cannot dump thousands of steps into one frame.
	if (m_autoRun && m_search.status() == SearchStatus::Running)
	{
		m_searchAccum += std::min(frameTime, 0.25f) * m_expansionsPerSecond;
		const int owed = static_cast<int>(m_searchAccum);
		m_searchAccum -= static_cast<float>(owed);

		for (int i = 0; i < owed && m_search.status() == SearchStatus::Running; ++i)
		{
			m_search.step(m_grid);
		}
	}

	// 4. Keep the drawn path in sync with the search result.
	syncPathLine();
}

// The path only changes when the search finishes or resets, so rebuild the line then,
// not every frame.
void Game::syncPathLine()
{
	if (m_search.status() != SearchStatus::Found)
	{
		if (m_pathLine.getVertexCount() > 0) m_pathLine.clear();
		return;
	}

	if (m_pathLine.getVertexCount() > 0) return; // already built for this result

	for (const Cell& c : m_search.path())
	{
		const Vec2 p = m_grid.cellCenter(c);
		m_pathLine.append(sf::Vertex{ { p.x, p.y }, PathColor });
	}
}

// --------------------------------------------------------------------------------------
// Agent control. Runs once per FRAME (it reacts to edits and UI clicks, it does not move anything).
//
// Reads : m_mapChanged, m_agentCommand, m_agentSpeed, m_search (status + path)
// Writes: m_agent (creates/destroys the entity), the agent's CPathFollower (stop, speed)
// --------------------------------------------------------------------------------------
void Game::sAgentControl()
{
	// 1. A map edit invalidates the route the agent is walking. Decision: freeze it where it
	//    stands and forget the route. (Alternative: let it finish its current leg.)
	if (m_mapChanged && m_agent)
	{
		auto& f = m_agent->getComponent<CPathFollower>().value();
		f.waypoints.clear();
		f.next = 0;
		f.state = FollowState::Idle;
		m_agent->getComponent<CTransform>().value().velocity = Vec2(0.f, 0.f);
	}

	// 2. UI requests. exchange() reads the command AND clears it: a click is handled exactly once.
	switch (std::exchange(m_agentCommand, AgentCommand::None))
	{
	case AgentCommand::Run:    runAgent();    break;
	case AgentCommand::Remove: removeAgent(); break;
	case AgentCommand::None:   break;
	}

	// 3. Live speed tuning: the slider value lives in Game, the component gets a copy.
	if (m_agent)
	{
		m_agent->getComponent<CPathFollower>().value().speed = m_agentSpeed;
	}
}

void Game::runAgent()
{
	// m_mapChanged: an edit happened THIS frame but sSearch has not reset the search yet,
	// so m_search may still say Found for a path that no longer exists. Refuse.
	if (m_mapChanged || m_search.status() != SearchStatus::Found) return;

	const std::vector<Cell>& path = m_search.path();
	if (path.empty()) return;

	removeAgent(); // one agent for now: running again replaces it

	// Cells -> world-space waypoints (cell CENTRES). After this the follower knows nothing about the grid.
	std::vector<Vec2> waypoints;
	waypoints.reserve(path.size());
	for (const Cell& c : path)
	{
		waypoints.push_back(m_grid.cellCenter(c));
	}

	const float radius = m_grid.cellSize() * 0.35f; // clearly smaller than a cell: fits inside any corridor of width 1

	auto agent = m_entityManager.addEntity("agent");
	agent->addComponent<CShape>(radius, 16, sf::Color(255, 140, 0), sf::Color::White, 2.f);
	agent->addComponent<CTransform>(waypoints.front());       // spawn ON the start cell
	agent->addComponent<CBoundingCircle>(radius);

	// NOTE: no CInput on purpose (control ownership: this entity is moved by sPathFollow only).
	auto& follower = agent->addComponent<CPathFollower>(m_agentSpeed);
	follower.waypoints = std::move(waypoints);
	follower.next = 1;                                         // index 0 is where we already stand
	follower.state = (follower.next < follower.waypoints.size()) ? FollowState::Following
		: FollowState::Arrived; // start == goal

	m_agent = agent;
}

void Game::removeAgent()
{
	if (!m_agent) return;

	m_agent->destroy();   // actually removed from the entity list at the next m_entityManager.update()
	m_agent.reset();
}

// --------------------------------------------------------------------------------------
// Path following. Runs once per fixed TICK, always with the fixed dt.
//
// Reads : CPathFollower (waypoints, next, speed, state), CTransform.pos
// Writes: CTransform.pos / prevPos / velocity, CPathFollower.next / state
//
// Distance budget for this tick = speed (px/s) * dt (s). dt is applied exactly once, here.
// pathfollow::advance() spends that budget along the waypoints (see PathFollow.h).
// --------------------------------------------------------------------------------------
void Game::sPathFollow(float dt)
{
	for (auto& e : m_entityManager.getEntities())
	{
		if (!e->isActive()) continue; // destroyed this frame, removed at the next update()
		if (!e->hasComponent<CPathFollower>() || !e->hasComponent<CTransform>()) continue;

		// Control ownership: keyboard-driven entities are moved by sMovement only.
		if (e->hasComponent<CInput>()) continue;

		auto& f = e->getComponent<CPathFollower>().value();
		auto& t = e->getComponent<CTransform>().value();

		t.prevPos = t.pos; // kept for future render interpolation, same as sMovement

		if (f.state != FollowState::Following)
		{
			t.velocity = Vec2(0.f, 0.f);
			continue;
		}

		const pathfollow::StepResult r = pathfollow::advance(t.pos, f.waypoints, f.next, f.speed * dt);

		t.pos = r.pos;
		t.velocity = r.dir * f.speed;   // px/s; direction of the last leg travelled (for display / later steering)
		f.next = r.next;

		if (r.arrived)
		{
			f.state = FollowState::Arrived;
			t.velocity = Vec2(0.f, 0.f);
		}
	}
}