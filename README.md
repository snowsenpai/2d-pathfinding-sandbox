# 2D Pathfinding Sandbox

A C++17 / SFML 3 / Dear ImGui sandbox for implementing, visualizing and comparing grid-based pathfinding algorithms, and for driving a 2D game entity along the routes they produce.

## 1. Project Overview

The sandbox is a learning and experimentation project. It provides a small interactive environment in which a grid map can be edited, a search can be run one expansion at a time or automatically, and the resulting path can be handed to an agent that walks it inside a fixed-timestep simulation.

The project has the following goals:

- Understand BFS, Dijkstra's algorithm and A*, and the trade-offs between them.
- Explore grid navigation: obstacles, weighted terrain, 4-way and 8-way movement, and diagonal corner rules.
- Make search behaviour visible (frontier, visited cells, cost heatmap, final path) and measurable (expansions, frontier size, path cost).
- Connect a computed route to entity movement while keeping route calculation and route following as separate responsibilities.
- Practice separating algorithms, simulation and rendering so the pieces can be reused in other C++/SFML projects.

This is a learning project. It is not a production navigation system or a game engine.

## 2. Features

### Grid-Based Map

- A `Grid` of 40 x 30 cells at 20 px per cell (an 800 x 600 world), created in `Game.h`. The grid is plain data with no SFML dependency.
- Three tile types: `Open`, `Wall` (never walkable) and `Mud` (walkable, more expensive to enter).
- Terrain costs are the cost of *entering* a tile: open = 1.0, mud = 3.0 by default. The mud cost is adjustable at runtime (1 to 10) from the UI. The open cost is fixed at 1.0 in the UI.
- Costs are clamped to a small positive minimum (`Grid::MinMoveCost = 0.01`) so a slider dragged to zero cannot break Dijkstra or A*.
- Out-of-bounds reads are treated as walls, and out-of-bounds writes are ignored.
- Coordinate conversion between world space (pixels, origin top-left, +y down) and cell space (integer column/row), using `floor` so negative pixel positions map to negative cells.
- Start and goal cells are stored on `Game`, not on the grid.
- 4-way or 8-way movement is selectable. A diagonal step costs `sqrt(2)` times the terrain cost of the cell being entered.
- By default a diagonal move requires both orthogonal neighbours to be walkable (no squeezing between wall corners). "Allow corner cutting" removes this restriction.

### Pathfinding Algorithms

All three algorithms live in `PathSearch` and are resumable: a search is seeded with `start()` and advanced one expansion per `step()` call. The code uses no SFML, ImGui or entity types.

| Algorithm | Strategy | Weighted costs | Strengths | Limitations |
|---|---|---|---|---|
| BFS | FIFO queue; expands cells in order of step count | Ignored while searching; the true cost of the found path is still reported | Fewest moves; simple; useful baseline | Can return an expensive path on weighted terrain |
| Dijkstra | Binary min-heap ordered by cost-so-far `g` | Yes | Cheapest path on weighted terrain, with no heuristic needed | Expands a disc around the start, so many cells on open maps |
| A* | Same code path as Dijkstra, ordered by `f = g + weight * scale * h(cell, goal)` | Yes | Same optimal cost as Dijkstra with an admissible heuristic, usually far fewer expansions | Optimality depends on the heuristic matching the movement rules; weight > 1 gives up optimality |

Dijkstra is implemented as A* with the `Zero` heuristic. Stale heap entries are skipped lazily when popped, because a binary heap cannot decrease a key. The goal test happens when a cell is popped, not when it is pushed.

Implemented heuristics (`Heuristic.h`): `Zero`, `Manhattan`, `Euclidean`, `Chebyshev` and `Octile`. A* options also include a weight (1 to 10 in the UI) and an optional tie-break that prefers the larger `g` among equal `f` values. The heuristic is scaled by the cheapest terrain cost so it stays in the same units as real costs.

### Search Visualization

- Start / restart a search, advance it one expansion at a time (Step), or enable Auto-run.
- Auto-run speed is set in expansions per second (1 to 2000, logarithmic slider). It runs on real frame time with a fractional carry, independent of the 60 Hz simulation ticks.
- Cells are drawn by state: visited (blue), frontier (yellow), start (green), goal (red). The final path is drawn as a line strip through cell centres.
- An optional cost heatmap colours visited cells by their cost relative to the largest visited cost. It is drawn as part of the visited/frontier overlay, so that overlay must be enabled.
- Status panel: search status, cells expanded, current and maximum frontier size, and, when a path is found, its length in cells and moves and its total cost.
- Any map edit (walls, mud, start, goal, mud cost, clear) or search-option change resets the search, clears overlays and discards the comparison table.

### Algorithm Comparison

"Compare all" runs BFS, Dijkstra and A* to completion on private `PathSearch` objects, using the current start, goal and options (only the algorithm differs). The search shown on screen is not affected. The results table reports, per algorithm:

| Column | Meaning |
|---|---|
| Result | Final search status (Found / No path) |
| Expanded | Cells expanded: a deterministic measure of algorithmic work |
| Max frontier | Largest number of cells waiting to be expanded at once (a memory-use indicator) |
| Cost | Total cost of the path found |
| Time (us) | Wall-clock time of the run, measured with `std::chrono::steady_clock` |

Expansion counts and frontier size are reproducible for a given map and options. The wall-clock time is noisy on a grid this small and should be read as a rough indication only. The table does not currently display path length in cells (the value is recorded internally; the main search panel shows it for the on-screen search).

### Agent Path Following

- "Run agent on path" is available once a search has reached `Found`. It spawns an agent entity on the start cell and sends it along the found path.
- The path's cells are converted to world-space waypoints at cell centres. After that conversion the follower knows nothing about the grid.
- Movement uses a distance budget per tick (`speed * dt`) spent leg by leg: leftover distance carries into the next leg, the agent lands exactly on each waypoint, and it cannot overshoot the end.
- Agent speed is adjustable (20 to 600 px/s) and applies live.
- The follower has three states: `Idle`, `Following` and `Arrived`. The UI shows the state, waypoint progress, position and speed.
- There is one agent. Running again replaces it, and "Remove agent" destroys it. An agent that has arrived stays in place until removed or replaced.
- A map edit freezes the agent where it stands and discards its route (the route is invalid). Running the agent is refused while the search result is stale.
- Only waypoint following is implemented. There are no steering behaviours (seek, arrive, avoidance, separation) and no path smoothing.

Pathfinding and path following are separate responsibilities: `PathSearch` computes a route over the grid, and `pathfollow::advance` moves a position along a list of points.

### Simulation and Debugging

- Fixed-timestep simulation at 60 Hz with an accumulator, spike clamp (0.25 s) and a cap of 5 ticks per frame. When the cap is hit the backlog is dropped, so the simulation slows down instead of spiralling.
- Player and agent movement run in fixed ticks, so their speed in px/s does not depend on frame rate.
- Pause, and step a single tick while paused. While paused the accumulator is not fed, so unpausing does not catch up. Pausing affects the player and agent but not the search, which is frame-driven.
- Frame-rate limit selection: 30, 60 (default), 144 or Unlimited.
- ImGui panels for simulation, editing, search, agent and player, with live readouts (FPS, fixed dt, total ticks, ticks last frame, simulated time, positions, velocities).
- Optional collision-shape overlay for entities (bounding box for the player, bounding circle for the agent).
- Fullscreen toggle. The window is created non-resizable at 800 x 600; in fullscreen the world is letterboxed to preserve its aspect ratio and ImGui is re-initialized after the window is recreated.

A keyboard-controlled player circle (spawned at a random position) exists for testing the simulation loop. It is clamped to the world bounds and does not collide with grid walls.

## 3. Technologies

| Component | Version / source | Role |
|---|---|---|
| C++17 | `CMAKE_CXX_STANDARD 17` | Language |
| SFML | 3.1.0 (`FetchContent`, git tag `3.1.0`) | Window, input, rendering, timing |
| Dear ImGui | v1.91.9b (`FetchContent`) | Runtime controls and debug UI |
| ImGui-SFML | v3.0 (`FetchContent`) | ImGui / SFML integration |
| CMake | 3.28 or newer | Configuration, dependency fetching, build, CTest registration |

All dependencies are fetched at configure time with CMake `FetchContent`. The version pairing of ImGui, ImGui-SFML and SFML 3.1.0 is declared in `CMakeLists.txt`; if configuration or compilation fails after a dependency change, check which ImGui version the chosen ImGui-SFML release expects.

Testing uses a small self-contained test executable (`sandbox_tests`) with its own `CHECK` macro, registered with CTest. It links no SFML or ImGui code.

## 4. Project Architecture

The sandbox follows a layered structure: pure logic headers and classes with no SFML dependency (grid, search, heuristics, path-following math, movement math, fixed stepping), a small entity-component layer, and a `Game` class that owns the application loop, UI and rendering.

**Grid** (`Grid.h/.cpp`). Map data and coordinate conversion: tile storage, walkability, entry cost (`moveCost`), the cheapest terrain cost (`minMoveCost`, used to scale heuristics), and world/cell conversion. It is owned by `Game` as data, not as an entity.

**PathSearch** (`PathSearch.h/.cpp`). Owns all search state: per-cell node state, parent link and best cost, the BFS queue or best-first heap, counters (expanded, frontier, maximum frontier), the reconstructed path and its cost. Algorithm selection comes from `SearchOptions`. Path reconstruction walks parent links from goal to start. The search asks the map only four questions (size, in-bounds, walkable, move cost), so another tile map could replace `Grid`.

**Heuristic** (`Heuristic.h`). Pure functions for `Zero`, `Manhattan`, `Euclidean`, `Chebyshev` and `Octile`, measured in cells. The header documents which are admissible for 4-way and 8-way movement (Manhattan is not admissible with diagonals).

**Movement and PathFollow** (`Movement.h`, `PathFollow.h`). `movement::` converts key flags to a normalized direction and integrates position. `pathfollow::advance` spends a distance budget along a list of world-space waypoints and reports the new position, next waypoint index, last direction and whether the route ended. Both are pure functions with no entity or SFML types.

**FixedStep** (`FixedStep.h`). `FixedStepper::advance(frameTime)` turns real frame time into the number of fixed ticks to run this frame (0, 1, 2, ...). Rendering and ImGui run once per frame regardless. An `alpha()` helper for render interpolation exists but is currently unused.

**Components, Entity and EntityManager.** Components are plain data: `CTransform`, `CBoundingBox`, `CBoundingCircle`, `CShape`, `CInput` and `CPathFollower`. An `Entity` stores each component type as a `std::optional` in a tuple. `EntityManager` creates entities (added at the next `update()`), keeps a tag lookup, and removes inactive entities. Control ownership is explicit: an entity is moved either by `CInput` (player, via `sMovement`) or by `CPathFollower` (agent, via `sPathFollow`), never both. `sPathFollow` skips any entity that has a `CInput`, and the agent is created without one.

**Game** (`Game.h/.cpp`). Application lifecycle, input handling, map editing, search execution, agent control, fixed-step simulation, ImGui panels and rendering. Systems are split into per-frame systems (`sUserInput`, `sImGui`, `sEditTools`, `sAgentControl`, `sSearch`, `sRender`) and per-tick systems (`sMovement`, `sPathFollow`, `sCollision`). The UI only *requests* search and agent actions by setting command flags; the corresponding system executes them.

### System Interaction

1. **Edit.** `sEditTools` paints walls or mud and places the start and goal, setting a map-changed flag only when something actually changed.
2. **Configure and search.** The ImGui panel edits `SearchOptions` and issues Start, Step, Reset or Compare commands. `sSearch` invalidates stale results first, then executes the command and any auto-run expansions owed from real frame time.
3. **Visualize.** `sRender` draws terrain, the visited/frontier overlay (optionally as a heatmap), start/goal, grid lines and the cached path line, which is rebuilt only when a search finishes.
4. **Convert.** On request, `Game` converts the found path's cells to cell-centre waypoints and spawns an agent with a `CPathFollower`.
5. **Simulate.** For each fixed tick, `sPathFollow` calls `pathfollow::advance` with `speed * fixedDt` and updates the agent's transform and follower state.

Per frame the order is: input, ImGui frame, UI widgets, edit tools, agent control, search, then N fixed ticks, then render. Search progress is driven by frame time and its own accumulator, entity movement by fixed ticks, and drawing happens once per frame.

### Directory Structure

```
.
├── CMakeLists.txt
└── src/
    ├── main.cpp
    ├── Game.h / Game.cpp
    ├── Grid.h / Grid.cpp
    ├── PathSearch.h / PathSearch.cpp
    ├── Heuristic.h
    ├── Movement.h
    ├── PathFollow.h
    ├── FixedStep.h
    ├── Components.h
    ├── Entity.h / Entity.cpp
    ├── EntityManager.h / EntityManager.cpp
    ├── Vec2.h / Vec2.cpp
    ├── MathUtils.h
    └── Tests.cpp
```

## 5. Building and Running

### Prerequisites

- A C++17 compiler (developed on Windows with Visual Studio and MSVC).
- CMake 3.28 or newer.
- Git, because SFML, Dear ImGui and ImGui-SFML are fetched during configuration (network access is required on first configure).
- On Linux, the system development packages SFML needs for windowing and OpenGL. See the SFML documentation for the list.

### Configuring

```bash
cmake -S . -B build
```

With Visual Studio you can instead open the folder directly; `pathfinding_sandbox` is set as the startup project.

### Building

```bash
cmake --build build --config Release --target pathfinding_sandbox
```

The executable target is `pathfinding_sandbox`. The `--config` option is only used by multi-configuration generators (Visual Studio, Xcode); single-configuration generators select the build type at configure time (`-DCMAKE_BUILD_TYPE=Release`).

### Running

The runtime output directory is `<build>/bin`. Multi-configuration generators typically append the configuration name.

```bash
# single-configuration generators (Makefiles, Ninja)
./build/bin/pathfinding_sandbox

# Visual Studio (multi-configuration)
build\bin\Release\pathfinding_sandbox.exe
```

### Running Tests

The unit-test target is `sandbox_tests`, registered with CTest as `sandbox_tests`. It does not link SFML or ImGui.

```bash
cmake --build build --config Debug --target sandbox_tests
ctest --test-dir build -C Debug --output-on-failure
```

The executable can also be run directly from `build/bin`; it exits with code 0 when all checks pass and prints a failure line (file, line, expression) for each failed check.

## 6. Controls

### Keyboard and Mouse

Keyboard shortcuts are ignored while an ImGui widget wants keyboard input. Mouse editing is ignored while the mouse is over an ImGui window or the application window is unfocused.

| Input | Action |
|---|---|
| `W` `A` `S` `D` | Move the player |
| `C` | Toggle collision-shape overlay |
| `P` | Pause / resume simulation ticks |
| `N` | Advance one tick (only while paused) |
| `F11` | Toggle fullscreen |
| `Esc` | Leave fullscreen (does nothing in windowed mode) |
| Left mouse button (hold/drag) | Apply the selected tool to the cell under the cursor |
| Right mouse button (hold/drag) | Erase (always, regardless of the selected tool) |

Tool behaviour:

| Tool | Behaviour |
|---|---|
| Wall | Places walls; cannot be painted onto the start or goal cell |
| Erase | Resets the cell to open terrain (removes walls and mud) |
| Start | Sets the start cell; the cell must be walkable and not the goal |
| Goal | Sets the goal cell; the cell must be walkable and not the start |
| Mud | Places mud; replaces a wall if painted over one |

### Dear ImGui Panel ("Sandbox")

| Section | Control | Effect |
|---|---|---|
| Simulation | Paused | Pause or resume fixed ticks (also `P`) |
| | Fullscreen | Toggle fullscreen (also `F11`) |
| | Step one tick | One tick while paused (also `N`) |
| | Frame limit | 30 / 60 / 144 / Unlimited |
| | Readouts | FPS, fixed dt and rate, total ticks, ticks last frame, simulated time |
| Editing | Tool radio buttons | Wall, Erase, Start, Goal, Mud |
| | Mud cost slider | Mud entry cost, 1 to 10 |
| | Clear terrain | Resets all tiles to open |
| | Clear start/goal | Unsets start and goal |
| Search | Start / restart | Begin a search with the current options (requires start and goal) |
| | Step | One expansion (enabled while a search is running) |
| | Reset | Back to idle, clears overlays |
| | Compare all | Run BFS, Dijkstra and A* and fill the results table |
| | Algorithm | BFS, Dijkstra or A* |
| | Heuristic | A* only: Zero, Manhattan, Euclidean, Chebyshev, Octile |
| | Weight | A* only: 1 to 10 |
| | Tie-break: prefer larger g | A* only |
| | 8-way movement | Toggle 4-way / 8-way |
| | Allow corner cutting | 8-way only |
| | Auto-run, Expansions/s | Automatic stepping at 1 to 2000 expansions per second |
| | Show visited/frontier | Toggle search overlay |
| | Cost heatmap | Colour visited cells by cost |
| Agent | Run agent on path | Spawn agent and follow the found path (requires a Found result) |
| | Remove agent | Destroy the agent |
| | Agent speed | 20 to 600 px/s |
| Player | Speed | 0 to 600 px/s |
| | Draw collision boxes | Same as `C` |

There are no keyboard shortcuts for search or agent control; those are available only through the ImGui panel.

## 7. Pathfinding Concepts Explored

**Grid-based navigation.** Each cell has up to four (orthogonal) or eight (with diagonals) neighbours. Walls are not traversable; open and mud cells are, at different entry costs. A diagonal costs `sqrt(2)` times the terrain cost, and the corner rule decides whether a diagonal may pass between two blocked cells.

**Graph search.** The grid is treated implicitly as a graph in which cells are nodes and legal moves are edges. A search keeps a frontier (discovered, waiting) and a visited set (expanded, cost final), and reconstructs the path from parent links once the goal is popped. BFS uses a FIFO queue and minimizes step count; Dijkstra and A* use a priority queue and minimize cost.

**Heuristics.** A heuristic estimates the remaining cost to the goal. If it never overestimates (admissible), A* returns an optimal path. Which heuristics are admissible depends on the movement rules: Manhattan is exact for 4-way movement but overestimates diagonals in 8-way movement, while Octile is exact on an open 8-way field. Weighted A* multiplies the estimate by a factor above 1 to search faster, at the cost of a path up to that factor worse than optimal. The UI warns when a chosen combination can lose optimality.

**Path quality and search efficiency.** Path cost and path length are different measures. BFS finds the fewest moves but may cross expensive terrain, while Dijkstra and A* minimize total cost. Nodes expanded and maximum frontier size describe how much work and memory a search used, independent of machine speed.

**Movement and simulation.** Search works in cell coordinates, while entities move in world pixels, so a path is converted to waypoints at cell centres. Movement is time-based (`speed * dt`) and advanced in fixed ticks, so simulation behaviour does not depend on the rendering frame rate.

## 8. Testing and Validation

`Tests.cpp` builds into `sandbox_tests`. It links only `Vec2.cpp`, `Grid.cpp` and `PathSearch.cpp` and includes the header-only logic, so it builds without SFML or ImGui. Each check reports file and line on failure, and the process returns a non-zero exit code if any check fails. CTest registers it as a single test, `sandbox_tests`.

Covered by the existing tests:

| Area | Scenarios |
|---|---|
| Vec2 | Zero-vector normalization (no NaN), length, `limited`, dot product |
| Movement | Zero-velocity integration; 1 second of movement at 30 / 60 / 144 FPS; diagonal input normalization and opposite-key cancellation |
| FixedStepper | Tick counts for various frame times; remainder carried between frames; spike clamp, tick cap and backlog drop; negative frame time ignored |
| Grid | World-to-cell conversion (origin, last cell, past the edge, negative coordinates), cell centres, out-of-bounds read/write, `clearWalls`, walkability, `moveCost` for open/mud/wall/out-of-bounds, clamping of zero or negative costs, `minMoveCost` |
| BFS | Open-field path length and expansion count, start equals goal, wall with a gap, unreachable goal, invalid start/goal (wall or out of bounds), stepping and reset, determinism, reuse of one search object |
| Path following | Partial move, exact arrival and overshoot clamping, corner carry-over, empty route, zero/negative distance, duplicate waypoints, end-to-end walk time at 60 Hz and 30 Hz, landing exactly on the goal centre, no wall clipping when sampled finely |
| Dijkstra | 4-way and 8-way open-field costs, mud versus detour decisions, BFS paying for mud, corner cutting on/off, unreachable goal, mud start cell, random-map property test against BFS (agreement on reachability, no cell expanded twice, cheapest-first order, valid paths, recomputed cost matches reported cost, Dijkstra never costlier than BFS, BFS never longer in steps) |
| Heuristics | Reference values, ordering `Chebyshev <= Euclidean <= Octile <= Manhattan`, symmetry, zero at the goal |
| A* | `Zero` heuristic equals Dijkstra (status, expansions, path), Octile optimal on an open 8-way field with fewer expansions, tie-break on/off, mud-strip costs, deliberate Manhattan overestimate on an 8-way map, weighted A* cost bounds, random-map property test that admissible heuristics match Dijkstra's cost |

Not covered by automated tests: `Game` (input handling, editing rules, UI), the entity-component layer, rendering, fullscreen handling and the comparison table. These are exercised manually through the sandbox. The automated tests verify algorithm and math correctness, while the visual sandbox is for observing behaviour, such as how the frontier grows or how different heuristics change expansion patterns.

## 9. Engineering Notes

- **Pathfinding data is separate from entity components.** Search state lives in `PathSearch`, and the agent's `CPathFollower` holds only world-space waypoints, so the follower has no grid dependency and can follow any route source.
- **Logic is independent of SFML.** Grid, search, heuristics, movement math, path-following math and fixed stepping have no rendering dependencies, which is why the test executable links neither SFML nor ImGui.
- **ImGui is a view, not an owner.** Widgets read and write members of `Game` and request search or agent actions through command flags; the systems execute them once per frame.
- **Control ownership.** An entity is driven by keyboard input or by a path follower, never both, which avoids two systems competing for velocity.
- **Per-frame versus per-tick systems.** Input, UI, editing, search and rendering run once per frame. Movement, path following and collision run in fixed ticks with a constant `dt`.
- **Invalidation.** Map edits and search-option changes reset the search and comparison results, and map edits also stop the agent. Starting an agent is refused if the result is stale.
- **Reusable rendering data.** The grid lines are built once, a single `RectangleShape` is reused for all cell drawing, and the path line strip is rebuilt only when a search finishes.
- **Defensive search inputs.** Invalid start/goal cells make `start()` return false without changing state, `step()` is a no-op unless running, and heap ordering is a strict weak ordering with deterministic tie-breaking.

## 10. Learning Resources

- **Understanding Goal-Based Vector Field Pathfinding** (Game Development / TutsPlus): background reading on flow-field style navigation as an alternative to per-agent path search. Vector-field pathfinding is **not** implemented in this sandbox.
  https://gamedevelopment.tutsplus.com/tutorials/understanding-goal-based-vector-field-pathfinding--gamedev-9007

- **OpenSteer**: a steering-behaviour library, consulted as a reference for how steering is structured. It is **not** a dependency of this project, and steering behaviours are not implemented here.
  https://opensteer.sourceforge.net/

- **Dave Churchill, COMP4300: Game Programming**, pathfinding and steering lecture: course material that informed the pathfinding side of the project and its separation of simulation concerns. Only the pathfinding and waypoint-following portions are reflected in the code.
  https://www.youtube.com/live/9RYnrO4XwO8

## 11. Future Improvements

These are possible extensions, not a committed roadmap.

- Replanning when the map changes, instead of freezing the agent.
- Path smoothing (for example line-of-sight shortcuts) on top of the grid path.
- Additional algorithms, such as jump point search or bidirectional search, and more terrain types.
- Display path length in the comparison table and add repeated-run timing to reduce timing noise.
- Multiple agents with local avoidance or separation.
- Steering behaviours (seek, arrive) as an alternative to waypoint snapping, using the already stored previous position for render interpolation.
- Flow-field pathfinding as a comparison against per-agent search.
- Unit tests for the entity layer and for editing rules, and larger benchmark maps.