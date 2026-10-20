# SFML ECS Game Template

A lightweight C++ game starter built with [SFML](https://www.sfml-dev.org/) and a simple Entity-Component-System (ECS) architecture. This repository is designed as a minimal but practical foundation for 2D games, demos, and experiments using CMake.

The project includes:

- A small ECS layer with `Entity`, `EntityManager`, and component data containers
- SFML rendering and window management
- Player input handling with `WASD`
- Basic movement and boundary collision logic
- Debug collision overlay toggled with `C`
- CMake-based build setup with automatic SFML fetching via `FetchContent`

## Features

- C++17 project configuration
- SFML 3.1.0 integration via CMake
- Simple component-driven design for gameplay logic
- Clean separation between entities, components, and systems
- Cross-platform build support through CMake

## Project Structure

```text
.
├── CMakeLists.txt
├── LICENSE.md
├── README.md
├── src/
│   ├── Components.h
│   ├── Entity.h
│   ├── Entity.cpp
│   ├── EntityManager.h
│   ├── EntityManager.cpp
│   ├── Game.h
│   ├── Game.cpp
│   ├── Vec2.h
│   ├── Vec2.cpp
│   ├── main.cpp
│   └── ...
└── build/
```

### Core concepts

- `Entity`: Represents an object in the game world and owns its components.
- `EntityManager`: Creates, updates, and tracks entities.
- `Components.h`: Contains reusable ECS data like transforms, input state, shapes, and colliders.
- `Game.*`: Contains the main game loop, rendering, player input, movement, and collision logic.
- `Vec2.h/.cpp`: A lightweight 2D vector helper used across the project.

## Getting Started

### Prerequisites

- CMake 3.28 or newer
- A modern C++ compiler with C++17 support
- Git (optional, for cloning the repository)

### Clone the repository

```bash
git clone https://github.com/snowsenpai/sfml_ecs_game_template.git
cd sfml_ecs_game_template
```

### Configure and build

```bash
cmake -B build
cmake --build build
```

### Run the game

On Linux/macOS:

```bash
./build/bin/ecs_template
```

On Windows, run the generated executable from the build output directory, typically:

```powershell
build\bin\ecs_template.exe
```

## Controls

- `W` / `A` / `S` / `D`: move the player
- `C`: toggle collision debug visualization
- Close window button: exit the game

## How it works

This repository demonstrates a minimal ECS pattern:

- Entities are created through `EntityManager`
- Components store data such as transform, bounds, color, and input state
- Systems are implemented in `Game` as methods like `sMovement()`, `sCollision()`, and `sPlayerInput()`
- Rendering is done with SFML shapes and a simple window loop

This keeps the code easy to extend as you grow the game with new entities, systems, and gameplay rules.

## Build Notes

The project uses CMake's `FetchContent` to download and build SFML automatically from GitHub, so you do not need to install SFML manually. The configuration is set to use SFML 3.1.0.

## License

This project is dual-licensed under the Public Domain and MIT licenses. See [LICENSE.md](LICENSE.md) for details.

## Future Ideas

This template is intentionally simple and can be expanded with:

- enemy AI and state machines
- sprite-based rendering
- projectiles and shooting logic
- menu and UI systems
- audio and sound effects
- scenes and level management

If you want to use this as the basis for a larger game, the current ECS and game loop structure makes a good starting point.
