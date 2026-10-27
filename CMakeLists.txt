cmake_minimum_required(VERSION 3.28)
project(pathfinding_sandbox LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ---- SFML (must come BEFORE imgui-sfml) ----
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.1.0
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

# ---- Dear ImGui + ImGui-SFML ----
# Tags below (imgui v1.91.9b + imgui-sfml v3.0) are what a public SFML 3 project uses; I have NOT
# verified them against SFML 3.1.0 myself. If configure/compile fails, check the ImGui-SFML README
# for the imgui version it expects.
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.9b
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(imgui)

set(IMGUI_DIR ${imgui_SOURCE_DIR})
set(IMGUI_SFML_FIND_SFML OFF)   # we already provide SFML above

FetchContent_Declare(imgui-sfml
    GIT_REPOSITORY https://github.com/SFML/imgui-sfml.git
    GIT_TAG v3.0
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(imgui-sfml)

# ---- the sandbox ----
add_executable(pathfinding_sandbox
    src/main.cpp
    src/MathUtils.h
    src/Vec2.h
    src/Vec2.cpp
    src/Movement.h
    src/PathFollow.h
    src/FixedStep.h
    src/Grid.h
    src/Grid.cpp
    src/Heuristic.h
    src/PathSearch.h
    src/PathSearch.cpp
    src/Components.h
    src/Entity.h
    src/Entity.cpp
    src/EntityManager.h
    src/EntityManager.cpp
    src/Game.h
    src/Game.cpp)

target_compile_features(pathfinding_sandbox PRIVATE cxx_std_17)
target_link_libraries(pathfinding_sandbox PRIVATE SFML::Graphics ImGui-SFML::ImGui-SFML)

# ---- unit tests: no SFML, no ImGui, builds in seconds ----
add_executable(sandbox_tests
    src/Tests.cpp
    src/Vec2.cpp
    src/Grid.cpp
    src/PathSearch.cpp)
target_compile_features(sandbox_tests PRIVATE cxx_std_17)

enable_testing()
add_test(NAME sandbox_tests COMMAND sandbox_tests)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT pathfinding_sandbox)