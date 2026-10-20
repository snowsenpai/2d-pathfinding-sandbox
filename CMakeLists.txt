cmake_minimum_required(VERSION 3.28)
project(ecs_template LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.1.0
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

add_executable(ecs_template
    src/main.cpp
    src/Vec2.h
    src/Vec2.cpp
    src/Components.h
    src/Entity.h
    src/Entity.cpp
    src/EntityManager.h
    src/EntityManager.cpp
    src/Game.h
    src/Game.cpp)

target_compile_features(ecs_template PRIVATE cxx_std_17)
target_link_libraries(ecs_template PRIVATE SFML::Graphics)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ecs_template)
