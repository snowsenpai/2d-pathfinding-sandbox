#pragma once

#include "Vec2.h"

#include <cstdint>
#include <vector>

// Plain data + coordinate conversion. No SFML, no ImGui, no entities.
// Owned by Game (it is map data, not an entity).
//
// Two coordinate spaces, never mix them silently:
//   world space : pixels, float, origin top-left, +y down   (Vec2)
//   cell space  : integer column/row                         (Cell)
//
//   worldToCell(p) = floor(p / cellSize)     cellCenter(c) = (c + 0.5) * cellSize

enum class Tile : std::uint8_t
{
	Open,   // walkable, costs TerrainCosts::open to enter
	Wall,   // NOT walkable, ever
	Mud     // walkable, costs TerrainCosts::mud to enter
};

// Cost of ENTERING a tile of each kind (a multiplier on distance: 1 = normal).
// Public data on purpose: ImGui edits it, and the search reads it through Grid::moveCost().
struct TerrainCosts
{
	float open = 1.0f;
	float mud = 3.0f;
};

struct Cell
{
	int x = 0; // column
	int y = 0; // row

	bool operator==(const Cell& rhs) const { return x == rhs.x && y == rhs.y; }
	bool operator!=(const Cell& rhs) const { return !(*this == rhs); }
};

class Grid
{
	int m_cols;
	int m_rows;
	float m_cellSize; // pixels per cell edge
	std::vector<Tile> m_tiles; // row-major: index = y * cols + x

	int index(Cell c) const; // precondition: inBounds(c)

public:
	Grid(int cols, int rows, float cellSize);

	TerrainCosts costs;   // edit freely; moveCost() clamps to a safe minimum

	int   cols() const;
	int   rows() const;
	float cellSize() const;

	bool inBounds(Cell c) const;

	// Out-of-bounds reads return Wall: "outside the map is blocked".
	// That is the convenient answer for neighbour expansion in BFS/A* later.
	Tile get(Cell c) const;

	// Out-of-bounds writes are ignored (so a mouse outside the window can never crash).
	void set(Cell c, Tile t);
	void clearWalls();   // resets EVERY tile to Open (walls and mud); name kept for compatibility

	// Walkable = inside the map and not a Wall. Mud IS walkable.
	bool isWalkable(Cell c) const;

	// Cost of entering cell c (see TerrainCosts). +infinity if c is out of bounds or a Wall.
	// Always >= MinMoveCost for walkable cells: Dijkstra needs strictly positive costs,
	// so a slider dragged to 0 (or below) can never break the search.
	float moveCost(Cell c) const;
	static constexpr float MinMoveCost = 0.01f;

	Cell worldToCell(const Vec2& p) const; // uses floor, NOT truncation (so -5px -> cell -1)
	Vec2 cellTopLeft(Cell c) const;        // pixels
	Vec2 cellCenter(Cell c) const;         // pixels
};
