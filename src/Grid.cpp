#include "Grid.h"
#include "Vec2.h"

#include <algorithm>
#include <cmath>

Grid::Grid(int cols, int rows, float cellSize)
	: m_cols(std::max(cols, 1))
	, m_rows(std::max(rows, 1))
	, m_cellSize(cellSize > 0.f ? cellSize : 1.f)
	, m_tiles(static_cast<std::size_t>(m_cols) * static_cast<std::size_t>(m_rows), Tile::Open)
{}

int   Grid::cols() const { return m_cols; }
int   Grid::rows() const { return m_rows; }
float Grid::cellSize() const { return m_cellSize; }

bool Grid::inBounds(Cell c) const
{
	return c.x >= 0 && c.x < m_cols && c.y >= 0 && c.y < m_rows;
}

int Grid::index(Cell c) const
{
	return c.y * m_cols + c.x;
}

Tile Grid::get(Cell c) const
{
	if (!inBounds(c)) return Tile::Wall;
	return m_tiles[static_cast<std::size_t>(index(c))];
}

void Grid::set(Cell c, Tile t)
{
	if (!inBounds(c)) return;
	m_tiles[static_cast<std::size_t>(index(c))] = t;
}

void Grid::clearWalls()
{
	std::fill(m_tiles.begin(), m_tiles.end(), Tile::Open);
}

Cell Grid::worldToCell(const Vec2& p) const
{
	// floor, not static_cast<int>(p / cellSize): truncation rounds toward zero,
	// so anything in (-cellSize, 0) would wrongly land in cell 0.
	return Cell{
		static_cast<int>(std::floor(p.x / m_cellSize)),
		static_cast<int>(std::floor(p.y / m_cellSize))
	};
}

Vec2 Grid::cellTopLeft(Cell c) const
{
	return Vec2(static_cast<float>(c.x) * m_cellSize, static_cast<float>(c.y) * m_cellSize);
}

Vec2 Grid::cellCenter(Cell c) const
{
	return Vec2((static_cast<float>(c.x) + 0.5f) * m_cellSize,
	            (static_cast<float>(c.y) + 0.5f) * m_cellSize);
}
