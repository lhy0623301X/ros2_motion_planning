/**
 * @file line_collision_checker.cpp
 * @brief Bresenham 直线碰撞检测器实现。
 */
#include "geometry/line_collision_checker.h"

namespace rmp::common::geometry {

bool LineCollisionChecker::hasCollision(
  int x0, int y0, int x1, int y1,
  const unsigned char * char_map,
  int size_x, int size_y,
  double lethal_threshold)
{
  const int map_size = size_x * size_y;

  auto grid2Index = [size_x](int x, int y) -> int {
    return x + size_x * y;
  };

  auto isOutOfBounds = [size_x, size_y](int x, int y) -> bool {
    return x < 0 || y < 0 || x >= size_x || y >= size_y;
  };

  const int dx = std::abs(x1 - x0);
  const int dy = -std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;

  while (true) {
    if (isOutOfBounds(x0, y0)) {
      return true;
    }

    const int idx = grid2Index(x0, y0);
    if (idx < 0 || idx >= map_size || char_map[idx] >= lethal_threshold) {
      return true;
    }

    if (x0 == x1 && y0 == y1) {
      break;
    }

    const int error2 = 2 * error;
    if (error2 >= dy) {
      error += dy;
      x0 += sx;
    }
    if (error2 <= dx) {
      error += dx;
      y0 += sy;
    }
  }

  return false;
}

}  // namespace rmp::common::geometry
