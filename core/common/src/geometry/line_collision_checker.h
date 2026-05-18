/**
 * @file line_collision_checker.h
 * @brief Bresenham 直线碰撞检测器 — 检查两点之间是否存在致命障碍物。
 *
 * 提供纯静态接口，不依赖 Node 类型或 accessor methods，
 * 直接操作 int 坐标和 costmap 原始数据，供所有规划算法复用。
 */
#ifndef RMP_COMMON_GEOMETRY_LINE_COLLISION_CHECKER_H_
#define RMP_COMMON_GEOMETRY_LINE_COLLISION_CHECKER_H_

#include <cmath>

namespace rmp::common::geometry {

class LineCollisionChecker
{
public:
  /**
   * @brief Bresenham 直线碰撞检测。
   *
   * 从 (x0, y0) 到 (x1, y1) 遍历所有经过的栅格，
   * 若任一栅格代价值 >= lethal_threshold 或越界，则判定为碰撞。
   *
   * @param x0               起点栅格 x
   * @param y0               起点栅格 y
   * @param x1               终点栅格 x
   * @param y1               终点栅格 y
   * @param char_map          costmap 原始代价数组指针
   * @param size_x            costmap 宽度（栅格数）
   * @param size_y            costmap 高度（栅格数）
   * @param lethal_threshold  致命障碍物阈值（通常为 LETHAL_OBSTACLE * inflation_factor）
   * @return true 表示碰撞（有障碍物），false 表示无碰撞
   */
  static bool hasCollision(
    int x0, int y0, int x1, int y1,
    const unsigned char * char_map,
    int size_x, int size_y,
    double lethal_threshold);
};

}  // namespace rmp::common::geometry

#endif  // RMP_COMMON_GEOMETRY_LINE_COLLISION_CHECKER_H_
