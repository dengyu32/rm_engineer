#pragma once

#include <algorithm>
#include <vector>

#include "coal/data_types.h"

namespace coal
{

inline Scalar cross2d(const Vec2s& origin, const Vec2s& a, const Vec2s& b)
{
  const Vec2s oa = a - origin;
  const Vec2s ob = b - origin;
  return oa.x() * ob.y() - oa.y() * ob.x();
}

inline void computePolygonConvexHull(const std::vector<Vec2s>& cloud, std::vector<Vec2s>& cvx_hull)
{
  cvx_hull.clear();
  if (cloud.size() <= 1)
  {
    cvx_hull = cloud;
    return;
  }

  std::vector<Vec2s> points = cloud;
  std::sort(points.begin(), points.end(), [](const Vec2s& lhs, const Vec2s& rhs) {
    if (lhs.x() == rhs.x())
    {
      return lhs.y() < rhs.y();
    }
    return lhs.x() < rhs.x();
  });

  points.erase(std::unique(points.begin(), points.end(), [](const Vec2s& lhs, const Vec2s& rhs) {
                 return lhs.x() == rhs.x() && lhs.y() == rhs.y();
               }),
               points.end());
  if (points.size() <= 1)
  {
    cvx_hull = points;
    return;
  }

  std::vector<Vec2s> hull;
  hull.reserve(points.size() * 2);
  for (const auto& point : points)
  {
    while (hull.size() >= 2 && cross2d(hull[hull.size() - 2], hull.back(), point) <= 0)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }

  const std::size_t lower_size = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it)
  {
    while (hull.size() > lower_size && cross2d(hull[hull.size() - 2], hull.back(), *it) <= 0)
    {
      hull.pop_back();
    }
    hull.push_back(*it);
  }

  if (!hull.empty())
  {
    hull.pop_back();
  }
  cvx_hull = std::move(hull);
}

}  // namespace coal
