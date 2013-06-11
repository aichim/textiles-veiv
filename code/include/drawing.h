#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace veiv
{
class Drawing
{
public:
  typedef boost::geometry::model::d2::point_xy<double> Point2d;
  typedef boost::geometry::model::polygon<Point2d> Polygon;

  Drawing ();

  std::vector<Polygon> polygons;
};
}
