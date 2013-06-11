#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace veiv
{
class Drawing
{
public:
  typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > Polygon;

  Drawing ();

  std::vector<Polygon> polygons;
};
}
