#ifndef _DRAWING_H_
#define _DRAWING_H_

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/shared_ptr.hpp>

namespace veiv
{
typedef boost::geometry::model::d2::point_xy<double> Point2d;
typedef boost::geometry::model::polygon<Point2d> Polygon;

struct IntersectionData
{
  size_t poly_index_src;
  size_t poly_index_tgt;
  size_t segment_index_src;
  size_t segment_index_tgt;

  Point2d point;
};

class Drawing
{
public:
  typedef boost::shared_ptr<Drawing> Ptr;

  Drawing ();

  void
  computeIntersections (std::vector<IntersectionData> &result);

  std::vector<Polygon> polygons_;
};
}

#endif
