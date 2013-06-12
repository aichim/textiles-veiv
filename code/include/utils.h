#pragma once

#include <string>

#include "drawing.h"

namespace veiv
{
  typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point3;

  bool
  saveOBJ (std::string &filename,
           std::vector<std::vector<Point3> > &model);

  bool
  extrudeCircleAlongPath (const std::vector<std::vector<Point3> > &model,
                          const double radius,
                          const size_t num_circle_samples,
                          const double sampling_dist,
                          std::vector<std::vector<Point3> > &result);
}
