#ifndef _UTILS_H_
#define _UTILS_H_

#include <string>

#include "drawing.h"

#include <Eigen/Core>

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


  void
  quadsBetweenCircles (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &curr_circle,
                       std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &prev_circle,
                       std::vector<std::vector<Point3> > &quads);

  void
  removeDuplicatePoints (const std::vector<std::vector<Point3> > &lines,
                         std::vector<std::vector<Point3> > &clean);
}

#endif
