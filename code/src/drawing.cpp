#include "drawing.h"


veiv::Drawing::Drawing ()
{
}


void
veiv::Drawing::computeIntersections (std::vector<IntersectionData> &result)
{
  /// For each pair of polygons
  for (size_t poly_src_i = 0; poly_src_i < polygons_.size (); ++poly_src_i)
  {
    for (size_t poly_tgt_i = poly_src_i + 1; poly_tgt_i < polygons_.size (); ++poly_tgt_i)
    {
      /// Compute the intersection points
      std::vector<Point2d> intersection_points;
      boost::geometry::intersection (polygons_[poly_src_i], polygons_[poly_tgt_i], intersection_points);

      /// If no intersection, go to the next pair of polygons
      if (intersection_points.size () == 0)
        continue;

      printf ("Found %zu intersections between poly %zu and poly %zu\n",
              intersection_points.size (), poly_src_i, poly_tgt_i);

      /// Check on which segments the intersection points fall on
      for (size_t intersection_i = 0; intersection_i < intersection_points.size (); ++intersection_i)
      {
        double min_dist_src = std::numeric_limits<double>::max ();
        size_t min_index_src = 0;
        for (size_t s_i = 0; s_i < polygons_[poly_src_i].outer ().size () - 1; ++s_i)
        {
          Point2d &a = polygons_[poly_src_i].outer ()[s_i];
          Point2d &b = polygons_[poly_src_i].outer ()[s_i + 1];
          double dist = boost::geometry::distance (boost::geometry::model::segment<Point2d> (a, b), intersection_points[intersection_i]);

          if (dist < min_dist_src)
          {
            min_dist_src = dist;
            min_index_src = s_i;
          }
        }

        double min_dist_tgt = std::numeric_limits<double>::max ();
        size_t min_index_tgt = 0;
        for (size_t s_i = 0; s_i < polygons_[poly_tgt_i].outer ().size () - 1; ++s_i)
        {
          Point2d &a = polygons_[poly_tgt_i].outer ()[s_i];
          Point2d &b = polygons_[poly_tgt_i].outer ()[s_i + 1];
          double dist = boost::geometry::distance (boost::geometry::model::segment<Point2d> (a, b), intersection_points[intersection_i]);

          if (dist < min_dist_tgt)
          {
            min_dist_tgt = dist;
            min_index_tgt = s_i;
          }
        }


        /// Create the IntersectionData structure and add it to the result
        IntersectionData idata;
        idata.point = intersection_points[intersection_i];
        idata.poly_index_src = poly_src_i;
        idata.poly_index_tgt = poly_tgt_i;
        idata.segment_index_src = min_index_src;
        idata.segment_index_tgt = min_index_tgt;

        result.push_back (idata);
      }
    }
  }

}
