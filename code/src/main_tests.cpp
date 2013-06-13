#include <iostream>
#include <cstdlib>
#include "utils.h"
#include "drawing.h"
#include "SVGReader.h"
#include "arc_generator.h"

//typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point3;

int
main (int argc,
      char **argv) {

  if (argc != 3) {
    fprintf(stderr, "Usage: %s FILE RADIUS\n", argv[0]);
    return 1;
  }
  std::string svg_file = argv[1];
  double radius_value = std::atof(argv[2]);
  veiv::Drawing::Ptr drawing(new veiv::Drawing());
  veiv::SVGReader reader;
  bool success = reader.readFile(svg_file, drawing);

  if (success)
  {
    printf ("works!\n");

    /// Show the points
    printf ("There are %zu closed curves\n", drawing->polygons_.size ());
    for (size_t poly_i = 0; poly_i < drawing->polygons_.size (); ++poly_i)
    {
      printf ("Poly %zu has %zu points\n", poly_i, drawing->polygons_[poly_i].outer ().size ());
      for (size_t p_i = 0; p_i < drawing->polygons_[poly_i].outer ().size (); ++p_i)
        printf ("point %f %f\n", drawing->polygons_[poly_i].outer ()[p_i].x (), drawing->polygons_[poly_i].outer ()[p_i].y ());
    }


    std::vector<veiv::IntersectionData> intersections;
    drawing->computeIntersections (intersections);

    for (size_t i = 0; i < intersections.size (); ++i)
    {
      printf ("poly %zu : %zu --- %zu : %zu at point %f %f\n",
              intersections[i].poly_index_src,
              intersections[i].segment_index_src,
              intersections[i].poly_index_tgt,
              intersections[i].segment_index_tgt,
              intersections[i].point.x (),
              intersections[i].point.y ());
    }

    /// Write as a 2d obj file
    std::vector<std::vector<veiv::Point3> > weave_3d;

    std::vector< std::vector<veiv::Point2> > curves_2d;
    for (size_t poly_i = 0; poly_i < drawing->polygons_.size (); ++poly_i)
    {
      std::vector<veiv::Point2> polygon;
      for (size_t p_i = 0; p_i < drawing->polygons_[poly_i].outer ().size (); ++p_i)
      {
        veiv::Point2 p (drawing->polygons_[poly_i].outer ()[p_i].x (),
                        drawing->polygons_[poly_i].outer ()[p_i].y ());
        polygon.push_back (p);
      }

      curves_2d.push_back (polygon);
    }

    //  for (size_t poly_i = 0; poly_i < drawing->polygons_.size (); ++poly_i)
    //  {
    //    std::vector<Point3> polygon;
    //    for (size_t p_i = 0; p_i < drawing->polygons_[poly_i].outer ().size (); ++p_i)
    //    {
    //      Point3 p ( drawing->polygons_[poly_i].outer ()[p_i].x (),
    //                 drawing->polygons_[poly_i].outer ()[p_i].y (),
    //                 0.);
    //      polygon.push_back (p);
    //    }
    //
    //    weave_3d.push_back (polygon);
    //  }

    veiv::generate_arc(curves_2d, intersections, radius_value, weave_3d);
    //std::vector< std::vector<Point3> > &output_curve_points)

    std::string output_filename ("output.obj");
    veiv::saveOBJ (output_filename, weave_3d);


    for (size_t poly_i = 0; poly_i < weave_3d.size (); ++poly_i)
      weave_3d[poly_i].resize (weave_3d[poly_i].size () - 1);
    std::vector<std::vector<veiv::Point3> > weave_3d_clean;
    veiv::removeDuplicatePoints (weave_3d, weave_3d_clean);

    std::vector<std::vector<veiv::Point3> > weave_tube;
    veiv::extrudeCircleAlongPath (weave_3d_clean, radius_value, 20, 5, weave_tube);
    output_filename = "output_tube.obj";
    veiv::saveOBJ (output_filename, weave_tube);
  }
  else
    return (-1);









  //  /// HACK delete the last point in every polygon
  //  for (size_t poly_i = 0; poly_i < weave_3d.size (); ++poly_i)
  //    weave_3d[poly_i].resize (weave_3d[poly_i].size () - 1);
  //
  //  std::vector<std::vector<Point3> > weave_tube;
  //  veiv::extrudeCircleAlongPath (weave_3d, 15, 20, 5, weave_tube);
  //  output_filename = "output_tube.obj";
  //  veiv::saveOBJ (output_filename, weave_tube);


  return (0);
}
