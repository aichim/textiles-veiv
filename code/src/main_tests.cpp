#include <iostream>
#include "utils.h"
#include "drawing.h"
#include "SVGReader.h"

int
main (int argc,
      char **argv) {

  if (argc != 2) {
		fprintf(stderr, "Usage: %s FILE\n", argv[0]);
		return 1;
	}
  std::string svg_file = argv[1];
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

  }

  return (!success);
}
