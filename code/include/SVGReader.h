#pragma once

#include "drawing.h"

struct svgtiny_shape;

namespace veiv
{
class SVGReader
{
public:
  SVGReader ();
  bool readFile (const std::string & filename, Drawing::Ptr drawing);

private:
  Drawing::Ptr drawing_;

  void render_path(svgtiny_shape *path);

  void new_path();
  void close_path();
  void move_to(const Point2d &);
  void line_to(const Point2d &);
  void curve_to(const Point2d &,
                const Point2d &,
                const Point2d &);
};
}
