#pragma once

#include "drawing.h"

struct svgtiny_shape;
namespace veiv
{
class SVGReader
{
public:
  SVGReader ();

  void
  setInputDrawing (Drawing::Ptr drawing)
  { drawing_ = drawing; }

  bool
  readFile (std::string &filename);

  Drawing::Ptr drawing_;

private:
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
