#pragma once

#include "drawing.h"

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
};
}
