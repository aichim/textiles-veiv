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

  return (!success);
}
