#include <iostream>
#include "utils.h"
#include "drawing.h"

int
main (int argc,
      char **argv) {

    if (argc != 2) {
		fprintf(stderr, "Usage: %s FILE\n", argv[0]);
		return 1;
	}
    std::string svg_file = argv[1];
    veiv::Drawing drawing;
    bool success = veiv::readSVG(svg_file, drawing);

    return (!success);
}
