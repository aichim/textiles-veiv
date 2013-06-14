//
//  main.cpp
//  conewarp
//
//  Created by Lukas Murmann on 6/13/13.
//  Copyright (c) 2013 Lukas Murmann. All rights reserved.
//

#include <iostream>
#include "mesh.h"
#include "conewarp.h"
#include <limits>


int main(int argc, const char * argv[])
{
    if (argc != 6) {
        fprintf(stderr, "Usage: %s <from_radius> <to_radius> <angle> infile outfile\n", argv[0]);
        exit(-1);
    }
    const float min_r = atof(argv[1]);
    const float max_r = atof(argv[2]);
    const float angle = atof(argv[3]);
    const std::string infile = argv[4];
    const std::string outfile = argv[5];


    MyMesh kernel;
    OpenMesh::IO::read_mesh(kernel, infile);

    veiv::conewarp(&kernel, min_r, max_r, angle);
    OpenMesh::IO::write_mesh(kernel, outfile);
    std::cerr << "done" << std::endl;
    return 0;

}

