//
//  main.cpp
//  Repeat
//
//  Created by Lukas Murmann on 6/13/13.
//  Copyright (c) 2013 Lukas Murmann. All rights reserved.
//

#include <iostream>
#include "mesh.h"
#include <Eigen/Dense>



int main(int argc, const char * argv[]) {

    if (argc != 7) {
        fprintf(stderr, "Usage: %s <n_repeat_x> <n_repeat_y> <kernel_size> <arc_at_ratio> infile outfile\n", argv[0]);
        exit(-1);
    }

    const int nx = atoi(argv[1]);
    const int ny = atoi(argv[2]);

    const double offs = atof(argv[3]);
    const double arc_at_ratio = atof(argv[4]);

    const std::string kernel_file = argv[5];
    const std::string out_file = argv[6];

//    static const double offs = 12;
//    static const double arc_at_ratio = 0.52;

    static const Eigen::Vector3f offsx = Eigen::Vector3f::UnitX() * offs;
    static const Eigen::Vector3f offsy = Eigen::Vector3f::UnitZ() * offs;

    // Very simple "script". Most functionality is in mesh.cpp file.




    MyMesh kernel;
    OpenMesh::IO::read_mesh(kernel, kernel_file);
    Center(&kernel);
    MyMesh base;
    for (int x = 0 ; x < nx ; ++x) {
        for (int y = 0 ; y < ny ; ++y) {
            const Eigen::Vector3f offset = x * offsx + y * offsy;
            MyMesh moved_kernel = kernel;
            Add(&moved_kernel, offset);
            Cat(&base, moved_kernel);
        }
    }

    static const Eigen::Vector3f diagonal_offs = (offsx + offsy) * arc_at_ratio;
    MyMesh base_diagonal = base;
    Add(&base, diagonal_offs);
    Cat(&base, base_diagonal);

    OpenMesh::IO::write_mesh(base, out_file);
    std::cerr << "done" << std::endl;
    return 0;
}

