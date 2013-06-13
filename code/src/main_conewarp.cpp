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
    MyMesh kernel;
    OpenMesh::IO::read_mesh(kernel, "/Users/lum/code/kernel_out.obj");

    veiv::conewarp(&kernel, 1, 2, 2*M_PI_2);
    OpenMesh::IO::write_mesh(kernel, "/Users/lum/code/kernel_normalized_pi.obj");
    std::cerr << "done" << std::endl;
    return 0;

}

