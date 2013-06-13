//
//  conewarp.cpp
//  Relight
//
//  Created by Lukas Murmann on 6/13/13.
//  Copyright (c) 2013 Lukas Murmann. All rights reserved.
//

#include "conewarp.h"
#include <cassert>
#include <boost/geometry.hpp>
#include <cmath>

namespace veiv {

    void conewarp(MyMesh * mesh, float rmin, float rmax, float phimax) {
        assert(rmax > rmin);
        assert(rmax > 0);
        assert(rmin > 0);
        assert(phimax > 0);
        // assert(phimax < 2 * 3.14);
        ToFirstQuadrant(mesh);

        float rgain = rmax - rmin;

        for (MyMesh::VertexIter vi = mesh->vertices_begin() ;
             vi != mesh->vertices_end() ;
             ++vi) {
            MyMesh::Point & p = mesh->point(vi);
            float phi = phimax * p[0];
            float r = rmin + rgain * p[2];
            //r *=r;

            p[0] = r * std::cos(phi);
            p[2] = r * std::sin(phi);
            p[1]*= 0.08; // FIXME: hardcoded
        }
    }
}