//
//  mesh.cpp
//  MViewer
//
//  Created by Lukas Murmann on 2/26/13.
//
//


#include "mesh.h"
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <set>
#include <vector>
#include <limits>


long GetNumOfElements(const MyMesh &) {
    assert(false);
    return 0;
}

long GetNumOfVertices(const MyMesh & mesh) {
    return GetVertices(mesh).cols();
}

Pointset GetVertices(const MyMesh & mesh) {
    Pointset ret;
    ret.resize(3, mesh.n_vertices());
    for (MyMesh::VertexIter cv_it = mesh.vertices_begin() ; cv_it != mesh.vertices_end() ; ++cv_it) {
        int v_idx = cv_it->idx();
        const MyMesh::Point & p = mesh.point(cv_it.handle());
        ret.col(v_idx) = Eigen::Vector3f(p[0], p[1], p[2]);
    }
    return ret;
}

Pointset GetNormals(const MyMesh & mesh) {
    Pointset ret;
    ret.resize(3, mesh.n_vertices());
    for (MyMesh::VertexIter cv_it = mesh.vertices_begin() ; cv_it != mesh.vertices_end() ; ++cv_it) {
        int v_idx = cv_it->idx();
        const MyMesh::Normal & n = mesh.normal(cv_it.handle());
        ret.col(v_idx) = Eigen::Vector3f(n[0], n[1], n[2]);
    }
    return ret;
}

Pointset GetVerticesFlat( const MyMesh & mesh ){
    Pointset ret;
    ret.resize(3, mesh.n_faces()*3);
    int idx = 0;
    for (MyMesh::ConstFaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it){
        for (MyMesh::ConstFaceVertexIter cfv_it = mesh.cfv_iter(f_it.handle()); cfv_it; ++cfv_it){
            const MyMesh::Point & p = mesh.point(cfv_it.handle());
            ret.col(idx) = Eigen::Vector3f(p[0], p[1], p[2]);
            ++idx;
        }
    }
    return ret;
}

Pointset GetTexCoords( const MyMesh &mesh ){
    Pointset ret;
    ret.resize(2, mesh.n_vertices());
    int idx = 0;
    for (MyMesh::VertexIter cv_it = mesh.vertices_begin() ; cv_it != mesh.vertices_end() ; ++cv_it) {
        const MyMesh::TexCoord2D & tc = mesh.texcoord2D(cv_it.handle());
        ret.col(idx) = Eigen::Vector2f(tc[0], tc[1]);
        ++idx;
    }
    return ret;
}


Pointset GetTexCoordsFlat( const MyMesh &mesh ){
    Pointset ret;
    ret.resize(2, mesh.n_faces()*3);
    int idx = 0;
    for (MyMesh::ConstFaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it){
        for (MyMesh::ConstFaceVertexIter cfv_it = mesh.cfv_iter(f_it.handle()); cfv_it; ++cfv_it){
            const MyMesh::TexCoord2D & tc = mesh.texcoord2D(cfv_it.handle());
            ret.col(idx) = Eigen::Vector2f(tc[0], tc[1]);
            ++idx;
        }
    }
    return ret;
}

Pointset GetNormalsFlat( const MyMesh & mesh ){
    Pointset ret;
    ret.resize(3, mesh.n_vertices());
    int idx = 0;
    for (MyMesh::ConstFaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it){
        const MyMesh::Normal & nf = mesh.normal(f_it.handle());
        for (MyMesh::ConstFaceVertexIter cfv_it = mesh.cfv_iter(f_it.handle()); cfv_it; ++cfv_it){
            ret.col(idx) = Eigen::Vector3f(nf[0], nf[1], nf[2]);
            ++idx;
        }
    }
    return ret;
}

int NumAdjacentFaces (const MyMesh & mesh, const MyMesh::VertexHandle & v) {
    int count = 0;
    for (MyMesh::ConstVertexFaceIter cvf  = mesh.cvf_begin(v) ;
         cvf ;
         ++cvf) {
        ++count;
    }
    return count;
}

std::vector<unsigned int> GetTriangleIndicesFlat(const MyMesh & mesh ){
    std::vector<unsigned int> ret;

    int idx = 0;
    for (MyMesh::ConstFaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it){
        for (MyMesh::ConstFaceVertexIter cfv_it = mesh.cfv_iter(f_it.handle()); cfv_it; ++cfv_it){
            ret.push_back(idx);
            ++idx;
        }
    }
    return ret;
}

void SetVertices(MyMesh *, const Pointset & vertices ) {
    assert(false);
}

void SetPoint(MyMesh *, int index, Eigen::Vector3f vertex) {
    assert(false);
}

void setNormals( MyMesh & mesh, const Pointset & normals ){
    for (MyMesh::VertexIter cv_it = mesh.vertices_begin() ; cv_it != mesh.vertices_end() ; ++cv_it) {
        int v_idx = cv_it->idx();
        MyMesh::Normal n = mesh.normal(cv_it.handle());
        n[0] = normals( 0, v_idx  );
        n[1] = normals( 1, v_idx  );
        n[2] = normals( 2, v_idx  );
        //        mesh.update_normal(cv_it.handle());
    }
}

std::vector<long> VertexNeighbors (const MyMesh & mesh, const MyMesh::VertexHandle & v) {
    std::vector<long> ret;
    for (MyMesh::ConstVertexVertexIter cvv = mesh.cvv_begin(v) ;
         cvv ;
         ++cvv) {
        const MyMesh::VertexHandle & nb = cvv.handle();
        ret.push_back(nb.idx());
    }
    return ret;
}

void VertexKNeighbors(MyMesh *mesh, long vertex_idx, int depth, std::vector<long> & neighbors){
    //recursion end condition
    if (depth <= 0) {
        return;
    }

    // a new vector to store current neighborhood
    const MyMesh::VertexHandle & vh = mesh->vertex_handle(static_cast<int>(vertex_idx));
    std::vector<long> currentNeighbors;

    // find the one-ring neighbors for all current level vertices
    mesh->status(vh).set_tagged(true);
    for (MyMesh::VertexVertexIter vv = mesh->vv_begin(vh); vv ; ++vv) {
        const MyMesh::VertexHandle & cur_vh = vv.handle();
        if( !mesh->status(cur_vh).tagged() ) {
            currentNeighbors.push_back(vv.handle().idx());
            mesh->status(cur_vh).set_tagged(true);
        }
    }

    // add selected neighbors
    neighbors.insert( neighbors.end(), currentNeighbors.begin(), currentNeighbors.end() );
    for (unsigned int i = 0; i<currentNeighbors.size(); ++i){
        VertexKNeighbors(mesh, currentNeighbors[i], depth - 1, neighbors);
    }
}


void VertexRadNeighborsRecurse(MyMesh *mesh, long vertex_idx, const float sqRadius,
                               const Eigen::Vector3f & center_location, std::vector<long> & neighbors){
    Eigen::Vector3f cur_location = GetVertex(*mesh, vertex_idx);
    if( (cur_location - center_location).squaredNorm() > sqRadius ) {
        return;
    }
    //vector to store indices of current 1 ring neighbors
    std::vector<long> currentNeighbors;

    // find the one-ring neighbors for current vertex
    const MyMesh::VertexHandle & vh = mesh->vertex_handle(static_cast<int>(vertex_idx));
    mesh->status(vh).set_tagged(true);
    for (MyMesh::VertexVertexIter vv = mesh->vv_begin(vh); vv ; ++vv) {
        const MyMesh::VertexHandle & cur_vh = vv.handle();
        if( !mesh->status(cur_vh).tagged() ) {
            currentNeighbors.push_back(vv.handle().idx());
            mesh->status(cur_vh).set_tagged(true);
        }
    }

    // add selected neighbors
    neighbors.insert( neighbors.end(), currentNeighbors.begin(), currentNeighbors.end() );
    for (unsigned int i = 0; i<currentNeighbors.size(); ++i){
        VertexRadNeighborsRecurse(mesh, currentNeighbors[i], sqRadius, center_location ,neighbors);
    }
}

void VertexRadNeighbors(MyMesh * mesh, const int center,
                        const float sqRadius, std::vector<long> & neighbors){
    Eigen::Vector3f center_location = GetVertex(*mesh, center);
    VertexRadNeighborsRecurse(mesh, center, sqRadius, center_location, neighbors );
}

void FaceRadNeighborsRecurse(MyMesh *mesh, int face_idx, const float radius,
                             const Eigen::Vector3f & center_location,
                             const OpenMesh::FPropHandleT<Eigen::Vector3f> & face_centroids,
                             OpenMesh::FPropHandleT<float> & face_distances,
                             std::vector<long> & neighbors){
    //check if this is valid neighbor
    Eigen::Vector3f cur_location = mesh->property( face_centroids, mesh->face_handle(face_idx) );
    float dist = (cur_location - center_location).norm();
    if( dist > radius ) {
        return;
    }
    //store distance for later
    mesh->property( face_distances, mesh->face_handle(face_idx) ) = dist;
    //vector to store indices of current 1 ring neighbors
    std::vector<long> currentNeighbors;

    // find the neighbors of current face
    const MyMesh::FaceHandle & fh = mesh->face_handle(face_idx);
    mesh->status(fh).set_tagged(true);
    for (MyMesh::FaceFaceIter ff = mesh->ff_begin(fh); ff; ++ff) {
        const MyMesh::FaceHandle & cur_fh = ff.handle();
        if( !mesh->status(cur_fh).tagged() ) {
            currentNeighbors.push_back(ff.handle().idx());
            mesh->status(cur_fh).set_tagged(true);
        }
    }

    // add selected neighbors
    neighbors.insert( neighbors.end(), currentNeighbors.begin(), currentNeighbors.end() );
    for (unsigned int i = 0; i<currentNeighbors.size(); ++i){
        FaceRadNeighborsRecurse(mesh,
                                static_cast<int>(currentNeighbors[i]),
                                radius,
                                center_location, 
                                face_centroids,
                                face_distances, 
                                neighbors);
    }
}

void FaceRadNeighbors(MyMesh *mesh, const long center, const float radius,
                      const OpenMesh::FPropHandleT<Eigen::Vector3f> & face_centroids,
                      OpenMesh::FPropHandleT<float> & face_distances,
                      std::vector<long> & neighbors){
    const Eigen::Vector3f center_location = mesh->property( face_centroids,
                                                             mesh->face_handle(static_cast<int>(center)) );
    FaceRadNeighborsRecurse(mesh,
                            static_cast<int>(center),
                            radius,
                            center_location,
                            face_centroids,
                            face_distances,
                            neighbors );
}

void clearAllVisitedStatus(MyMesh * mesh){
    for (MyMesh::VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end() ; ++v_it){
        mesh->status(v_it).set_tagged(false);
    }
    for (MyMesh::FaceIter f_it = mesh->faces_begin(); f_it != mesh->faces_end() ; ++f_it){
        mesh->status(f_it).set_tagged(false);
    }
}

void clearFaceVisitedStatus(MyMesh * mesh, const std::vector<long> & list){
    for(unsigned int i = 0; i < list.size(); i++ ){
        const MyMesh::FaceHandle &cur_fh = mesh->face_handle(static_cast<int>(list[i]));
        mesh->status(cur_fh).set_tagged(false);
    }
}

std::vector<unsigned int>   GetTriangleIndices(const MyMesh & mesh) {
    std::vector<unsigned int> ret;

    for (MyMesh::ConstFaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it){
        // circulate around the current face
        for (MyMesh::ConstFaceVertexIter cfv_it = mesh.cfv_iter(f_it.handle()); cfv_it; ++cfv_it){
            // print out each vertex of a current face
            int current_index = cfv_it.handle().idx();
            ret.push_back(current_index);
        }
    }

    return ret;
}


std::vector<long>  VertexNeighborEdges (const MyMesh & mesh, const MyMesh::VertexHandle & v) {
    std::vector<long> ret;
    for (MyMesh::ConstVertexEdgeIter cve = mesh.cve_begin(v) ;
         cve ;
         ++cve) {
        const MyMesh::EdgeHandle & nb = cve.handle();
        ret.push_back(nb.idx());
    }
    return ret;

}
Eigen::Vector2f GetBoundingBox(const MyMesh &mesh){
    // bounding box method, for now it just returns the

    OpenMesh::Vec3f bbMin, bbMax;
    MyMesh::ConstVertexIter v_it(mesh.vertices_begin());
    MyMesh::ConstVertexIter v_end(mesh.vertices_end());
    bbMin = bbMax = OpenMesh::vector_cast<OpenMesh::Vec3f>(mesh.point(v_it));
    for (; v_it!=v_end; ++v_it) {
        bbMin.minimize( OpenMesh::vector_cast<OpenMesh::Vec3f>(mesh.point(v_it)));
        bbMax.maximize( OpenMesh::vector_cast<OpenMesh::Vec3f>(mesh.point(v_it)));
    }
    return Eigen::Vector2f(bbMin[1], bbMax[1]);
}

void Center( MyMesh * mesh ) {
    Eigen::Vector3f centroid = GetCentroid(*mesh);
    Substract(mesh, centroid);
}

void Normalize(MyMesh * mesh){
    Eigen::Vector2f range = GetBoundingBox( *mesh );
    float a = -1.0/(range(0) - range(1));
    // float b = -range[0] * a;
    Center(mesh);
    Scale(mesh, a);
}

void Cat(MyMesh * base, const MyMesh & b) {
    std::vector<MyMesh::VertexHandle> vhandles(b.n_vertices());
    for (MyMesh::ConstVertexIter vi = b.vertices_begin() ;
         vi != b.vertices_end() ;
         ++vi) {
        vhandles[vi->idx()] = base->add_vertex(b.point(vi));
    }
    for (MyMesh::ConstFaceIter fi = b.faces_begin() ;
         fi != b.faces_end() ;
         ++fi) {
        const MyMesh::FaceHandle & cfh = fi;
        std::vector<MyMesh::VertexHandle> face_verts;
        for (MyMesh::ConstFaceVertexIter fvi = b.cfv_begin(cfh) ;
             fvi ;
             ++fvi) {
            const MyMesh::VertexHandle & b_verthandle = fvi;
            const MyMesh::VertexHandle & base_verthandle = vhandles[b_verthandle.idx()];
            face_verts.push_back(base_verthandle);
        }
        base->add_face(face_verts);
    }
}

MyMesh Merge(const MyMesh & a, const MyMesh & b) {
    MyMesh ret = a;
    Cat(&ret, b);
    return ret;
}

Eigen::Vector3f GetCentroid( const MyMesh & mesh ) {
    unsigned num_verts = mesh.n_vertices();
    assert(num_verts);
    OpenMesh::Vec3f centroid(0,0,0);

    for ( MyMesh::ConstVertexIter cv_it = mesh.vertices_begin();
         cv_it!= mesh.vertices_end();
         ++cv_it) {
        centroid += mesh.point(cv_it.handle());
    }
    centroid /= num_verts;
    return Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
}

void Scale(MyMesh * mesh, float s) {
    for ( MyMesh::VertexIter v_it = mesh->vertices_begin();
         v_it!= mesh->vertices_end();
         ++v_it) {
        mesh->point(v_it.handle()) *= s;
    }
}
void Scale(MyMesh * mesh, const Eigen::Vector3f & v) {
    printf("WARNING: Nonuniform scaling is not implemented correctly.\n");
    printf("         Make sure that normals are correct after using.\n");
    for ( MyMesh::VertexIter v_it = mesh->vertices_begin();
         v_it!= mesh->vertices_end();
         ++v_it) {
        MyMesh::Point & p =  mesh->point(v_it.handle());
        p[0] *= v(0);
        p[1] *= v(1);
        p[2] *= v(2);
    }
}

void Add(MyMesh * mesh, const Eigen::Vector3f & v) {
    OpenMesh::Vec3f add(v(0), v(1), v(2));
    for ( MyMesh::VertexIter v_it = mesh->vertices_begin();
         v_it!= mesh->vertices_end();
         ++v_it) {
        mesh->point(v_it.handle()) += add;
    }
}

void Substract(MyMesh * mesh, const Eigen::Vector3f & v) {
    Add(mesh, -v);
}
float ComputeSumOfDistances( const MyMesh & a, const MyMesh & b ) {
    float sum = 0;
    assert(a.n_vertices() == b.n_vertices());
    MyMesh::ConstVertexIter cv_it_a = a.vertices_begin();
    MyMesh::ConstVertexIter cv_it_b = b.vertices_begin();
    for ( ;
         cv_it_a != a.vertices_end();
         ++cv_it_a, ++cv_it_b) {
        sum += (a.point(cv_it_a.handle()) - b.point(cv_it_b.handle())).norm();
    }
    return sum;
}

long NumBoundaryVertices(const MyMesh & m) {
    unsigned ret = 0;
    for (MyMesh::ConstVertexIter cvi = m.vertices_begin() ;
         cvi != m.vertices_end() ;
         ++cvi) {
        if (m.is_boundary(cvi)) {
            ret++;
        }
    }
    return ret;
}

bool is_in_boundary_onering(const MyMesh & m, const MyMesh::VertexHandle &vh) {
    bool has_bound_nb = false;

    // only do something if vertex is not boundary vertex
    if (!m.is_boundary(vh)) {
        for (MyMesh::ConstVertexVertexIter cvv = m.cvv_begin(vh);
             cvv ;
             ++cvv) {
            if (m.is_boundary(cvv)) {
                has_bound_nb = true;
                break;
            }
        }
    }

    return has_bound_nb;
}

/* Note: This is not super great design. I could also just code the
 foreach / foreach stuff once and then call a callback for each vertex.*/
long NumBoundary1RingVertices(const MyMesh & m) {
    std::set<unsigned> onering_indices;
    for (MyMesh::ConstVertexIter cvi = m.vertices_begin();
         cvi != m.vertices_end() ;
         ++cvi) {

        if (m.is_boundary(cvi)) {
            for (MyMesh::ConstVertexVertexIter cvv = m.cvv_begin(cvi);
                 cvv ;
                 ++cvv) {
                const MyMesh::VertexHandle & vh = cvv;
                //printf("At %d\n", vh.idx());
                if (!m.is_boundary(vh)) {
                    onering_indices.insert(vh.idx());
                }
            }
        }
    }
    return onering_indices.size();
}

Eigen::Vector3f GetEdgeVertex (const MyMesh & mesh, const MyMesh::EdgeHandle & eh, int v_idx) {
    const MyMesh::HalfedgeHandle heh =  mesh.halfedge_handle(eh, v_idx);
    const MyMesh::VertexHandle vh = mesh.to_vertex_handle(heh);
    const MyMesh::Point p = mesh.point(vh);
    return Eigen::Vector3f(p[0], p[1], p[2]);
}

Pointset GetFaceVertices(const MyMesh &m, const MyMesh::FaceHandle &fh) {
    assert (fh.is_valid());
    Pointset ret(3,3);
    int i = 0;
    for (MyMesh::ConstFaceVertexIter fvi = m.cfv_begin(fh) ;
         fvi ;
         ++fvi) {
        ret.col(i) = ToEigen(m.point(fvi));
    }
    return ret;
}

Pointset GetFace(const MyMesh & mesh, const MyMesh::EdgeHandle & eh, int v_idx) {
    const MyMesh::HalfedgeHandle heh =  mesh.halfedge_handle(eh, v_idx);
    const MyMesh::FaceHandle fh = mesh.face_handle(heh);
    Pointset ret(0,0);
    if (fh.is_valid()) {
        ret = GetFaceVertices(mesh, fh);
    }
    return ret;
}


Eigen::Vector3f GetVertex (const MyMesh & mesh, long v_idx) {
    const MyMesh::VertexHandle vh = mesh.vertex_handle(static_cast<int>(v_idx));
    const MyMesh::Point p = mesh.point(vh);
    return Eigen::Vector3f(p[0], p[1], p[2]);
}

Eigen::Vector3f GetVNormal (const MyMesh & mesh, long v_idx) {
    const MyMesh::VertexHandle vh = mesh.vertex_handle(static_cast<int>(v_idx));
    const MyMesh::Normal n = mesh.normal(vh);
    return Eigen::Vector3f(n[0], n[1], n[2]);
}

Eigen::Vector3f GetFNormal (const MyMesh & mesh, long v_idx) {
    const MyMesh::FaceHandle fh = mesh.face_handle(static_cast<int>(v_idx));
    const MyMesh::Normal n = mesh.normal(fh);
    return Eigen::Vector3f(n[0], n[1], n[2]);
}

Eigen::Vector3f GetFaceCentroid (const MyMesh & mesh, const long f_idx) {
    const MyMesh::FaceHandle & fh = mesh.face_handle(static_cast<int>(f_idx));
    MyMesh::Point c;
    mesh.calc_face_centroid(fh , c);
    return Eigen::Vector3f(c[0], c[1], c[2]);
}

std::vector<float> ToVector(const Pointset & p) {
    size_t len = p.rows() * p.cols();
    std::vector<float> ret(len);
    std::copy(p.data(), p.data() + len, ret.begin());
    return ret;
}

//        mesh.set_normal( v_it, newLoc );


float FaceArea(const MyMesh & mesh, const MyMesh::FaceHandle & fh) {

    MyMesh::Point pts[3];
    int i = 0;
    for (MyMesh::ConstFaceVertexIter cfv = mesh.cfv_begin(fh) ;
         cfv ;
         ++cfv ) {
        pts[i++] = mesh.point(cfv);
    }
    auto v1 = pts[1] - pts[0];
    auto v2 = pts[2] - pts[0];
    auto n = v1 % v2; //cross product. I don't want to convert to Eigen for this.
    // norm of cross product is area of parallelogram, so divide by 2
    return n.length() / 2;
}
/* Compute simple barycentric neighborhood area: sum(neighbors) / 3
 Voronoi cells would be more exact
 */
float NeighborhoodArea(const MyMesh & mesh, const MyMesh::VertexHandle &vh) {
    float area = 0;
    for (MyMesh::ConstVertexFaceIter cvf = mesh.cvf_begin(vh) ;
         cvf ;
         ++cvf ) {
        area += FaceArea(mesh, cvf);
    }
    return area / 3;
}
void SetMeanCurvature(MyMesh* mesh) {
    float min = 1000;
    float max = 0;
    std::vector<float> curvatures(mesh->n_vertices());
    for ( MyMesh::VertexIter v_it = mesh->vertices_begin();
         v_it!= mesh->vertices_end();
         ++v_it) {
        float w = UniformLaplaceBeltrami(*mesh, v_it).norm();
        if (w < min) {
            min = w;
        }
        if (w > max) {
            max = w;
        }
        curvatures[v_it->idx()] = w;
    }

    std::vector<float> curvatures_sorted(mesh->n_vertices());
    std::copy(curvatures.begin(), curvatures.end(), curvatures_sorted.begin());
    std::sort(curvatures_sorted.begin(), curvatures_sorted.end());

    int lowerbound = ceil(curvatures.size() * 0.05);
    int upperbound = floor(curvatures.size() * 0.55);
    min = curvatures_sorted[lowerbound];
    max = curvatures_sorted[upperbound];

    float range = max - min;
    for ( int i = 0 ; i < curvatures.size() ; ++i ) {
        float w = curvatures[i];
        w = w - min;
        w = w / range;
        //w = fmin(w, 1);
        w = fmax(w, 0) * 255;
        mesh->set_color(mesh->vertex_handle(i), MyMesh::Color(w, w, w));
    }
}
std::pair<Eigen::Vector3f,
Eigen::Vector3f>  AxisAlignedBoundingBox(const MyMesh & mesh) {
    float maxx = std::numeric_limits<double>::min();
    float maxy = std::numeric_limits<double>::min();
    float maxz = std::numeric_limits<double>::min();

    float minx = std::numeric_limits<double>::max();
    float miny = std::numeric_limits<double>::max();
    float minz = std::numeric_limits<double>::max();

    for ( MyMesh::VertexIter v_it = mesh.vertices_begin();
         v_it!= mesh.vertices_end();
         ++v_it) {
        const MyMesh::Point & p = mesh.point(v_it);
        if (p[0] > maxx ) {
            maxx = p[0];
        }
        if (p[0] < minx) {
            minx = p[0];
        }

        if (p[1] > maxy) {
            maxy = p[1];
        }
        if (p[1] < miny) {
            miny = p[1];
        }

        if (p[2] > maxz) {
            maxz = p[2];
        }
        if (p[2] < minz) {
            minz = p[2];
        }
    }
    return std::make_pair(Eigen::Vector3f(minx, miny, minz),
                          Eigen::Vector3f(maxx, maxy, maxz));
}

void ToFirstQuadrant(MyMesh * mesh) {
    auto bb = AxisAlignedBoundingBox(*mesh);
    Substract(mesh, bb.first);
    Eigen::Vector3f diff = bb.second - bb.first;
    Eigen::Vector3f normalizer(1/diff(0), 1/diff(1), 1/diff(2));
    Scale(mesh, normalizer);
    
}

std::vector<float> GetColors(const MyMesh & mesh) {
    std::vector<float> colors(4 * mesh.n_vertices());
    for ( MyMesh::ConstVertexIter v_it = mesh.vertices_begin();
         v_it!= mesh.vertices_end();
         ++v_it) {
        int i = v_it->idx();
        const MyMesh::Color & c = mesh.color(v_it);
        colors[i*4] = c[0] / 255.0;
        colors[i*4 + 1] = c[1] / 255.0;
        colors[i*4 + 2] = c[2] / 255.0;
        colors[i*4 + 3] = 1; // alpha
    }
    return colors;
}

std::map<unsigned, float> UniformLaplaceBeltramiWeights(const MyMesh & m, const MyMesh::VertexHandle & vh) {
    std::map<unsigned, float> ret;
    std::vector<long> nbs = VertexNeighbors(m, vh);
    if(nbs.size() == 0) {
        printf("Unconnected vertex here\n");
    } else {
        float neighborhood_area = NeighborhoodArea(m, vh);
        assert(neighborhood_area > 1e-9);
        float w = -1.0 / nbs.size();
        for (auto nb_idx : nbs) {
            ret[static_cast<int>(nb_idx)] = w / neighborhood_area;
        }
        ret[vh.idx()] = 1 / neighborhood_area;
    }
    return ret;
}
Eigen::Vector3f UniformLaplaceBeltrami(const MyMesh & m, const MyMesh::VertexHandle & vh) {
    Eigen::Vector3f ret = Eigen::Vector3f::Zero();
    std::map<unsigned, float> weights = UniformLaplaceBeltramiWeights(m, vh);
    for (std::map<unsigned, float>::iterator it = weights.begin() ;
         it != weights.end() ;
         ++it) {
        unsigned idx = it->first;
        float w = it->second;
        auto t = w * m.point(m.vertex_handle(idx));
        Eigen::Vector3f foo(t[0], t[1], t[2]);
        ret += foo;
    }
    return ret;
}



