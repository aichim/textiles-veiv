//
//  mesh.h
//  MViewer
//
//  Created by Lukas Murmann on 2/26/13.
//
//

#ifndef __MViewer__mesh__
#define __MViewer__mesh__


#include <Eigen/Dense>
typedef Eigen::MatrixXf Pointset;
#include <OpenMesh/Core/IO/MeshIO.hh>

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
typedef std::shared_ptr<MyMesh> MeshPtr;


template<typename T>
Eigen::Vector3f ToEigen(T t) {
    return Eigen::Vector3f(t[0], t[1], t[2]);
}

long                        GetNumOfElements(const MyMesh &) ;
Pointset                    GetVertices(const MyMesh &);
long                        GetNumOfVertices(const MyMesh & mesh);
Pointset                    GetNormals(const MyMesh &);
std::vector<float>          GetColors(const MyMesh &);
Pointset                    GetTexCoords(const MyMesh &);
std::vector<unsigned int>   GetTriangleIndices(const MyMesh &) ;


Pointset                    GetFaceVertices(const MyMesh &, const MyMesh::FaceHandle &);


Pointset                    GetVerticesFlat(const MyMesh &);
Pointset                    GetNormalsFlat( const MyMesh & mesh );
Pointset                    GetTexCoordsFlat(const MyMesh & mesh );
std::vector<unsigned int>   GetTriangleIndicesFlat(const MyMesh &) ;

void                        SetVertices(MyMesh *, const Pointset & vertices );
void                        SetPoint(MyMesh *, int index, Eigen::Vector3f vertex);
void                        SetNormals(MyMesh & mesh, const Pointset normals );

Eigen::Vector2f             GetBoundingBox(const MyMesh &mesh);
Eigen::Vector3f             GetCentroid( const MyMesh & mesh );

MyMesh                      Merge(const MyMesh & a, const MyMesh & b);
void                        Cat(MyMesh * base, const MyMesh & b);

void                        ToFirstQuadrant(MyMesh * m);
std::pair<Eigen::Vector3f,
          Eigen::Vector3f>  AxisAlignedBoundingBox(const MyMesh &);


void                        Scale(MyMesh * mesh, float s);
void                        Scale(MyMesh * mesh, const Eigen::Vector3f & v);
void                        Add(MyMesh * mesh, const Eigen::Vector3f & v);
void                        Substract(MyMesh * mesh, const Eigen::Vector3f & v);
void                        Normalize( MyMesh * mesh );
void                        Center( MyMesh * mesh );

int                         NumAdjacentFaces (const MyMesh & mesh, const MyMesh::VertexHandle & h);
std::vector<long>           VertexNeighbors (const MyMesh & mesh, const MyMesh::VertexHandle &);
std::vector<long>           VertexNeighborEdges (const MyMesh & mesh, const MyMesh::VertexHandle &);

float                       FaceArea(const MyMesh & mesh, const MyMesh::FaceHandle &);
float                       NeighborhoodArea(const MyMesh & mesh, const MyMesh::VertexHandle &);
void                        SetMeanCurvature(MyMesh*);

void                        FaceRadNeighbors(MyMesh *mesh, const long center, const float radius,
                                             const OpenMesh::FPropHandleT<Eigen::Vector3f> & face_centroids,
                                             OpenMesh::FPropHandleT<float> & face_distances,
                                             std::vector<long> & neighbors);
void                        VertexRadNeighbors(MyMesh * mesh, const long center,
                                               const float sqRadius, std::vector<long> & neighbors);
void                        VertexKNeighbors(MyMesh * mesh, long vertex_idx,
                                            int depth, std::vector<long> & neighbors);
void                        clearAllVisitedStatus(MyMesh * mesh);
void                        clearFaceVisitedStatus(MyMesh * mesh, const std::vector<long> & list);

// Returns empty matrix if face does not exist (boundary edge)
Pointset                    GetFace(const MyMesh & mesh, const MyMesh::EdgeHandle & h, int v_idx);

Eigen::Vector3f             GetEdgeVertex (const MyMesh & mesh, const MyMesh::EdgeHandle & h, int v_idx);
Eigen::Vector3f             GetVertex(const MyMesh & mesh, long v_idx);
Eigen::Vector3f             GetVNormal(const MyMesh & mesh, long v_idx);
Eigen::Vector3f             GetFNormal(const MyMesh & mesh, long v_idx);
Eigen::Vector3f             GetFaceCentroid(const MyMesh & mesh, const long f_idx);
float                       ComputeSumOfDistances( const MyMesh & a, const MyMesh & d );

long                        NumBoundaryVertices( const MyMesh & m);
long                        NumBoundary1RingVertices(const MyMesh & m);
bool                        is_in_boundary_onering(const MyMesh & m, const MyMesh::VertexHandle &vh);

Eigen::Vector3f             UniformLaplaceBeltrami(const MyMesh &, const MyMesh::VertexHandle &);
std::map<unsigned, float>   UniformLaplaceBeltramiWeights(const MyMesh &, const MyMesh::VertexHandle &);

std::vector<float>          ToVector(const Pointset &);


#endif /* defined(__MViewer__mesh__) */
