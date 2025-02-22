#include "meshprocessor.h"
#include "learnply.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void MeshProcessor::calcVertNormals(Polyhedron *poly) {
  for (int i = 0; i < poly->nverts(); i++) {
    poly->vlist[i]->normal = Eigen::Vector3d(0.0, 0.0, 0.0);
    for (int j = 0; j < poly->vlist[i]->ntris(); j++) {
      poly->vlist[i]->normal += poly->vlist[i]->tris[j]->normal;
    }
    poly->vlist[i]->normal.normalize();
  }
}

void MeshProcessor::calcFaceNormalsAndArea(Polyhedron *poly) {
    for (int i = 0; i < poly->ntris(); i++) {
        Vertex* v0 = poly->tlist[i]->verts[0];
        Vertex* v1 = poly->tlist[i]->verts[1];
        Vertex* v2 = poly->tlist[i]->verts[2];
        poly->tlist[i]->normal = (v2->pos - v0->pos).cross(v1->pos - v0->pos);

        poly->tlist[i]->area = poly->tlist[i]->normal.norm() * 0.5;
        poly->tlist[i]->normal.normalize();
    }
}

void MeshProcessor::calcEdgeLength(Polyhedron *poly) {
  for (int i = 0; i < poly->nedges(); i++) {
    Vertex* v0 = poly->elist[i]->verts[0];
    Vertex* v1 = poly->elist[i]->verts[1];
    poly->elist[i]->length = (v1->pos - v0->pos).norm();
  }
}

/******************************************************************************
Check if the given ray intersects the triangle

Entry:
  rayOrigin - origin of the ray
  rayDirection - direction of the ray
  v0,v1,v2 - three vertices of the triangle
  out - output of (u,v,t)

Exit:
  return true if the ray intersects the triangle; otherwise, return false
******************************************************************************/
bool MeshProcessor::rayIntersectsTriangle(Eigen::Vector3f &rayOrigin, Eigen::Vector3f &rayDirection,
                                          Eigen::Vector3f &v0, Eigen::Vector3f &v1, Eigen::Vector3f& v2,
                                          Eigen::Vector3f &out) {
  float u = 0.0f, v = 0.0f, t = 0.0f;
  /*
  TODO: Implement the function to pick the edge
  */

  //Store the (u,v,t) in out
  out << u, v, t;
  return false;
}

/******************************************************************************
Make the triangles of the given polyhedron have the same relative orientation.

Entry:
  poly - pointer of polyhedron

Exit:
  return true if any triangle is filped; otherwise, return false
******************************************************************************/
bool MeshProcessor::fixOrientation(Polyhedron* poly)
{ 
    int filped_tri_counter = 0;
    /*
      TODO: Implement the function to pick the edge
      Hint:
        1. Start with a triangle
        2. Use the corners and the corners' opposite corners to find the adjacent triangles
        3. For each pair of triangles, compare the order of the vertices on the shared edge
           - checkEdgeDirection(Edge* edge, Vertex* v0, Vertex* v1)
        4. If the triangles have the opposite relative orientation, filp the triangle 
           - filpTriangle(Triangle* tri)
           - filped_tri_counter++
        5. Iteratively check all the triangles (BFS or DFS)
    */

    if (filped_tri_counter > 0){
        std::cout << "Filp: " << filped_tri_counter << " triangles" << std::endl;
        poly->recreate_corners();
        calcVertNormals(poly);
    }
    return filped_tri_counter > 0;
}

/******************************************************************************
Check if the edge has the same direction from v0 to v1

Entry:
  poly - pointer of polyhedron

Exit:
  return true if any triangle is filped; otherwise, return false
******************************************************************************/
bool MeshProcessor::checkEdgeDirection(Edge* edge, Vertex* v0, Vertex* v1)
{
    return (edge->verts[0] == v0 && edge->verts[1] == v1);
}

/******************************************************************************
Filp the order of the vertics in the given triangle
******************************************************************************/
void MeshProcessor::filpTriangle(Triangle* tri)
{
    std::swap(tri->verts[0], tri->verts[2]);
    tri->normal = -tri->normal;
}
