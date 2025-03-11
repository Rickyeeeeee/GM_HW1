#include "meshprocessor.h"
#include "learnply.h"
#include <queue>
#include <set>

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

  auto vo0 = rayOrigin - v0;
  auto v10 = v1 - v0;
  auto v20 = v2 - v0;
  auto p = rayDirection.cross(v20);
  auto q = vo0.cross(v10);
  t = q.dot(v20) / p.dot(v10);
  u = p.dot(vo0) / p.dot(v10);
  v = q.dot(rayDirection) / p.dot(v10);

  //Store the (u,v,t) in out
  out << u, v, t;
  return (u >= 0 && v >= 0 && u + v <= 1.1);
}

bool MeshProcessor::rayIntersectsSphere(const Eigen::Vector3f& rayOrigin, const Eigen::Vector3f& rayDirection, const Eigen::Vector3f& center, float radius)
{
    
	Eigen::Vector3f oc = rayOrigin - center;
	float a = rayDirection.dot(rayDirection);
	float b = 2.0 * oc.dot(rayDirection);
	float c = oc.dot(oc) - radius * radius;
	float discriminant = b * b - 4 * a * c;
	if (discriminant < 0) {
		return false;
	}
	else {
		return true;
	}

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
    int fliped_tri_counter = 0;
    /*
      TODO: Implement the function to pick the edge
      Hint:
        1. Start with a triangle
        2. Use the corners and the corners' opposite corners to find the adjacent triangles
        3. For each pair of triangles, compare the order of the vertices (corners) on the shared edge
        4. If the triangles have the opposite relative orientation, flip the triangle 
           - filpTriangle(Triangle* tri)
           - filped_tri_counter++
        5. Iteratively check all the triangles (BFS or DFS)
    */

	std::queue<Triangle*> q;
	std::set<int> visited;

	q.push(poly->tlist[0]);
	visited.insert(poly->tlist[0]->index);

	while (!q.empty()) {
		Triangle* tri = q.front();
		q.pop();
		for (int i = 0; i < 3; i++) {
			Corner* c = tri->corners[i];
			if (!c->oppsite) continue; // if the edge is on the boundary, skip it
			Corner* opp_c = c->oppsite;
			Triangle* adj_tri = opp_c->tri;
			if (visited.find(adj_tri->index) == visited.end()) {
				// Edge shared by two triangles
				Edge* e = c->edge;
				int v0 = e->verts[0]->index;
				int v1 = e->verts[1]->index;
				// Find the indeces on both triangles
				int v0_tri = -1;
				int v1_tri = -1;
				for (int j = 0; j < 3; j++) {
					if (tri->verts[j]->index == v0) {
						v0_tri = j;
					}
					if (tri->verts[j]->index == v1) {
						v1_tri = j;
					}
				}
				int v0_adj = -1;
				int v1_adj = -1;
				for (int j = 0; j < 3; j++) {
					if (adj_tri->verts[j]->index == v0) {
						v0_adj = j;
					}
					if (adj_tri->verts[j]->index == v1) {
						v1_adj = j;
					}
				}
				// Check the orientation

				bool isFlipped = (v0_tri + 1) % 3 == v1_tri;  // v0 ¡÷ v1 in tri (counter-clockwise)
				bool isFlippedAdj = (v0_adj + 1) % 3 == v1_adj; // v0 ¡÷ v1 in adj_tri (counter-clockwise)

				if (isFlipped == isFlippedAdj) { // If orientations differ
					filpTriangle(adj_tri);
					fliped_tri_counter++;
				}

				q.push(adj_tri);
				visited.insert(adj_tri->index);

			}
		}
	}	

    if (fliped_tri_counter > 0){
        std::cout << "Flip: " << fliped_tri_counter << " triangles" << std::endl;
        poly->recreate_corners();
        calcVertNormals(poly);
    }
    return fliped_tri_counter > 0;
}

/******************************************************************************
Filp the order of the vertics in the given triangle
******************************************************************************/
void MeshProcessor::filpTriangle(Triangle* tri)
{
    std::swap(tri->verts[0], tri->verts[2]);
    tri->normal = -tri->normal;
}
