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
  Eigen::Vector3d v0, v1, v2;
  double length[3];

  poly->area = 0.0;
  for (int i = 0; i < poly->ntris(); i++) {
    for (int j = 0; j < 3; j++) {
      length[j] = poly->tlist[i]->edges[j]->length;
    }
    double temp_s = (length[0] + length[1] + length[2]) * 0.5;
    poly->tlist[i]->area =
        sqrt(temp_s * (temp_s - length[0]) * (temp_s - length[1]) * (temp_s - length[2]));

    poly->area += poly->tlist[i]->area;
    v1 << poly->vlist[poly->tlist[i]->verts[0]->index]->pos[0],
          poly->vlist[poly->tlist[i]->verts[0]->index]->pos[1],
          poly->vlist[poly->tlist[i]->verts[0]->index]->pos[2];
    v2 << poly->vlist[poly->tlist[i]->verts[1]->index]->pos[0],
          poly->vlist[poly->tlist[i]->verts[1]->index]->pos[1],
          poly->vlist[poly->tlist[i]->verts[1]->index]->pos[2];
    v0 << poly->vlist[poly->tlist[i]->verts[2]->index]->pos[0],
          poly->vlist[poly->tlist[i]->verts[2]->index]->pos[1],
          poly->vlist[poly->tlist[i]->verts[2]->index]->pos[2];
    poly->tlist[i]->normal = (v0 - v1).cross(v2 - v1);
    poly->tlist[i]->normal.normalize();
  }

  double signedVolume = 0.0;
  Eigen::Vector3d test = poly->center;
  for (int i = 0; i < poly->ntris(); i++) {
    const Eigen::Vector3d &cent = poly->vlist[poly->tlist[i]->verts[0]->index]->pos;
    signedVolume += (test - cent).dot(poly->tlist[i]->normal) * poly->tlist[i]->area;
  }
  signedVolume /= poly->area;
  if (signedVolume < 0) {
    poly->orientation = 0;
  } else {
    poly->orientation = 1;
    for (int i = 0; i < poly->ntris(); i++) {
      poly->tlist[i]->normal *= -1.0;
    }
  }
}

void MeshProcessor::calcBoundingSphere(Polyhedron *poly) {
  Eigen::Vector3d minV, maxV;
  for (int i = 0; i < poly->nverts(); i++) {
    if (i == 0) {
      minV = poly->vlist[i]->pos;
      maxV = poly->vlist[i]->pos;
    } else {
      if (poly->vlist[i]->pos[0] < minV[0]) {
        minV[0] = poly->vlist[i]->pos[0];
      }
      if (poly->vlist[i]->pos[0] > maxV[0]) {
        maxV[0] = poly->vlist[i]->pos[0];
      }
      if (poly->vlist[i]->pos[1] < minV[1]) {
        minV[1] = poly->vlist[i]->pos[1];
      }
      if (poly->vlist[i]->pos[1] > maxV[1]) {
        maxV[1] = poly->vlist[i]->pos[1];
      }
      if (poly->vlist[i]->pos[2] < minV[2]) {
        minV[2] = poly->vlist[i]->pos[2];
      }
      if (poly->vlist[i]->pos[2] > maxV[2]) {
        maxV[2] = poly->vlist[i]->pos[2];
      }
    }
  }
  poly->center = (minV + maxV) * 0.5;
  poly->radius = (poly->center - minV).norm();
}

void MeshProcessor::calcEdgeLength(Polyhedron *poly) {
  Eigen::Vector3d v1, v2;
  for (int i = 0; i < poly->nedges(); i++) {
    v1 << poly->elist[i]->verts[0]->pos[0], poly->elist[i]->verts[0]->pos[1],
        poly->elist[i]->verts[0]->pos[2];
    v2 << poly->elist[i]->verts[1]->pos[0], poly->elist[i]->verts[1]->pos[1],
        poly->elist[i]->verts[1]->pos[2];
    poly->elist[i]->length = (v1 - v2).norm();
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
  /*
  TODO: Implement the function to pick the edge
  */

  // HINT: Store the (u,v,t) in out

  return false;
};