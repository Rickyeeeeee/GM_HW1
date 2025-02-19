#include "learnply.h"
#include "learnply_io.h"
#include "meshprocessor.h"
#include "ply.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Polyhedron::Polyhedron() : orientation(0), center(0.0, 0.0, 0.0), radius(0.0), area(0.0) {}

Polyhedron::Polyhedron(std::vector<Vertex *> &verts, std::vector<Triangle *> &tris, bool re_index)
    : orientation(0), center(0.0, 0.0, 0.0), radius(0.0), area(0.0) {
  vlist = verts;
  tlist = tris;
  if (re_index) {
    /* index the vertices and triangles */
    for (int i = 0; i < nverts(); i++) {
      vlist[i]->index = i;
    }
    for (int i = 0; i < ntris(); i++) {
      tlist[i]->index = i;
    }
  }
}

/******************************************************************************
Read in a polyhedron from a file.
******************************************************************************/
Polyhedron *Polyhedron::createPolyhedron(const std::string &filename) {
  Polyhedron *poly = new Polyhedron();

  FILE *file = fopen(filename.c_str(), "r");

  int i, j;
  int elem_count;
  char *elem_name;

  /*** Read in the original PLY object ***/

  PlyFile *in_ply = read_ply(file);

  for (i = 0; i < in_ply->num_elem_types; i++) {
    /* prepare to read the i'th list of elements */
    elem_name = setup_element_read_ply(in_ply, i, &elem_count);

    if (equal_strings("vertex", elem_name)) {
      /* create a vertex list to hold all the vertices */
      poly->vlist.resize(elem_count, NULL);
      /* set up for getting vertex elements */
      setup_property_ply(in_ply, &vert_props[0]);
      setup_property_ply(in_ply, &vert_props[1]);
      setup_property_ply(in_ply, &vert_props[2]);
      /* grab all the vertex elements */
      for (j = 0; j < poly->nverts(); j++) {
        Vertex_io vert;
        get_element_ply(in_ply, (void *)&vert);
        /* copy info from the "vert" structure */
        poly->vlist[j] = new Vertex(vert.x, vert.y, vert.z);
      }
    } else if (equal_strings("face", elem_name)) {
      /* create a list to hold all the face elements */
      poly->tlist.resize(elem_count, NULL);
      /* set up for getting face elements */
      setup_property_ply(in_ply, &face_props[0]);

      /* grab all the face elements */
      for (j = 0; j < elem_count; j++) {
        Face_io face;
        get_element_ply(in_ply, (void *)&face);

        if (face.nverts != 3) {
          std::cout << "Face has" << face.nverts << " vertices (should be three).\n";
          delete poly;
          exit(-1);
        }

        /* copy info from the "face" structure */
        poly->tlist[j] = new Triangle;
        poly->tlist[j]->verts[0] = (Vertex *)face.verts[0];
        poly->tlist[j]->verts[1] = (Vertex *)face.verts[1];
        poly->tlist[j]->verts[2] = (Vertex *)face.verts[2];
      }
    }
  }

  /* close the file */
  close_ply(in_ply);

  /* fix up vertex pointers in triangles */
  for (i = 0; i < poly->ntris(); i++) {
    poly->tlist[i]->verts[0] = poly->vlist[reinterpret_cast<std::size_t>(poly->tlist[i]->verts[0])];
    poly->tlist[i]->verts[1] = poly->vlist[reinterpret_cast<std::size_t>(poly->tlist[i]->verts[1])];
    poly->tlist[i]->verts[2] = poly->vlist[reinterpret_cast<std::size_t>(poly->tlist[i]->verts[2])];
  }

  /* get rid of triangles that use the same vertex more than once */
  for (std::vector<Triangle *>::iterator it = poly->tlist.begin(); it != poly->tlist.end();) {
    Triangle *tri = *it;
    Vertex *v0 = tri->verts[0];
    Vertex *v1 = tri->verts[1];
    Vertex *v2 = tri->verts[2];

    if (v0 == v1 || v1 == v2 || v2 == v0) {
      delete tri;
      it = poly->tlist.erase(it);
    } else {
      it++;
    }
  }

  /* index the vertices and triangles */
  for (int i = 0; i < poly->nverts(); i++) {
    poly->vlist[i]->index = i;
  }
  for (int i = 0; i < poly->ntris(); i++) {
    poly->tlist[i]->index = i;
  }

  fclose(file);

  float PI = 3.14159265358979323846f;
  Eigen::Matrix3d rotX = Eigen::AngleAxisd(-PI / 2.0, Eigen::Vector3d::UnitX()).matrix();
  for (auto &v : poly->vlist) {
    v->pos = rotX * v->pos;
  }

  return poly;
}

void Polyhedron::initialize() {
  /* create pointers from vertices to triangles */
  set_vertex_to_tri_ptrs();
  /* make edges */
  create_edges();
  /* order the pointers from vertices to faces */
  for (int i = 0; i < nverts(); i++) {
    order_vertex_to_tri_ptrs(vlist[i]);
  }
  /* make corners */
  create_corners();
}

void Polyhedron::finalize() {
  int i;
  for (i = 0; i < ntris(); i++) {
    free(tlist[i]);
  }
  for (i = 0; i < nedges(); i++) {
    elist[i]->tris.clear();
    free(elist[i]);
  }
  for (i = 0; i < nverts(); i++) {
    vlist[i]->tris.clear();
    free(vlist[i]);
  }
  tlist.clear();
  vlist.clear();
  elist.clear();
}

/******************************************************************************
Find out if there is another face that shares an edge with a given face.

Entry:
  f1    - face that we're looking to share with
  v1,v2 - two vertices of f1 that define edge

Exit:
  return the matching face, or NULL if there is no such face
******************************************************************************/
Triangle *Polyhedron::find_edge(Triangle *f1, Vertex *v1, Vertex *v2) {
  int i, j;
  Triangle *f2;
  Triangle *adjacent = NULL;

  /* look through all faces of the first vertex */

  for (i = 0; i < v1->ntris(); i++) {
    f2 = v1->tris[i];
    if (f2 == f1)
      continue;
    /* examine the vertices of the face for a match with the second vertex */
    for (j = 0; j < 3; j++) {
      /* look for a match */
      if (f2->verts[j] == v2) {
        /* if we've got a match, return this face */
        return (f2);
      }
    }
  }
  return (adjacent);
}

/******************************************************************************
Create an edge.

Entry:
  v1,v2 - two vertices of f1 that define edge
******************************************************************************/
void Polyhedron::create_edge(Vertex *v1, Vertex *v2) {
  int i, j;
  Triangle *f;

  /* create the edge */
  Edge *e = new Edge;
  e->index = nedges();
  e->verts[0] = v1;
  e->verts[1] = v2;
  elist.push_back(e);

  /* count all triangles that will share the edge, and do this */
  /* by looking through all faces of the first vertex */
  int ntris = 0;
  for (i = 0; i < v1->ntris(); i++) {
    f = v1->tris[i];
    /* examine the vertices of the face for a match with the second vertex */
    for (j = 0; j < 3; j++) { /* look for a match */
      if (f->verts[j] == v2) {
        ntris++;
        break;
      }
    }
  }

  /* make room for the face pointers (at least two) */
  e->tris.resize(ntris);

  /* create pointers from edges to faces and vice-versa */
  int tidx = 0;
  for (i = 0; i < v1->ntris(); i++) {
    f = v1->tris[i];
    /* examine the vertices of the face for a match with the second vertex */
    for (j = 0; j < 3; j++) {
      if (f->verts[j] == v2) {
        e->tris[tidx] = f;
        tidx++;

        if (f->verts[(j + 1) % 3] == v1) {
          f->edges[j] = e;
        } else if (f->verts[(j + 2) % 3] == v1) {
          f->edges[(j + 2) % 3] = e;
        } else {
          fprintf(stderr, "Non-recoverable inconsistancy in create_edge()\n");
          exit(-1);
        }
        break; /* we'll only find one instance of v2 */
      }
    }
  }
}

/******************************************************************************
Create edges.
******************************************************************************/
void Polyhedron::create_edges() {
  /* create all the edges by examining all the triangles */
  for (int i = 0; i < ntris(); i++) {
    Triangle *f = tlist[i];
    for (int j = 0; j < 3; j++) {
      if (f->edges[j]) {
        continue;
      } /* skip over edges that we've already created */
      Vertex *v1 = f->verts[j];
      Vertex *v2 = f->verts[(j + 1) % 3];
      create_edge(v1, v2);
    }
  }
}

/******************************************************************************
Create Corners.
******************************************************************************/
void Polyhedron::create_corners() {
  for (int i = 0; i < ntris(); i++) {
    Triangle *tri = tlist[i];

    // add three new corners
    Corner *corner0 = new Corner();
    Corner *corner1 = new Corner();
    Corner *corner2 = new Corner();

    clist.push_back(corner0);
    clist.push_back(corner1);
    clist.push_back(corner2);

    // create properties of first corner
    corner0->tri = tri;
    tri->corners[0] = corner0;
    corner0->vertex = tri->verts[0];
    corner0->vertex->corners.push_back(corner0);
    corner0->edge = tri->edges[1];
    corner0->edge->corners.push_back(corner0);
    corner0->prev = corner2;
    corner0->next = corner1;

    // create properties of second corner
    corner1->tri = tri;
    tri->corners[1] = corner1;
    corner1->vertex = tri->verts[1];
    corner1->vertex->corners.push_back(corner1);
    corner1->edge = tri->edges[2];
    corner1->edge->corners.push_back(corner1);
    corner1->prev = corner0;
    corner1->next = corner2;

    // create properties of third corner
    corner2->tri = tri;
    tri->corners[2] = corner2;
    corner2->vertex = tri->verts[2];
    corner2->vertex->corners.push_back(corner2);
    corner2->edge = tri->edges[0];
    corner2->edge->corners.push_back(corner2);
    corner2->prev = corner1;
    corner2->next = corner0;
  }

  // find opposite corners
  for (int i = 0; i < nedges(); i++) {
    Edge *e = elist[i];
    Corner *c0 = e->corners[0];
    if (c0->oppsite != NULL) {
      continue;
    }
    if (e->ntris() > 1) {
      Corner *c1 = e->corners[1];
      c0->oppsite = c1;
      c1->oppsite = c0;
    }
  }
}

/******************************************************************************
Create pointers from vertices to faces.
******************************************************************************/
void Polyhedron::set_vertex_to_tri_ptrs() {
  for (int i = 0; i < ntris(); i++) {
    Triangle *f = tlist[i];
    for (int j = 0; j < 3; j++) {
      Vertex *v = f->verts[j];
      v->tris.push_back(f);
    }
  }
}

/******************************************************************************
Find the other triangle that is incident on an edge, or NULL if there is
no other.
******************************************************************************/
Triangle *Polyhedron::find_other_triangle(Edge *edge, Triangle *tri) {
  /* search for any other triangle */
  for (int i = 0; i < edge->ntris(); i++)
    if (edge->tris[i] != tri)
      return (edge->tris[i]);

  /* there is no such other triangle if we get here */
  return (NULL);
}

/******************************************************************************
Order the pointers to faces that are around a given vertex.

Entry:
  v - vertex whose face list is to be ordered
******************************************************************************/
void Polyhedron::order_vertex_to_tri_ptrs(Vertex *v) {
  int i, j;
  Triangle *f;
  Triangle *fnext;
  int nf;
  int vindex;
  int boundary;
  int count;

  nf = v->ntris();
  f = v->tris[0];

  /* go backwards (clockwise) around faces that surround a vertex */
  /* to find out if we reach a boundary */

  boundary = 0;

  for (i = 1; i <= nf; i++) {

    /* find reference to v in f */
    vindex = -1;
    for (j = 0; j < 3; j++)
      if (f->verts[j] == v) {
        vindex = j;
        break;
      }

    /* error check */
    if (vindex == -1) {
      fprintf(stderr, "can't find vertex #1\n");
      exit(-1);
    }

    /* corresponding face is the previous one around v */
    fnext = find_other_triangle(f->edges[vindex], f);

    /* see if we've reached a boundary, and if so then place the */
    /* current face in the first position of the vertice's face list */

    if (fnext == NULL) {
      /* find reference to f in v */
      for (j = 0; j < v->ntris(); j++)
        if (v->tris[j] == f) {
          v->tris[j] = v->tris[0];
          v->tris[0] = f;
          break;
        }
      boundary = 1;
      break;
    }

    f = fnext;
  }

  /* now walk around the faces in the forward direction and place */
  /* them in order */

  f = v->tris[0];
  count = 0;

  for (i = 1; i < nf; i++) {

    /* find reference to vertex in f */
    vindex = -1;
    for (j = 0; j < 3; j++)
      if (f->verts[(j + 1) % 3] == v) {
        vindex = j;
        break;
      }

    /* error check */
    if (vindex == -1) {
      fprintf(stderr, "can't find vertex #2\n");
      exit(-1);
    }

    /* corresponding face is next one around v */
    fnext = find_other_triangle(f->edges[vindex], f);

    /* break out of loop if we've reached a boundary */
    count = i;
    if (fnext == NULL) {
      break;
    }

    /* swap the next face into its proper place in the face list */
    for (j = 0; j < v->ntris(); j++)
      if (v->tris[j] == fnext) {
        v->tris[j] = v->tris[i];
        v->tris[i] = fnext;
        break;
      }

    f = fnext;
  }
}
