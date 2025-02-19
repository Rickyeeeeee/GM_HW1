#include "model.h"

Model::Model(const std::string &path) {
  meshRenderer = std::make_unique<MeshRenderer>();
  this->setPolyhedron(Polyhedron::createPolyhedron(path));
}

Model::Model() { meshRenderer = std::make_unique<MeshRenderer>(); };

void Model::setPolyhedron(Polyhedron *poly) {
  polyhedron.reset(poly);
  polyhedron->initialize();

  MeshProcessor::calcEdgeLength(polyhedron.get());
  MeshProcessor::calcBoundingSphere(polyhedron.get());
  MeshProcessor::calcFaceNormalsAndArea(polyhedron.get());
  MeshProcessor::calcVertNormals(polyhedron.get());

  meshRenderer->setupBuffers(poly);
  meshRenderer->setNormalMode(poly, false);
  meshRenderer->setColors(poly, 0);
}
