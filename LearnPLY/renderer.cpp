#include "renderer.h"

Renderer::Renderer(int width, int height, Scene *scenePtr)
    : screenWidth(width), screenHeight(height), scene(scenePtr), plyShader(nullptr),
      colorShader(nullptr), textShader(nullptr), axesShader(nullptr) {
  if (!scene) {
    throw std::runtime_error("Scene pointer is null");
  }
  if (!scene->getCamera()) {
    throw std::runtime_error("Camera is null");
  }

  setupShaders();

  // Initialize OpenGL state
  glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
  glEnable(GL_DEPTH_TEST);

  vertexHelper = std::make_unique<VertexHelper>();
  vertexHelper->initialize(colorShader, textShader);

  edgeHelper = std::make_unique<EdgeHelper>();
  edgeHelper->initialize(colorShader);

  axesHelper = std::make_unique<AxesHelper>();
  axesHelper->initialize(axesShader, 1.0f);
}

Renderer::~Renderer() {}

void Renderer::setupShaders() {
  plyShader = new Shader("../shaders/plyShader.vs.glsl", "../shaders/plyShader.fs.glsl"); // Default
  colorShader = new Shader("../shaders/colorShader.vs.glsl",
                               "../shaders/colorShader.fs.glsl"); // Wireframe
  textShader = new Shader("../shaders/textShader.vs.glsl", "../shaders/textShader.fs.glsl");
  axesShader = new Shader("../shaders/axesShader.vs.glsl", "../shaders/axesShader.fs.glsl");
}

void Renderer::releaseShaders() {
  if (plyShader != nullptr) {
    delete plyShader;
    plyShader = nullptr;
  }
  if (colorShader != nullptr) {
    delete colorShader;
    colorShader = nullptr;
  }
  if (textShader != nullptr) {
    delete textShader;
    textShader = nullptr;
  }
  if (axesShader != nullptr) {
    delete axesShader;
    axesShader = nullptr;
  }
}

void Renderer::render() {
  if (!scene || !scene->getModel()) {
    return;
  }

  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Regular render
  const Camera *cam = scene->getCamera();
  const Eigen::Matrix4f &projection = cam->projectionMatrix();
  const Eigen::Matrix4f &view = cam->viewMatrix().matrix();

  bool drawWireframe = scene->getDrawWireframe();
  bool drawFaceNormal = scene->getDrawFaceNormal();
  bool drawVertexLabel = scene->getDrawVertexLabel();
  plyShader->use();
  plyShader->setMat4("projection_matrix", projection);
  plyShader->setMat4("view_matrix", view);
  plyShader->setMat4("model_matrix", Eigen::Matrix4f::Identity());
  plyShader->setVec3("view_dir", cam->direction().cast<float>());
  scene->getModel()->getMeshRenderer()->drawPLY();
  if (drawWireframe) {
    colorShader->use();
    colorShader->setVec3("color", 0.0f, 0.0f, 0.0f);
    colorShader->setMat4("projection_matrix", projection);
    colorShader->setMat4("view_matrix", view);
    colorShader->setMat4("model_matrix", Eigen::Matrix4f::Identity());
    scene->getModel()->getMeshRenderer()->drawWireframe();
  }

  // Vertex helper
  vertexHelper->use(projection, view);
  for (auto &v : scene->getModel()->getPolyhedron()->vlist) {
    if (!v->selected) {
      continue;
    }
    Eigen::Vector3f pos = v->pos.cast<float>();
    vertexHelper->draw(pos);
  }

  if (drawVertexLabel) {
      vertexHelper->useLabel(projection, view);
      for (auto& v : scene->getModel()->getPolyhedron()->vlist) {
          if (!v->selected) {
              continue;
          }
          Eigen::Vector3f pos = v->pos.cast<float>();
          Eigen::Vector3f normal = v->normal.cast<float>();
          vertexHelper->drawLabel(pos, normal, std::to_string(v->index).c_str());
      }
  }

  // Edge helper
  edgeHelper->use(projection, view);
  for (auto &e : scene->getModel()->getPolyhedron()->elist) {
    if (!e->selected) {
      continue;
    }
    Eigen::Vector3f start = e->verts[0]->pos.cast<float>();
    Eigen::Vector3f end = e->verts[1]->pos.cast<float>();
    edgeHelper->draw(start, end);
  }

  // Axes helper
  axesHelper->draw(view, projection,
                   static_cast<float>(screenWidth) / static_cast<float>(screenHeight));

  glUseProgram(0);
}

void Renderer::resize(int width, int height) {
  screenWidth = width;
  screenHeight = height;
}
