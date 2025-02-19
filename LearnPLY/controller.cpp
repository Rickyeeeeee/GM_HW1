#include "controller.h"
#include "learnply.h"
#include <cstddef>

Controller::Controller(int width, int height, GLFWwindow *window, Scene *scenePtr)
    : screenWidth(width), screenHeight(height), isDragging(false), scene(scenePtr) {
  // Trackball initialization
  trackball = std::make_unique<Trackball>();
  trackball->setCamera(scene->getCamera());
  trackball->start(Trackball::Around);
  // GUI Initialization
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.FontGlobalScale = 2.0f;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 130");
}

void Controller::resize(int width, int height) {
  screenWidth = width;
  screenHeight = height;
}

void Controller::handleMouseClick(int button, double xpos, double ypos, int action) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    isDragging = (action == GLFW_PRESS);
    if (isDragging) {
      trackball->start(Trackball::Around);
    }
  } else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
    if (selectionMode == (int)SelectType::Faces) {
      selectFaces(xpos, ypos);
    } else if (selectionMode == (int)SelectType::Vertices) {
      selectVertices(xpos, ypos);
    } else if (selectionMode == (int)SelectType::Edges) {
      selectEdges(xpos, ypos);
    }
  }
}

void Controller::handleMouseMove(double xpos, double ypos) {
  if (isDragging) {
    trackball->track(Eigen::Vector2i(xpos, ypos));
  }
}

void Controller::handleKeyPress(int key) {
  auto camera = scene->getCamera();
  if (key == GLFW_KEY_W) {
    camera->zoom(ZOOM_SPEED);
  } else if (key == GLFW_KEY_S) {
    camera->zoom(-ZOOM_SPEED);
  } else if (key == GLFW_KEY_A) {
    camera->localTranslate(Eigen::Vector3f(-PAN_SPEED, 0, 0));
  } else if (key == GLFW_KEY_D) {
    camera->localTranslate(Eigen::Vector3f(PAN_SPEED, 0, 0));
  } else if (key == GLFW_KEY_Z) {
    camera->localTranslate(Eigen::Vector3f(0, PAN_SPEED, 0));
  } else if (key == GLFW_KEY_X) {
    camera->localTranslate(Eigen::Vector3f(0, -PAN_SPEED, 0));
  }
}

void Controller::handleMouseScroll(double xoffset, double yoffset) {
  auto camera = scene->getCamera();
  if (camera) {
    camera->zoom((float)yoffset * ZOOM_SPEED);
  }
}

void Controller::render() {
  // Render GUI
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGui::Begin("Control Panel", NULL, ImGuiWindowFlags_AlwaysAutoResize);

  // Misc controls
  if (ImGui::CollapsingHeader("Misc.")) {
    if (ImGui::Button("Screenshot"))
      saveScreenshot();
    if (ImGui::Button("Save Camera Position"))
      exportCameraPosition();
    if (ImGui::Button("Load Camera Position"))
      importCameraPosition();
    ImGui::Separator();
  }

  // Visualization controls
  if (ImGui::CollapsingHeader("Visualization")) {

    ImGui::Text("Selected Model");
    const std::string &str = scene->modelNames[scene->selectedModel];
    if (ImGui::BeginCombo("##Model", str.c_str())) {
      for (int n = 0; n < scene->modelNames.size(); n++) {
        bool is_selected = (scene->selectedModel == n);
        if (ImGui::Selectable(scene->modelNames[n].c_str(), is_selected)) {
          scene->loadModel(n);
        }
        if (is_selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }

    if (ImGui::Checkbox("Using Face Normal", &scene->drawFaceNormal)) {
      MeshRenderer *meshRenderer = scene->getModel()->getMeshRenderer();
      meshRenderer->setNormalMode(scene->getModel()->getPolyhedron(), scene->drawFaceNormal);
    }
    ImGui::Checkbox("Wireframe", &scene->drawWireframe);
    ImGui::Checkbox("Vertex Labels", &scene->drawVertexLabel);
    ImGui::Separator();
  }

  // Selection and processing controls
  if (ImGui::CollapsingHeader("Selection")) {
    ImGui::Text("Selection Mode");
    ImGui::Combo("##Selection Mode", &selectionMode, "None\0Faces\0Vertices\0Edges\0");
    if (ImGui::Button("Find Closures"))
      findClosures();
    if (ImGui::Button("Find Links"))
      findLinks();
    if (ImGui::Button("Find Stars"))
      findStars();
    if (ImGui::Button("Find Boundaries"))
      findBoundaries();
    if (ImGui::Button("Find Interiors"))
      findInteriors();
    if (ImGui::Button("Clear Selections"))
      clearSelections();
  }

  ImGui::End();
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Controller::clearSelections() {
  for (auto &v : scene->getModel()->getPolyhedron()->vlist) {
    v->selected = false;
  }
  for (auto &e : scene->getModel()->getPolyhedron()->elist) {
    e->selected = false;
  }
  for (auto &t : scene->getModel()->getPolyhedron()->tlist) {
    t->selected = false;
  }
  scene->getModel()->getMeshRenderer()->updateColors(scene->getModel()->getPolyhedron());
}

Eigen::Vector3f Controller::calculateNearPlanePoint(const double &x, const double &y, int height,
                                                    Camera *camera) {
  float near = camera->nearDist();
  Eigen::Vector2f point2D(x, height - y);
  Eigen::Vector3f pt = camera->unProject(point2D, near);

  return pt;
}
void Controller::saveScreenshot() {
  std::vector<unsigned char> pixels(screenWidth * screenHeight * 3);
  glReadPixels(0, 0, screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "screenshot_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".ppm";

  std::ofstream file(ss.str(), std::ios::binary);
  if (!file) {
    std::cerr << "Failed to open file for writing: " << ss.str() << std::endl;
    return;
  }

  file << "P6\n" << screenWidth << " " << screenHeight << "\n255\n";

  for (int y = screenHeight - 1; y >= 0; y--) {
    for (int x = 0; x < screenWidth; x++) {
      int idx = (y * screenWidth + x) * 3;
      file.write(reinterpret_cast<char *>(&pixels[idx]), 3);
    }
  }

  file.close();
  std::cout << "Screenshot saved to: " << ss.str() << std::endl;
}

void Controller::exportCameraPosition(const std::string &filename) {
  if (!scene->getCamera()) {
    std::cerr << "Camera not initialized" << std::endl;
    return;
  }

  std::ofstream file(filename);
  if (!file) {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
    return;
  }

  Eigen::Vector3f position = scene->getCamera()->position();
  Eigen::Vector3f target = scene->getCamera()->target();
  float fovy = scene->getCamera()->fovY();

  file << "[Camera]\n";
  file << "position_x=" << position.x() << "\n";
  file << "position_y=" << position.y() << "\n";
  file << "position_z=" << position.z() << "\n";
  file << "target_x=" << target.x() << "\n";
  file << "target_y=" << target.y() << "\n";
  file << "target_z=" << target.z() << "\n";
  file << "fovy=" << fovy << "\n";

  file.close();
  std::cout << "Camera position saved to: " << filename << std::endl;
}

void Controller::importCameraPosition(const std::string &filename) {
  if (!scene->getCamera()) {
    std::cerr << "Camera not initialized" << std::endl;
    return;
  }

  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Failed to open file for reading: " << filename << std::endl;
    return;
  }

  scene->getCamera()->reset();

  Eigen::Vector3f position, target;
  float fovy;
  std::string line;
  std::string section;

  while (std::getline(file, line)) {
    // Trim whitespace
    line.erase(0, line.find_first_not_of(" \t"));
    line.erase(line.find_last_not_of(" \t") + 1);

    if (line.empty() || line[0] == ';')
      continue;

    if (line[0] == '[' && line[line.length() - 1] == ']') {
      section = line.substr(1, line.length() - 2);
      continue;
    }

    if (section == "Camera") {
      size_t delimPos = line.find('=');
      if (delimPos != std::string::npos) {
        std::string key = line.substr(0, delimPos);
        float value = std::stof(line.substr(delimPos + 1));

        if (key == "position_x")
          position.x() = value;
        else if (key == "position_y")
          position.y() = value;
        else if (key == "position_z")
          position.z() = value;
        else if (key == "target_x")
          target.x() = value;
        else if (key == "target_y")
          target.y() = value;
        else if (key == "target_z")
          target.z() = value;
        else if (key == "fovy")
          fovy = value;
      }
    }
  }

  scene->getCamera()->setPosition(position);
  scene->getCamera()->setTarget(target);
  scene->getCamera()->setFovY(fovy);

  file.close();
  std::cout << "Camera position loaded from: " << filename << std::endl;
}

int Controller::selectFaces(const double &x, const double &y) {
  // Convert screen coordinates to a point on the near plane
  Eigen::Vector3f pt = calculateNearPlanePoint(x, y, screenHeight, scene->getCamera());

  // Get mesh data structures
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  const std::vector<Triangle *> &triangles = MeshProcessor::getTriangles(mesh);
  const std::vector<Vertex *> &vertices = MeshProcessor::getVertices(mesh);

  int tidx = -1;
  // Setup ray origin and direction for intersection test
  Eigen::Vector3f cam_org = scene->getCamera()->position();
  Eigen::Vector3f cam_dir = (pt - cam_org).normalized();

  // Initialize target with maximum values
  Eigen::Vector3f target(FLT_MAX, FLT_MAX, FLT_MAX); // u,v,t

  // Test intersection with each triangle
  for (int i = 0; i < triangles.size(); i++) {
    const Triangle *tri = triangles[i];
    Eigen::Vector3f p0 = tri->verts[0]->pos.cast<float>();
    Eigen::Vector3f p1 = tri->verts[1]->pos.cast<float>();
    Eigen::Vector3f p2 = tri->verts[2]->pos.cast<float>();

    Eigen::Vector3f bary;
    // Check if ray intersects current triangle
    if (MeshProcessor::rayIntersectsTriangle(cam_org, cam_dir, p0, p1, p2, bary)) {
      std::cout << bary.transpose() << std::endl;
      // Update if this intersection is closer than previous ones
      if (bary.z() < target.z()) {
        target = bary;
        tidx = i;
      }
    }
  }

  // Toggle selection state of the intersected triangle if found
  if (tidx != -1) {
    mesh->tlist[tidx]->selected = !mesh->tlist[tidx]->selected;
    scene->getModel()->getMeshRenderer()->updateColors(mesh);
  }
  std::cout << "Select Triangle Index: " << tidx << '\n';
  return tidx;
}

int Controller::selectVertices(const double &x, const double &y) {
  Eigen::Vector3f pt = calculateNearPlanePoint(x, y, screenHeight, scene->getCamera());
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  const std::vector<Triangle *> &triangles = MeshProcessor::getTriangles(mesh);
  const std::vector<Vertex *> &vertices = MeshProcessor::getVertices(mesh);
  int tidx = -1;
  Eigen::Vector3f cam_org = scene->getCamera()->position();
  Eigen::Vector3f cam_dir = (pt - cam_org).normalized();
  Eigen::Vector3f target(FLT_MAX, FLT_MAX, FLT_MAX); // u,v,t
  for (int i = 0; i < triangles.size(); i++) {
    const Triangle *tri = triangles[i];
    Eigen::Vector3f p0 = tri->verts[0]->pos.cast<float>();
    Eigen::Vector3f p1 = tri->verts[1]->pos.cast<float>();
    Eigen::Vector3f p2 = tri->verts[2]->pos.cast<float>();
    Eigen::Vector3f bary;

    // TODO: Implement MeshProcessor::rayIntersectsTriangle in meshprocessor.cpp
    if (MeshProcessor::rayIntersectsTriangle(cam_org, cam_dir, p0, p1, p2, bary)) {
      if (bary.z() < target.z()) {
        target = bary;
        tidx = i;
      }
    }
  }

  // HINT: The process of selecting a face is explained in Controller::selectFaces
  // HINT: ID of the selected face is stored in tidx

  int vidx = -1;
  if (tidx != -1) {
    /*
    TODO: Implement the function to pick the vertex and store the index in vidx
    */

    if (vidx != -1) {
      mesh->vlist[vidx]->selected = !mesh->vlist[vidx]->selected;
    }
  }
  std::cout << "Select Vertex Index: " << vidx << '\n';
  return vidx;
}

int Controller::selectEdges(const double &x, const double &y) {
  Eigen::Vector3f pt = calculateNearPlanePoint(x, y, screenHeight, scene->getCamera());
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  const std::vector<Triangle *> &triangles = MeshProcessor::getTriangles(mesh);
  const std::vector<Vertex *> &vertices = MeshProcessor::getVertices(mesh);
  int tidx = -1;
  Eigen::Vector3f cam_org = scene->getCamera()->position();
  Eigen::Vector3f cam_dir = (pt - cam_org).normalized();
  Eigen::Vector3f target(FLT_MAX, FLT_MAX, FLT_MAX); // u,v,t
  for (int i = 0; i < triangles.size(); i++) {
    const Triangle *tri = triangles[i];
    Eigen::Vector3f p0 = tri->verts[0]->pos.cast<float>();
    Eigen::Vector3f p1 = tri->verts[1]->pos.cast<float>();
    Eigen::Vector3f p2 = tri->verts[2]->pos.cast<float>();
    Eigen::Vector3f bary;
    if (MeshProcessor::rayIntersectsTriangle(cam_org, cam_dir, p0, p1, p2, bary)) {
      if (bary.z() < target.z()) {
        target = bary;
        tidx = i;
      }
    }
  }

  // HINT: The process of selecting a face is explained in Controller::selectFaces
  // HINT: ID of the selected face is stored in tidx

  int eidx = -1;
  if (tidx != -1) {
    /*
    TODO: Implement the function to pick the edge and store the index in eidx
    */

    if (eidx != -1) {
      mesh->elist[eidx]->selected = !mesh->elist[eidx]->selected;
    }
  }
  std::cout << "Select Edge Index: " << eidx << '\n';
  return eidx;
}

/*
  HINT:
  You can get the list of vertices, edges, or faces from the Polyhedron class
  You can get the selected status of a vertex, edge, or face from the corresponding class's
  "selected" member variable It is HIGHLY recommended to check out the Polyhedron class in
  polyhedron.h
*/

void Controller::findClosures() {
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  /*
  TODO: Implement the function to find the closures
  */

  scene->getModel()->getMeshRenderer()->updateColors(mesh);
}

void Controller::findLinks() {
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  /*
  TODO: Implement the function to find the links
  */

  scene->getModel()->getMeshRenderer()->updateColors(mesh);
}

void Controller::findStars() {
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  /*
  TODO: Implement the function to find the stars
  */

  scene->getModel()->getMeshRenderer()->updateColors(mesh);
}

void Controller::findBoundaries() {
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  /*
  TODO: Implement the function to find the boundaries
  */

  scene->getModel()->getMeshRenderer()->updateColors(mesh);
}

void Controller::findInteriors() {
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  /*
  TODO: Implement the function to find the interiors
  */

  scene->getModel()->getMeshRenderer()->updateColors(mesh);
}
