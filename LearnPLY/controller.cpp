#include "controller.h"
#include "learnply.h"
#include <stdio.h>
#include <cstddef>
#include <ImGUI/imgui.h>
#include <ImGUI/imgui_impl_glfw.h>
#include <ImGUI/imgui_impl_opengl3.h>
#include <chrono>

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
	  selectFacesImproved(xpos, ypos);
      //selectFaces(xpos, ypos);
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
  MeshRenderer* meshRenderer = scene->getModel()->getMeshRenderer();
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
      meshRenderer->setNormalMode(scene->getModel()->getPolyhedron(), scene->drawFaceNormal);
    }
    ImGui::Checkbox("Wireframe", &scene->drawWireframe);
    ImGui::Checkbox("Vertex Labels", &scene->drawVertexLabel);
    ImGui::Separator();
  }

  // Selection controls
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

  // Processing controls
  if (ImGui::CollapsingHeader("Process")) {
      if (ImGui::Button("Fix Orientation")) {
          bool flag = MeshProcessor::fixOrientation(scene->getModel()->getPolyhedron());
          if (flag){
              meshRenderer->setNormalMode(scene->getModel()->getPolyhedron(), scene->drawFaceNormal);
          }
      }
          
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
  // Get the starting time point
  auto start = std::chrono::high_resolution_clock::now();
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
  // Get the ending time point
  auto end = std::chrono::high_resolution_clock::now();

  // Toggle selection state of the intersected triangle if found
  if (tidx != -1) {
    mesh->tlist[tidx]->selected = !mesh->tlist[tidx]->selected;
    scene->getModel()->getMeshRenderer()->updateColors(mesh);
  }
  std::cout << "Select Triangle Index: " << tidx;
  // Calculate the elapsed time in milliseconds
  std::chrono::duration<double, std::milli> elapsed = end - start;
  std::cout << " (Elapsed time: " << elapsed.count() << " ms)" << std::endl;
  return tidx;
}

int Controller::selectFacesImproved(double x, double y)
{
    // Convert screen coordinates to a point on the near plane
    Eigen::Vector3f pt = calculateNearPlanePoint(x, y, screenHeight, scene->getCamera());

    // Get mesh data structures
    Polyhedron* mesh = scene->getModel()->getPolyhedron();
    const std::vector<Triangle*>& triangles = MeshProcessor::getTriangles(mesh);
    const std::vector<Vertex*>& vertices = MeshProcessor::getVertices(mesh);

    int tidx = -1;
    // Setup ray origin and direction for intersection test
    Eigen::Vector3f cam_org = scene->getCamera()->position();
    Eigen::Vector3f cam_dir = (pt - cam_org).normalized();

	// Construct a uniform Octree within the bounding box of the 
	// Every voxel hold a list of triangles that intersect with it
	// Voxel size is 1/4 the largest edge length of the bounding box
	auto initStart = std::chrono::high_resolution_clock::now();
	// Get the bounding box of the mesh
	Eigen::Vector3f min = mesh->vlist[0]->pos.cast<float>();
	Eigen::Vector3f max = mesh->vlist[0]->pos.cast<float>();
	for (int i = 1; i < mesh->vlist.size(); i++) {
		min = min.cwiseMin(mesh->vlist[i]->pos.cast<float>());
		max = max.cwiseMax(mesh->vlist[i]->pos.cast<float>());
	}

	// Calculate the largest edge length of the bounding box
	float max_edge = (max - min).maxCoeff();
	float voxel_size = max_edge / 4;

	// Caculate the voxel dimension in integer
	Eigen::Vector3i dim = ((max - min) / voxel_size).cast<int>();
    dim.x() += 1;
	dim.y() += 1;
	dim.z() += 1;

	// Allocate memory for the octree
    std::vector<std::vector<std::vector<std::vector<int>>>> octree;
	octree.resize(dim.x());
	for (int i = 0; i < dim.x(); i++) {
		octree[i].resize(dim.y());
		for (int j = 0; j < dim.y(); j++) {
			octree[i][j].resize(dim.z());
		}
	}

	// Insert triangles into the octree
	for (int i = 0; i < triangles.size(); i++) {
		const Triangle* tri = triangles[i];
		Eigen::Vector3f p0 = tri->verts[0]->pos.cast<float>();
		Eigen::Vector3f p1 = tri->verts[1]->pos.cast<float>();
		Eigen::Vector3f p2 = tri->verts[2]->pos.cast<float>();

		// Calculate the bounding box of the triangle
		Eigen::Vector3f tri_min = p0.cwiseMin(p1).cwiseMin(p2);
		Eigen::Vector3f tri_max = p0.cwiseMax(p1).cwiseMax(p2);

		// Calculate the voxel indices of the bounding box
		Eigen::Vector3f min_idx = (tri_min - min) / voxel_size;
		Eigen::Vector3f max_idx = (tri_max - min) / voxel_size;

		// Insert the triangle into the voxels
		for (int x = min_idx.x(); x <= max_idx.x(); x++) {
			for (int y = min_idx.y(); y <= max_idx.y(); y++) {
				for (int z = min_idx.z(); z <= max_idx.z(); z++) {
					octree[x][y][z].push_back(i);
				}
			}
		}
	}
	auto initEnd = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> initElapsed = initEnd - initStart;
	std::cout << "Octree Initialization Time: " << initElapsed.count() << " ms" << std::endl;
	// Initialize target with maximum values
	Eigen::Vector3f target(FLT_MAX, FLT_MAX, FLT_MAX); // u,v,t
	// Get the starting time point
	auto start = std::chrono::high_resolution_clock::now();
	// Test intersection with the voxels
	for (int x = 0; x < dim.x(); x++) {
		for (int y = 0; y < dim.y(); y++) {
			for (int z = 0; z < dim.z(); z++) {
				// Continue if the ray does not intersect with the voxel
				Eigen::Vector3f voxel_min = min + Eigen::Vector3f(x, y, z) * voxel_size;
				Eigen::Vector3f voxel_max = voxel_min + Eigen::Vector3f(voxel_size, voxel_size, voxel_size);

				// Check if the ray intersects with the voxel
				// Do math calculations to check if the ray intersects with the voxel
				float tmin = (voxel_min.x() - cam_org.x()) / cam_dir.x();
				float tmax = (voxel_max.x() - cam_org.x()) / cam_dir.x();

				if (tmin > tmax) {
					std::swap(tmin, tmax);
				}

				float tymin = (voxel_min.y() - cam_org.y()) / cam_dir.y();
				float tymax = (voxel_max.y() - cam_org.y()) / cam_dir.y();

				if (tymin > tymax) {
					std::swap(tymin, tymax);
				}

				if ((tmin > tymax) || (tymin > tmax)) {
					continue;
				}

				for (int i = 0; i < octree[x][y][z].size(); i++) {
					const Triangle* tri = triangles[octree[x][y][z][i]];
					Eigen::Vector3f p0 = tri->verts[0]->pos.cast<float>();
					Eigen::Vector3f p1 = tri->verts[1]->pos.cast<float>();
					Eigen::Vector3f p2 = tri->verts[2]->pos.cast<float>();

					Eigen::Vector3f bary;
					// Check if ray intersects current triangle
					if (MeshProcessor::rayIntersectsTriangle(cam_org, cam_dir, p0, p1, p2, bary)) {
						// Update if this intersection is closer than previous ones
						if (bary.z() < target.z()) {
							target = bary;
							tidx = octree[x][y][z][i];
						}
					}
				}
			}
		}
	}
     
    // Get the ending time point
    auto end = std::chrono::high_resolution_clock::now();

    // Toggle selection state of the intersected triangle if found
    if (tidx != -1) {
        mesh->tlist[tidx]->selected = !mesh->tlist[tidx]->selected;
        scene->getModel()->getMeshRenderer()->updateColors(mesh);
    }
    std::cout << "Select Triangle Index: " << tidx;
    // Calculate the elapsed time in milliseconds
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << " (Elapsed time: " << elapsed.count() << " ms)" << std::endl;
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

	  float u = target.x();
	  float v = target.y();
	  // Distance between uv and (0,0), (1,0), (0,1)
	  float d0 = (u - 0) * (u - 0) + (v - 0) * (v - 0);
	  float d1 = (u - 1) * (u - 1) + (v - 0) * (v - 0);
	  float d2 = (u - 0) * (u - 0) + (v - 1) * (v - 1);

	  // Find the closest vertex
	  if (d0 < d1 && d0 < d2) {
		  vidx = triangles[tidx]->verts[0]->index;
	  }
	  else if (d1 < d0 && d1 < d2) {
		  vidx = triangles[tidx]->verts[1]->index;
	  }
	  else {
		  vidx = triangles[tidx]->verts[2]->index;
	  }
	

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

	  float u = target.x();
	  float v = target.y();
      float u0 = 0;
	  float v0 = 0;
	  float u1 = 1;
	  float v1 = 0;
	  float u2 = 0;
	  float v2 = 1;


	  // Distance between uv and the three edges
      float d01 = fabs((v1 - v0) * u - (u1 - u0) * v + u1 * v0 - v1 * u0);
	  float d12 = fabs((v2 - v1) * u - (u2 - u1) * v + u2 * v1 - v2 * u1);
	  float d20 = fabs((v0 - v2) * u - (u0 - u2) * v + u0 * v2 - v0 * u2);

	  // Find the closest edge
	  if (d01 < d12 && d01 < d20) {
		  eidx = triangles[tidx]->edges[0]->index;
	  }
	  else if (d12 < d01 && d12 < d20) {
		  eidx = triangles[tidx]->edges[1]->index;
	  }
	  else {
		  eidx = triangles[tidx]->edges[2]->index;
	  }

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
  std::vector<Edge*> selectedEdges;
  std::vector<Vertex*> selectedVertices;

  for (const auto& edge : mesh->elist) {
	  if (edge->selected) {
		  // Add all the vertices that contain the edge
		  for (const auto& vertex : edge->verts) {
			  selectedVertices.push_back(vertex);
		  }
	  }
  }

  for (const auto& tri : mesh->tlist) {
	  if (tri->selected) {
		  // Add all the vertices that contain the triangle
		  for (const auto& vertex : tri->verts) {
			  selectedVertices.push_back(vertex);
		  }
		  // Add all the edges that contain the triangle
		  for (const auto& edge : tri->edges) {
			  selectedEdges.push_back(edge);
		  }
	  }
  }

  for (auto& vertex : selectedVertices) {
	  vertex->selected = true;
  }

  for (auto& edge : selectedEdges) {
	  edge->selected = true;
  }

  scene->getModel()->getMeshRenderer()->updateColors(mesh);
}

void Controller::findLinks() {
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  /*
  TODO: Implement the function to find the links
  */
  // Compute Closure(Star(s)) \ Star(Closure(s))
  // 1. Compute Closure(Star(s))
  std::vector<Edge*> linkEdges;
  std::vector<Vertex*> linkVertices;
  std::vector<Triangle*> linkTriangles;
  std::vector<Edge*> selectedEdges;
  std::vector<Vertex*> selectedVertices;
  std::vector<Triangle*> selectedTriangles;

  for (const auto& vertex : mesh->vlist) {
	  if (vertex->selected) {
		  selectedVertices.push_back(vertex);
		  for (const auto& tri : vertex->tris) {
			  linkTriangles.push_back(tri);
		  }
	  }
  }

  for (const auto& edge : mesh->elist) {
	  if (edge->selected) {
		  selectedEdges.push_back(edge);
		  edge->verts[0]->selected = true;
		  edge->verts[1]->selected = true;
		  selectedVertices.push_back(edge->verts[0]);
		  selectedVertices.push_back(edge->verts[1]);
		  // Add all the vertices that contain the edge
		  for (const auto& tri : edge->tris) {
			  linkTriangles.push_back(tri);
		  }
	  }
  }

  for (const auto& tri : linkTriangles) {
	  tri->selected = false;
	  for (const auto& vertex : tri->verts) {
		  if (!vertex->selected) {
			  linkVertices.push_back(vertex);
		  }
	  }
	  for (const auto& edge : tri->edges) {
		  if (!edge->selected && !edge->verts[0]->selected && !edge->verts[1]->selected) {
			  linkEdges.push_back(edge);
		  }
	  }
  }	

  for (auto& vertex : linkVertices) {
	  vertex->selected = true;
  }

  for (auto& edge : linkEdges) {
	  edge->selected = true;
  }
	
  for (auto& vertex : selectedVertices) {
	  vertex->selected = false;
  }

  for (auto& edge : selectedEdges) {
	  edge->verts[0]->selected = false;
	  edge->verts[1]->selected = false;
	  edge->selected = false;
  }

  for (auto& tri : mesh->tlist) {
	  tri->selected = false;
  }
  scene->getModel()->getMeshRenderer()->updateColors(mesh);
}

void Controller::findStars() {
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  /*
  TODO: Implement the function to find the stars
  */
  std::vector<Triangle*> selectedTriangles;
  std::vector<Edge*> selectedEdges;

  for (const auto& edge : mesh->elist) {
	  if (edge->selected) {
		  // Add all the triangles that contain the edge
		  for (const auto& tri : edge->tris) {
		    selectedTriangles.push_back(tri);
		  }
	  }
  }

  for (const auto& vertex : mesh->vlist) {
	  if (vertex->selected) {
		  // Add all the triangles that contain the vertex
		  for (const auto& tri : vertex->tris) {
			  selectedTriangles.push_back(tri);
		  }
		  for (const auto& corner : vertex->corners) {
			  selectedEdges.push_back(corner->next->edge);
		  }
	  }
  }

  for (auto& tri : selectedTriangles) {
	  tri->selected = true;
  }
  for (auto& edge : selectedEdges) {
	  edge->selected = true;
  }

  scene->getModel()->getMeshRenderer()->updateColors(mesh);
}

void Controller::findBoundaries() {
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  /*
  TODO: Implement the function to find the boundaries
  */

  // For all selected triangles, find the edges that are not shared by any other triangles
  std::vector<Edge*> selectedEdges;

  for (const auto& tri : mesh->tlist) {
	  if (tri->selected) {
		  for (const auto& edge : tri->edges) {
			  bool shared = false;
			  for (const auto& tri2 : edge->tris) {
				  if (tri2 != tri && tri2->selected == true) {
					  shared = true;
					  break;
				  }
			  }
			  if (!shared) {
				  selectedEdges.push_back(edge);
			  }
		  }
	  }
  }

  // Clear all selections
  clearSelections();

  // Select the boundary edges and vertices
  for (auto& edge : selectedEdges) {
	  edge->selected = true;
	  edge->verts[0]->selected = true;
	  edge->verts[1]->selected = true;
  }

  scene->getModel()->getMeshRenderer()->updateColors(mesh);
}

void Controller::findInteriors() {
  Polyhedron *mesh = scene->getModel()->getPolyhedron();
  /*
  TODO: Implement the function to find the interiors
  */
  // For all selected triangles, find the edges that are not shared by any other triangles
  std::vector<Edge*> selectedEdges;

  for (const auto& tri : mesh->tlist) {
	  if (tri->selected) {
		  for (const auto& edge : tri->edges) {
			  bool shared = false;
			  for (const auto& tri2 : edge->tris) {
				  if (tri2 != tri && tri2->selected == true) {
					  shared = true;
					  break;
				  }
			  }
			  if (!shared) {
				  selectedEdges.push_back(edge);
			  }
		  }
	  }
  }

  // Select the boundary edges and vertices
  for (auto& edge : selectedEdges) {
	  edge->selected = false;
	  edge->verts[0]->selected = false;
	  edge->verts[1]->selected = false;
  }

  scene->getModel()->getMeshRenderer()->updateColors(mesh);
}
