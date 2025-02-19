#include <iostream>
#include <memory>
#define GLFW_INCLUDE_NONE
#include "controller.h"
#include "model.h"
#include "renderer.h"
#include "scene.h"
#include <GLFW/glfw3.h>
#include <glad/glad.h>

namespace LearnPLY {
// Constants
constexpr int WINDOW_WIDTH = 1024;
constexpr int WINDOW_HEIGHT = 1024;

std::unique_ptr<Scene> scene;
std::unique_ptr<Controller> controller;
std::unique_ptr<Renderer> renderer;

// GLFW callbacks
void framebufferSizeCallback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
  scene->resize(width, height);
  controller->resize(width, height);
  renderer->resize(width, height);
}

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods) {
  if (action != GLFW_PRESS && action != GLFW_REPEAT)
    return;
  controller->handleKeyPress(key);
}

void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods) {
  double xpos, ypos;
  glfwGetCursorPos(window, &xpos, &ypos);
  controller->handleMouseClick(button, xpos, ypos, action);
}

void cursorPosCallback(GLFWwindow *window, double xpos, double ypos) {
  controller->handleMouseMove(xpos, ypos);
}

void scrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
  controller->handleMouseScroll(xoffset, yoffset);
}

void printGLInfo() {
  std::cout << "GL_VENDOR: " << glGetString(GL_VENDOR) << '\n'
            << "GL_RENDERER: " << glGetString(GL_RENDERER) << '\n'
            << "GL_VERSION: " << glGetString(GL_VERSION) << '\n'
            << "GL_SHADING_LANGUAGE_VERSION: " << glGetString(GL_SHADING_LANGUAGE_VERSION)
            << std::endl;
}
} // namespace LearnPLY

int main(int argc, char **argv) {
  // Initialize GLFW
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW" << std::endl;
    return -1;
  }

  // Configure GLFW
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  // Create window
  GLFWwindow *window = glfwCreateWindow(LearnPLY::WINDOW_WIDTH, LearnPLY::WINDOW_HEIGHT,
                                        "PLY Viewer", nullptr, nullptr);
  if (!window) {
    std::cerr << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(window);

  // Initialize GLAD
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  // Print OpenGL information
  LearnPLY::printGLInfo();

  // Set callbacks
  glfwSetFramebufferSizeCallback(window, LearnPLY::framebufferSizeCallback);
  glfwSetKeyCallback(window, LearnPLY::keyCallback);
  glfwSetMouseButtonCallback(window, LearnPLY::mouseButtonCallback);
  glfwSetCursorPosCallback(window, LearnPLY::cursorPosCallback);
  glfwSetScrollCallback(window, LearnPLY::scrollCallback);

  // Initialize components
  auto model = std::make_unique<Model>();
  LearnPLY::scene =
      std::make_unique<Scene>(std::move(model), LearnPLY::WINDOW_WIDTH, LearnPLY::WINDOW_HEIGHT);

  LearnPLY::scene->loadModel(0);

  LearnPLY::renderer = std::make_unique<Renderer>(LearnPLY::WINDOW_WIDTH, LearnPLY::WINDOW_HEIGHT,
                                                  LearnPLY::scene.get());
  LearnPLY::controller = std::make_unique<Controller>(
      LearnPLY::WINDOW_WIDTH, LearnPLY::WINDOW_HEIGHT, window, LearnPLY::scene.get());

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    LearnPLY::renderer->render();
    LearnPLY::controller->render();
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // Cleanup is handled by smart pointers
  glfwTerminate();
  return 0;
}
