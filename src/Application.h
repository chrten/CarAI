#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "UserInputController.h"

#include <vector>

class Application
{
public:
  Application(int w = 640, int h = 480, int glMajor = 2, int glMinor = 0, double physicsTimeStep = 1.0 / 60.0);
  virtual ~Application();


  // execute main loop
  virtual void exec();


  virtual void init() {}

  virtual void draw(double time) {}
  virtual void updatePhysics(double dt) {}

  // handle window input events via controller
  static void addUserInputController(UserInputController* controller);
  static void removeUserInputController(UserInputController* controller);

  GLFWwindow* window() const { return m_wnd; }

  int framebufferWidth() const;
  int framebufferHeight() const;

  void framebufferSize(int* w, int* h) const;

private:

  // glfw error callback
  static void errorCallback(int error, const char* description);

  // glfw input callbacks
  static void windowSizeCallback(GLFWwindow* window, int w, int h);
  static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
  static void cursorPosCallback(GLFWwindow* window, double x, double y);
  static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
  static void scrollCallback(GLFWwindow* window, double x, double y);

  static std::vector<UserInputController*> s_inputController;

private:

  GLFWwindow* m_wnd;

  double m_time;
  double m_physicsTimeStep;
};

