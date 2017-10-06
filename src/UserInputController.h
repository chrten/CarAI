#pragma once

#include <GLFW/glfw3.h>


// base interface
class UserInputController
{
public:

  UserInputController() {}
  virtual ~UserInputController() {}

  // window input events
  virtual void resizeEvent(GLFWwindow* wnd, int w, int h) {}
  virtual void mouseButtonEvent(GLFWwindow* wnd, int button, int action, int mods) {}
  virtual void cursorPosEvent(GLFWwindow* wnd, double x, double y) {}
  virtual void keyEvent(GLFWwindow* wnd, int key, int scancode, int action, int mods) {}
  virtual void scrollEvent(GLFWwindow* wnd, double x, double y) {}
};




class AntTweakbarInputController : public UserInputController
{
public:
  AntTweakbarInputController() {}
  ~AntTweakbarInputController() {}
  virtual void resizeEvent(GLFWwindow* wnd, int w, int h);
  virtual void mouseButtonEvent(GLFWwindow* wnd, int button, int action, int mods);
  virtual void cursorPosEvent(GLFWwindow* wnd, double x, double y);
  virtual void keyEvent(GLFWwindow* wnd, int key, int scancode, int action, int mods);
  virtual void scrollEvent(GLFWwindow* wnd, double x, double y);
};