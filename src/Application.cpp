#include "Application.h"

#include <iostream>


std::vector<UserInputController*> Application::s_inputController;


Application::Application(int w, int h, int glMajor, int glMinor, double physicsTimeStep)
  : m_wnd(0),m_time(0.0), m_physicsTimeStep(physicsTimeStep)
{
  // init glfw and create window
  glfwSetErrorCallback(errorCallback);
  if (!glfwInit())
    exit(EXIT_FAILURE);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, glMajor);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, glMinor);

  m_wnd = glfwCreateWindow(w, h, "Simulation", NULL, NULL);
  if (!m_wnd)
  {
    glfwTerminate();
    exit(EXIT_FAILURE);
  }


  // Set GLFW event callbacks
  glfwSetWindowSizeCallback(m_wnd, windowSizeCallback);
  glfwSetKeyCallback(m_wnd, keyCallback);
  glfwSetCursorPosCallback(m_wnd, cursorPosCallback);
  glfwSetMouseButtonCallback(m_wnd, mouseButtonCallback);
  glfwSetScrollCallback(m_wnd, scrollCallback);


  glfwMakeContextCurrent(m_wnd);
  glfwSwapInterval(1);


  // init gl function pointers with GLAD
  gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
}


Application::~Application()
{
  glfwDestroyWindow(m_wnd);
  glfwTerminate();
}


void Application::exec()
{
  m_time = glfwGetTime();

  double timeAccum = 0.0;
  double worldTime = 0.0;

  while (!glfwWindowShouldClose(m_wnd))
  {
    double newTime = glfwGetTime();
    double dt = newTime - m_time;

    // update physics with fixed time intervals
    timeAccum += dt;

    while (timeAccum >= m_physicsTimeStep)
    {
      updatePhysics(m_physicsTimeStep);
      timeAccum -= m_physicsTimeStep;

      worldTime += m_physicsTimeStep;
    }
    m_time = newTime;

    draw(worldTime);

    glfwSwapBuffers(m_wnd);
    glfwPollEvents();
  }
}



void Application::addUserInputController(UserInputController* controller)
{
  if (controller)
    s_inputController.push_back(controller);
}


void Application::removeUserInputController(UserInputController* controller)
{
  size_t n = s_inputController.size();
  for (size_t i = 0; i < n; ++i)
    if (s_inputController[i] == controller)
    {
      s_inputController.erase(s_inputController.begin() + i);
      break;
    }
}

int Application::framebufferWidth() const
{
  int w, h;
  framebufferSize(&w, &h);
  return w;
}


int Application::framebufferHeight() const
{
  int w, h;
  framebufferSize(&w, &h);
  return h;
}

void Application::framebufferSize(int* w, int* h) const
{
  if (m_wnd)
    glfwGetFramebufferSize(m_wnd, w, h);
  else
  {
    if (w) *w = 0;
    if (h) *h = 0;
  }
}

void Application::errorCallback(int error, const char* description)
{
  std::cerr << "Error: " << description << std::endl;
}

void Application::windowSizeCallback(GLFWwindow* window, int w, int h)
{
  size_t n = s_inputController.size();
  for (size_t i = 0; i < n; ++i)
    s_inputController[i]->resizeEvent(window, w, h);
}

void Application::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
  size_t n = s_inputController.size();
  for (size_t i = 0; i < n; ++i)
    s_inputController[i]->mouseButtonEvent(window, button, action, mods);
}

void Application::cursorPosCallback(GLFWwindow* window, double x, double y)
{
  size_t n = s_inputController.size();
  for (size_t i = 0; i < n; ++i)
    s_inputController[i]->cursorPosEvent(window, x, y);
}

void Application::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  size_t n = s_inputController.size();
  for (size_t i = 0; i < n; ++i)
    s_inputController[i]->keyEvent(window, key, scancode, action, mods);
}

void Application::scrollCallback(GLFWwindow* window, double x, double y)
{
  size_t n = s_inputController.size();
  for (size_t i = 0; i < n; ++i)
    s_inputController[i]->scrollEvent(window, x, y);
}
