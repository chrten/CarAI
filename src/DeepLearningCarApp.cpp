#include "DeepLearningCarApp.h"


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


#include <iostream>
#include <vector>


class AppKeyControl : public UserInputController
{
public:
  AppKeyControl(DeepLearningCarApp* app) : m_app(app) {}

  virtual void keyEvent(GLFWwindow* wnd, int key, int scancode, int action, int mods)
  {
    if (key == GLFW_KEY_C && action == GLFW_RELEASE)
    {
      if (dynamic_cast<CameraControllerUser*>(m_app->camController()))
        m_app->camController(new CameraControllerFollow(m_app->cam(), m_app->simulation()->vehicle(0)->physics()->getRigidBody()));
      else
      {
        CameraControllerUser* camControl = new CameraControllerUser(m_app->cam());
        m_app->addUserInputController(camControl);
        m_app->camController(camControl);
      }
    }
  }

private:
  DeepLearningCarApp* m_app;
};


DeepLearningCarApp::DeepLearningCarApp(int w, int h, int glMajor, int glMinor, double physicsTimeStep)
  : Application(w, h, glMajor, glMinor, physicsTimeStep),
  m_cam(0), m_camControl(0), m_simulation(0), m_renderer(0), m_settings(0)
{
  m_cam = new Camera();

  // user controlled camera
  CameraControllerUser* camUserControl = new CameraControllerUser(m_cam);
  m_camControl = camUserControl;
  addUserInputController(camUserControl);

  // pass window events to tweakbar
  static AntTweakbarInputController tweakbarController;
  addUserInputController(&tweakbarController);


  // pass window events to app controller
  static AppKeyControl appController(this);
  addUserInputController(&appController);
}


DeepLearningCarApp::~DeepLearningCarApp()
{
  delete m_camControl;
  delete m_cam;
  delete m_simulation;
  delete m_renderer;
  delete m_settings;

  TwTerminate();
}

void DeepLearningCarApp::init()
{
  // init AntTweakBar
  TwInit(TW_OPENGL, NULL);
  int w, h;
  glfwGetWindowSize(window(), &w, &h);
  TwWindowSize(w, h);


  m_settings = new INIReader("../data/settings.ini");

  m_simulation = new Simulation(m_settings, this);
  
  // ----------------------------------------------------

  m_cam->perspective(90.0f * 3.14159f / 180.0f, static_cast<float>(w) / h, 0.1f, 10000.0f);

  // ----------------------------------------------------

  if (m_settings->Get("simulation", "camera", "userCam") == "followCam")
  {
    // use vehicle follow cam
    camController(new CameraControllerFollow(m_cam, m_simulation->vehicle(0)->physics()->getRigidBody()));
  }

  // -----------------------------------------------------

  m_renderer = new Renderer();


  m_simulation->initTweakVars(m_renderer->tweakbar());
}

void DeepLearningCarApp::draw(double time)
{
  int width, height;
  framebufferSize(&width, &height);

  m_renderer->framebufferSize(width, height);

  m_renderer->draw(time, m_simulation, m_cam);
}

void DeepLearningCarApp::updatePhysics(double dt)
{
  if (m_simulation)
    m_simulation->update(dt);

  // follow best vehicle
  CameraControllerFollow* followCam = dynamic_cast<CameraControllerFollow*>(m_camControl);
  if (followCam)
  {
    Vehicle* best = m_simulation->bestVehicle();

    if (m_simulation->userVehicle())
      best = m_simulation->userVehicle();

    if (best)
      followCam->body(best->physics()->getRigidBody());
  }

  // animate camera
  if (m_camControl)
    m_camControl->update(dt);
}

void DeepLearningCarApp::camController(CameraController* camControl)
{
  if (dynamic_cast<UserInputController*>(m_camControl))
    removeUserInputController(dynamic_cast<UserInputController*>(m_camControl));
  delete m_camControl;
  m_camControl = camControl;
}


