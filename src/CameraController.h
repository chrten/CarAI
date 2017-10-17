#pragma once

#include "Camera.h"

#include "UserInputController.h"

#include <btBulletDynamicsCommon.h>


class CameraController
{
public:
  CameraController(Camera* cam) : m_cam(cam) {}
  virtual ~CameraController() {}

  // animate
  virtual void update(double dt) {}


  Camera* cam() const { return m_cam; }

protected:

  Camera* m_cam;
};


// WSAD: move, RIGHT MOUSE + MOUSE MOVE: rotate
class CameraControllerUser : public CameraController, public UserInputController
{
public:

  CameraControllerUser(Camera* cam);
  virtual ~CameraControllerUser();

  void update(double dt);


  // window input events
  void mouseButtonEvent(GLFWwindow* wnd, int button, int action, int mods);
  void cursorPosEvent(GLFWwindow* wnd, double x, double y);
  void keyEvent(GLFWwindow* wnd, int key, int scancode, int action, int mods);


private:

  // translate from WSAD key press to speed in world space
  float m_moveTranslation;

  // speed factor when shift is pressed
  float m_speedFactorMod;

  // translate from mouse movement to angular speed
  float m_rotateTranslation;


  // current camera velocity and angular velocity
  glm::vec3 m_camVelocity;
  glm::vec2 m_camAngularVel;

  // user interaction
  bool m_rotateCameraMode;
  glm::vec<2, double> m_rotateCameraCursorPos;
};


// follow cam for rigid bodies
class CameraControllerFollow : public CameraController
{
public:
  CameraControllerFollow(Camera* cam, btRigidBody* body);

  void update(double dt);


  btRigidBody* body() const { return m_body; }
  void body(btRigidBody* b) { m_body = b; }

private:

  btRigidBody* m_body;

};