
#include "CameraController.h"


#include <glm/gtc/matrix_transform.hpp>



CameraControllerUser::CameraControllerUser(Camera* cam)
  : CameraController(cam),
  m_moveTranslation(5.0f), m_speedFactorMod(10.0f), m_rotateTranslation(15.0f * 3.14159f / 180.0f),
  m_camVelocity(0.0f, 0.0f, 0.0f), m_camAngularVel(0.0f, 0.0f),
  m_rotateCameraMode(false), m_rotateCameraCursorPos(0.0f, 0.0f)
{

}

CameraControllerUser::~CameraControllerUser()
{

}

void CameraControllerUser::update(double dt)
{
  m_cam->translateRel(m_camVelocity * float(dt));

  if (m_rotateCameraMode)
  {
    m_cam->rotateRel(m_camAngularVel.x * float(dt), m_camAngularVel.y * float(dt));
    m_camAngularVel = glm::vec2(0.0f, 0.0f);
  }

}

void CameraControllerUser::mouseButtonEvent(GLFWwindow* wnd, int button, int action, int mods)
{
  if (button == GLFW_MOUSE_BUTTON_RIGHT)
  {
    if (action == GLFW_PRESS)
      glfwGetCursorPos(wnd, &m_rotateCameraCursorPos.x, &m_rotateCameraCursorPos.y);
    m_rotateCameraMode = action != GLFW_RELEASE;

    glfwSetInputMode(wnd, GLFW_CURSOR, m_rotateCameraMode ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
  }
}

void CameraControllerUser::cursorPosEvent(GLFWwindow* wnd, double x, double y)
{
  // relative movement
  glm::vec2 r(y - m_rotateCameraCursorPos.y, x - m_rotateCameraCursorPos.x);
  m_camAngularVel = -m_rotateTranslation * r;

  m_rotateCameraCursorPos.x = x;
  m_rotateCameraCursorPos.y = y;

  if (!m_rotateCameraMode)
    m_camAngularVel = glm::vec2(0.0f, 0.0f);
}

void CameraControllerUser::keyEvent(GLFWwindow* wnd, int key, int scancode, int action, int mods)
{
  float speed = m_moveTranslation;
  if (mods & GLFW_MOD_SHIFT)
    speed *= m_speedFactorMod;

  if (key == GLFW_KEY_W)
  {
    if (action != GLFW_RELEASE)
      m_camVelocity.z = speed;
    else
      m_camVelocity.z = 0.0f;
  }
  else if (key == GLFW_KEY_S)
  {
    if (action != GLFW_RELEASE)
      m_camVelocity.z = -speed;
    else
      m_camVelocity.z = 0.0f;
  }
  if (key == GLFW_KEY_A)
  {
    if (action != GLFW_RELEASE)
      m_camVelocity.x = -speed;
    else
      m_camVelocity.x = 0.0f;
  }
  else if (key == GLFW_KEY_D)
  {
    if (action != GLFW_RELEASE)
      m_camVelocity.x = speed;
    else
      m_camVelocity.x = 0.0f;
  }
}


CameraControllerFollow::CameraControllerFollow(Camera* cam, btRigidBody* body)
  : CameraController(cam), m_body(body)
{

}

void CameraControllerFollow::update(double dt)
{
  if (m_body)
  {
    const btVector3& pos = m_body->getWorldTransform().getOrigin();
    const btMatrix3x3& rot = m_body->getWorldTransform().getBasis();

    glm::vec3 target(pos[0], pos[1], pos[2]);
    glm::vec3 dir(rot[0][2], rot[1][2], rot[2][2]);

    glm::vec3 posCam = target - dir * 6.0f + m_cam->up() * 3.0f;

    m_cam->pos(posCam);
    m_cam->dir(dir);
  }
}
