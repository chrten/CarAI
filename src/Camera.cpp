
#include "Camera.h"

#include <glm/gtc/matrix_transform.hpp>

Camera::Camera(const glm::vec3& pos, const glm::vec3& dir, const glm::vec3& up,
  bool upVecConstraint)
  : m_pos(pos), m_dir(dir), m_up(up), m_upVecConstraint(upVecConstraint),
  m_dirty(true),
  m_view(1.0f), m_proj(1.0f)
{
  update();
}

Camera::~Camera()
{

}

void Camera::translate(const glm::vec3& v)
{
  m_pos += v;
  m_dirty = true;
}

void Camera::translateRel(const glm::vec3& v)
{
  glm::vec3 r = glm::cross(m_dir, m_up);
  glm::vec3 vabs = v.x * r + v.y * m_up + v.z * m_dir;
  translate(vabs);
}


void Camera::rotateRel(float x, float y)
{
  glm::vec3 r = glm::cross(m_dir, m_up);

  bool canRotateX = true;
  if (m_upVecConstraint)
  {
    // avoid parallel dir and up vecs
    glm::mat4 rotX = glm::rotate(glm::mat4(1.0f), x, r);

    glm::vec3 newDir = (glm::mat3)(rotX) * m_dir;
    float cosangle = glm::dot(glm::normalize(newDir), glm::normalize(m_up));

    if (std::fabs(cosangle) > 0.995f)
      canRotateX = false;
  }

  glm::mat4 rot(1.0f);
  rot = glm::rotate(rot, y, m_up);
  if (canRotateX)
    rot = glm::rotate(rot, x, r);

  glm::mat3 rot3 = (glm::mat3)rot;

  m_dir = rot3 * m_dir;

  if (!m_upVecConstraint)
    m_up = rot3 * m_up;

  m_dirty = true;
}


void Camera::perspective(float fovy, float aspect, float near, float far)
{
  m_proj = glm::perspective(fovy, aspect, near, far);
}


void Camera::ortho(float left, float right, float bottom, float top, float near, float far)
{
  m_proj = glm::ortho(left, right, bottom, top, near, far);
}

void Camera::orthonormalize()
{
  if (!m_upVecConstraint)
  {
    m_dir = glm::normalize(m_dir);

    glm::vec3 r = glm::cross(m_dir, m_up);

    m_up = glm::cross(r, m_dir);
    m_up = glm::normalize(m_up);

    m_up = glm::normalize(m_up);

    m_dirty = true;
  }  
}

void Camera::update()
{
  orthonormalize();
  m_view = glm::lookAt(m_pos, m_pos + m_dir, m_up);
  m_dirty = false;
}
