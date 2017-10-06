#pragma once

#include <glm/glm.hpp>


class Camera
{
public:

  Camera(const glm::vec3& pos = glm::vec3(0.0f, 0.0f, 0.0f),
    const glm::vec3& dir = glm::vec3(0.0f, 0.0f, -1.0f),
    const glm::vec3& up = glm::vec3(0.0f, 1.0f, 0.0f),
    bool upVecConstraint = true);
  virtual ~Camera();


  // absolute translation in world space
  void translate(const glm::vec3& v);

  // translate relative to camera axes
  void translateRel(const glm::vec3& v);

  // rotate along right and up vector (angles in rad)
  void rotateRel(float x, float y);


  void pos(const glm::vec3& pos) { m_pos = pos; m_dirty = true; }
  void dir(const glm::vec3& dir) { m_dir = dir; m_dirty = true; }
  void up(const glm::vec3& up) { m_up = up; m_dirty = true; }

  const glm::vec3& pos() const { return m_pos; }
  const glm::vec3& dir() const { return m_dir; }
  const glm::vec3& up() const { return m_up; }

  const glm::mat4& view() { if (m_dirty) update(); return m_view; }



  void perspective(float fovy, float aspect, float near, float far);
  void ortho(float left, float right, float bottom, float top, float near, float far);

  void proj(const glm::mat4& p) { m_proj = p; }
  const glm::mat4& proj() const { return m_proj; }

private:

  void orthonormalize();
  void update();

  glm::vec3 m_pos;
  glm::vec3 m_dir;
  glm::vec3 m_up;
  bool m_upVecConstraint;
  bool m_dirty;

  glm::mat4 m_view;
  glm::mat4 m_proj;
};