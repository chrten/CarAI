#pragma once


#include "GLObjects.h"
#include "Simulation/Simulation.h"


#include <glad/glad.h>

#include "Camera.h"

#include <AntTweakBar.h>


class Renderer
{
public:
  Renderer();
  virtual ~Renderer();


  void framebufferSize(int w, int h) { m_width = w; m_height = h; }

  TwBar* tweakbar() { return m_bar; }


  // animate
  virtual void update(double dt) {}


  virtual void draw(double time, Simulation* sim, Camera* cam);

private:

  void drawCoordSys();

  void drawGrid(const glm::mat4& wvp);

  void drawLines(int numVertices, const void* vertices, int stride, const glm::mat4& wvp, const glm::vec4& color, bool strip = false);

private:

  int m_width, m_height;

  TwBar* m_bar;


  GL::Program* m_singleColorProg;

  GL::VertexBuffer* m_gridVBO;
  GL::VertexBuffer* m_coordsysVBO;

  GL::Mesh* m_sphereMesh;
  GL::Mesh* m_boxMesh;
  GL::Mesh* m_trackMesh;
};