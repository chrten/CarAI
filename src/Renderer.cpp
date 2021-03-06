
#include "Renderer.h"

#include <glm/gtc/matrix_transform.hpp>


Renderer::Renderer()
  : m_width(0), m_height(0), m_bar(0),
  m_singleColorProg(0), m_gridVBO(0), m_coordsysVBO(0),
  m_sphereMesh(0), m_boxMesh(0), m_trackMesh(0)
{
  // basic shader 

  m_singleColorProg = new GL::Program();
  m_singleColorProg->linkFromFile("../data/shaders/simple_vs.glsl", "../data/shaders/simple_fs.glsl");

  m_sphereMesh = new GL::Mesh("../data/obj/sphere.obj");
  m_boxMesh = new GL::Mesh("../data/obj/cube.obj");


  m_bar = TwNewBar("TweakBar");
}

Renderer::~Renderer()
{
  delete m_boxMesh;
  delete m_sphereMesh;
  delete m_singleColorProg;

  delete m_coordsysVBO;
  delete m_gridVBO;
}

void Renderer::draw(double time, Simulation* sim, Camera* cam)
{
  glViewport(0, 0, m_width, m_height);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);


  cam->perspective(90.0f * 3.14159f / 180.0f, static_cast<float>(m_width) / m_height, 0.0001f, 100.0f);


  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  glm::mat4 viewProj = cam->proj() * cam->view();


  if (m_trackMesh)
  {
    m_singleColorProg->use();
    m_singleColorProg->setUniform4f("color", glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
    m_singleColorProg->setUniformMatrix4f("WVP", viewProj);

    m_trackMesh->draw();

    // disable visualization in debug drawer to prevent crash
    int f = sim->trackBody()->getCollisionFlags();
    sim->trackBody()->setCollisionFlags(f | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
  }

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);




  // draw sensor info
  {
    static std::vector<float> sensorLines;
    
    sensorLines.clear();
    for (int i = 0; i < sim->numVehicles(); ++i)
    {
      Vehicle* v = sim->vehicle(i);

      if (v)
      {
        int ns = v->numSensors();
        for (int i = 0; i < ns; ++i)
        {
          Vehicle::Sensor* s = v->sensor(i);

          for (int k = 0; k < 3; ++k)
            sensorLines.push_back(s->startWS[k]);

          btVector3 endWS = s->startWS + (s->endWS - s->startWS).normalized() * s->dist;
          for (int k = 0; k < 3; ++k)
            sensorLines.push_back(endWS[k]);
        }
      }
    }


    int numLineVerts = static_cast<int>(sensorLines.size() / 3);

    drawLines(numLineVerts, sensorLines.data(), 12, viewProj, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
  }
  
  if (!sim->trackSegments().empty())
  {
    int numLineVerts = static_cast<int>(sim->trackSegments().size());
    drawLines(numLineVerts, sim->trackSegments().data(), sizeof(btVector3), viewProj, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), true);
  }




  m_singleColorProg->disable();






  {
    static GLDebugDrawer bulletDebugDrawer;

    bulletDebugDrawer.setDebugMode(btIDebugDraw::DBG_DrawWireframe);
    bulletDebugDrawer.setViewProj(viewProj);

    sim->world()->setDebugDrawer(&bulletDebugDrawer);

    bulletDebugDrawer.beginDraw();
    sim->world()->debugDrawWorld();
    bulletDebugDrawer.endDraw();
  }




  //drawGrid();



  // draw tweakbars
  TwDraw();
}



void Renderer::drawCoordSys()
{

}

void Renderer::drawGrid(const glm::mat4& wvp)
{
  static float extents = 10.0f;
  static int n = 10;

  if (!m_gridVBO)
  {
    std::vector<glm::vec3> pos(n * 2 * 2);

    float xref = -extents * 0.5f;

    static float scale = 1.0f; // 2.0f / extents;

    for (int i = 0; i < n; ++i)
    {
      float x = float(i) / (n - 1) * extents + xref;

      // horizontal
      pos[i * 2 + 0] = glm::vec3(x, 0.0f, xref) * scale;
      pos[i * 2 + 1] = glm::vec3(x, 0.0f, xref + extents) * scale;

      // vertical
      pos[i * 2 + 0 + n * 2] = glm::vec3(xref, 0.0f, x) * scale;
      pos[i * 2 + 1 + n * 2] = glm::vec3(xref + extents, 0.0f, x) * scale;
    }

    m_gridVBO = new GL::VertexBuffer();
    m_gridVBO->setData(12 * pos.size(), &pos[0]);
  }


  m_singleColorProg->use();
  m_singleColorProg->setUniform4f("color", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
  m_singleColorProg->setUniformMatrix4f("WVP", wvp);


  m_gridVBO->bind();
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12, 0);

  glDrawArrays(GL_LINES, 0, n * 2 * 2);
}

void Renderer::drawLines(int numVertices, const void* vertices, int stride, const glm::mat4& wvp, const glm::vec4& color, bool strip)
{
  if (numVertices)
  {
    m_singleColorProg->use();
    m_singleColorProg->setUniform4f("color", color);
    m_singleColorProg->setUniformMatrix4f("WVP", wvp);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, vertices);

    glDrawArrays(strip ? GL_LINE_STRIP : GL_LINES, 0, numVertices);
    glDrawArrays(GL_POINTS, 0, numVertices);

    glDisableVertexAttribArray(0);
  }
}
