
#include "Renderer.h"

#include <glm/gtc/matrix_transform.hpp>


Renderer::Renderer()
  : m_singleColorProg(0), m_gridVBO(0), m_coordsysVBO(0),
  m_sphereMesh(0), m_boxMesh(0), m_trackMesh(0)
{
  // basic shader 

  m_singleColorProg = new GL::Program();
  m_singleColorProg->linkFromFile("../data/shaders/simple_vs.glsl", "../data/shaders/simple_fs.glsl");

  m_sphereMesh = new GL::Mesh("../data/obj/sphere.obj");
  m_boxMesh = new GL::Mesh("../data/obj/cube.obj");
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


//   if (0 && m_sphereBody && m_sphereMesh)
//   {
//     btVector3 spherePos = m_sphereBody->getCenterOfMassPosition();
//     glm::mat4 wvp = viewProj;
//     wvp = glm::translate(wvp, glm::vec3(spherePos[0], spherePos[1], spherePos[2]));
// 
//     m_singleColorProg->use();
//     m_singleColorProg->setUniform4f("color", glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
//     m_singleColorProg->setUniformMatrix4f("WVP", wvp);
// 
//     m_sphereMesh->draw();
//   }
// 
//   if (0 && m_vehicle)
//   {
//     glm::mat4 wvp;
//     m_vehicle->getChassisWorldTransform().getOpenGLMatrix((float*)&wvp);
//     wvp = viewProj * wvp;
//     wvp = glm::translate(wvp, glm::vec3(m_vehicleChassisOffset[0], m_vehicleChassisOffset[1], m_vehicleChassisOffset[2]));
//     wvp = glm::scale(wvp, glm::vec3(m_vehicleChassisExtents[0], m_vehicleChassisExtents[1], m_vehicleChassisExtents[2]) * 2.0f);
// 
//     m_singleColorProg->use();
//     m_singleColorProg->setUniform4f("color", glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
//     m_singleColorProg->setUniformMatrix4f("WVP", wvp);
// 
//     m_boxMesh->draw();
//   }

  if (m_trackMesh)
  {
    m_singleColorProg->use();
    m_singleColorProg->setUniform4f("color", glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
    m_singleColorProg->setUniformMatrix4f("WVP", viewProj);

    m_trackMesh->draw();

    // disable visualization in debug drawer to prevent crash
    int f = sim->m_trackBody->getCollisionFlags();
    sim->m_trackBody->setCollisionFlags(f | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
  }

  m_singleColorProg->disable();

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


  {
    static GLDebugDrawer bulletDebugDrawer;

    bulletDebugDrawer.setDebugMode(btIDebugDraw::DBG_DrawWireframe);
    bulletDebugDrawer.setViewProj(viewProj);

    sim->m_bullet->world->setDebugDrawer(&bulletDebugDrawer);

    bulletDebugDrawer.beginDraw();
    sim->m_bullet->world->debugDrawWorld();
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
