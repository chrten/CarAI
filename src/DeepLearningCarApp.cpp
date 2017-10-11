#include "DeepLearningCarApp.h"


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>


#include <lodepng.h>

#include <iostream>
#include <vector>




DeepLearningCarApp::DeepLearningCarApp(int w, int h, int glMajor, int glMinor, double physicsTimeStep)
  : Application(w, h, glMajor, glMinor, physicsTimeStep),
  m_bar(0), 
  m_cam(0), m_camControl(0),
  m_bullet(0), m_groundBody(0), m_sphereBody(0), m_vehicle(0), m_vehicleController(0),
  m_trackBody(0),
  m_singleColorProg(0), m_gridVBO(0), m_coordsysVBO(0),
  m_sphereMesh(0), m_boxMesh(0), m_trackMesh(0)
{
  m_cam = new Camera();

  // user controlled camera
  CameraControllerUser* camUserControl = new CameraControllerUser(m_cam);
  m_camControl = camUserControl;
  addUserInputController(camUserControl);

  // pass window events to tweakbar
  static AntTweakbarInputController tweakbarController;
  addUserInputController(&tweakbarController);
}


DeepLearningCarApp::~DeepLearningCarApp()
{
  delete m_sphereMesh;
  delete m_boxMesh;
  delete m_trackMesh;

  if (m_bullet)
  {
    if (m_vehicle)
    {
      m_bullet->world->removeRigidBody(m_vehicle->getRigidBody());
      delete m_vehicle->getRigidBody();
      m_bullet->world->removeVehicle(m_vehicle);
      delete m_vehicle;
    }

    delete m_bullet;
  }
  delete m_vehicleController;

  delete m_coordsysVBO;
  delete m_gridVBO;
  delete m_singleColorProg;

  delete m_camControl;
  delete m_cam;

  TwTerminate();
}

void DeepLearningCarApp::init()
{
  // init AntTweakBar
  TwInit(TW_OPENGL, NULL);
  int w, h;
  glfwGetWindowSize(window(), &w, &h);
  TwWindowSize(w, h);

  //m_bar = TwNewBar("TweakBar");

  // ----------------------------------------------------

  m_cam->perspective(90.0f * 3.14159f / 180.0f, static_cast<float>(w) / h, 0.0001f, 100.0f);

  // ----------------------------------------------------

  m_bullet = new BulletInterface();
  m_bullet->world->setGravity(btVector3(0, -10, 0));

  //m_groundBody = m_bullet->createManagedRigidBody(std::make_shared<btStaticPlaneShape>(btVector3(0, 1, 0), 1), 0.0, btVector3(0, -1, 0), false);
  //m_groundBody = m_bullet->createManagedRigidBody(std::make_shared<btBoxShape>(btVector3(100, 1, 100)), 0.0, btVector3(0, -1, 0), false);
  m_sphereBody = m_bullet->createManagedRigidBody(std::make_shared<btSphereShape>(0.5f), 1.0, btVector3(-3, 2, -1), true);
  m_sphereBody = m_bullet->createManagedRigidBody(std::make_shared<btSphereShape>(0.5f), 1.0, btVector3(3, 2, 1), true);
  m_sphereBody = m_bullet->createManagedRigidBody(std::make_shared<btSphereShape>(0.5f), 1.0, btVector3(3, 2, -1), true);

  initVehicle();
  initTrack();

  m_vehicleController = new VehicleControllerUser(m_vehicle);
  addUserInputController(m_vehicleController);

  // use vehicle follow cam
  if (dynamic_cast<UserInputController*>(m_camControl))
    removeUserInputController(dynamic_cast<UserInputController*>(m_camControl));
  delete m_camControl;
  m_camControl = new CameraControllerFollow(m_cam, m_vehicle->getRigidBody());

  // -----------------------------------------------------

  // basic shader 

  m_singleColorProg = new GL::Program();
  m_singleColorProg->linkFromFile("../data/shaders/simple_vs.glsl", "../data/shaders/simple_fs.glsl");

  m_sphereMesh = new GL::Mesh("../data/obj/sphere.obj");
  m_boxMesh = new GL::Mesh("../data/obj/cube.obj");
}

void DeepLearningCarApp::initVehicle()
{
  if (!m_vehicleChassisShape.get())
  {
    m_vehicleChassisExtents = btVector3(1, 0.5, 2);
    m_vehicleChassisOffset = btVector3(0.0, 1.0, 0.0);

    //The btBoxShape is centered at the origin
    m_vehicleChassisShape = std::make_shared<btBoxShape>(m_vehicleChassisExtents);

    //A compound shape is used so we can easily shift the center of gravity of our vehicle to its bottom
    //This is needed to make our vehicle more stable
    m_vehicleChassisCompound = std::make_shared<btCompoundShape>();

    btTransform localTransform;
    localTransform.setIdentity();
    localTransform.setOrigin(m_vehicleChassisOffset);

    //The center of gravity of the compound shape is the origin. When we add a rigidbody to the compound shape
    //it's center of gravity does not change. This way we can add the chassis rigidbody one unit above our center of gravity
    //keeping it under our chassis, and not in the middle of it
    m_vehicleChassisCompound->addChildShape(localTransform, m_vehicleChassisShape.get());
  }

  m_vehicle = m_bullet->createUnmanagedVehicle(m_vehicleChassisCompound, 1200, btVector3(0.0, 1.0, 0.0));


}


void DeepLearningCarApp::initTrack()
{
  // load from heightfield
  unsigned int w, h;
  if (!lodepng::decode(m_trackHeights, w, h, "../data/tracks/track0.png", LCT_GREY, 8u))
  {
    std::shared_ptr<btHeightfieldTerrainShape> trackShape = std::make_shared<btHeightfieldTerrainShape>(w, h, &m_trackHeights[0], 10.0f / 256.0f, 0.0f, 10.0f, 1, PHY_UCHAR, false);
    m_trackBody = m_bullet->createManagedRigidBody(trackShape, 0.0f, btVector3(0.0f, 0.0f, 0.0f), false);
  }
  else
    std::cout << "failed to load track" << std::endl;
}

void DeepLearningCarApp::draw(double time)
{
  int width, height;
  framebufferSize(&width, &height);
  glViewport(0, 0, width, height);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);


  m_cam->perspective(90.0f * 3.14159f / 180.0f, static_cast<float>(width) / height, 0.0001f, 100.0f);


  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  glm::mat4 viewProj = m_cam->proj() * m_cam->view();


  if (0&&m_sphereBody && m_sphereMesh)
  {
    btVector3 spherePos = m_sphereBody->getCenterOfMassPosition();
    glm::mat4 wvp = viewProj;
    wvp = glm::translate(wvp, glm::vec3(spherePos[0], spherePos[1], spherePos[2]));

    m_singleColorProg->use();
    m_singleColorProg->setUniform4f("color", glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    m_singleColorProg->setUniformMatrix4f("WVP", wvp);

    m_sphereMesh->draw();
  }

  if (0 && m_vehicle)
  {
    glm::mat4 wvp;
    m_vehicle->getChassisWorldTransform().getOpenGLMatrix((float*)&wvp);
    wvp = viewProj * wvp;
    wvp = glm::translate(wvp, glm::vec3(m_vehicleChassisOffset[0], m_vehicleChassisOffset[1], m_vehicleChassisOffset[2]));
    wvp = glm::scale(wvp, glm::vec3(m_vehicleChassisExtents[0], m_vehicleChassisExtents[1], m_vehicleChassisExtents[2]) * 2.0f);

    m_singleColorProg->use();
    m_singleColorProg->setUniform4f("color", glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
    m_singleColorProg->setUniformMatrix4f("WVP", wvp);

    m_boxMesh->draw();
  }

  if (m_trackMesh)
  {
    m_singleColorProg->use();
    m_singleColorProg->setUniform4f("color", glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
    m_singleColorProg->setUniformMatrix4f("WVP", viewProj);

    m_trackMesh->draw();

    // disable visualization in debug drawer to prevent crash
    int f = m_trackBody->getCollisionFlags();
    m_trackBody->setCollisionFlags(f | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
  }

  m_singleColorProg->disable();
  
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


  {
    static GLDebugDrawer bulletDebugDrawer;

    bulletDebugDrawer.setDebugMode(btIDebugDraw::DBG_DrawWireframe);
    bulletDebugDrawer.setViewProj(viewProj);

    m_bullet->world->setDebugDrawer(&bulletDebugDrawer);

    bulletDebugDrawer.beginDraw();
    m_bullet->world->debugDrawWorld();
    bulletDebugDrawer.endDraw();
  }
  



  drawGrid();

  // draw tweakbar
  TwDraw();
}

void DeepLearningCarApp::updatePhysics(double dt)
{
  // update bullet world
  m_bullet->world->stepSimulation(static_cast<btScalar>(dt), 10);

  // animate camera
  m_camControl->update(dt);
}



// void DeepLearningCarApp::resizeEvent(int w, int h)
// {
//   m_cam->perspective(90.0f * 3.14159f / 180.0f, static_cast<float>(w) / h, 0.0001f, 100.0f);
// }

// 
// void DeepLearningCarApp::keyEvent(int key, int scancode, int action, int mods)
// {
//   if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
//     glfwSetWindowShouldClose(window(), GLFW_TRUE);
// 
//   m_camControl->keyEvent(key, scancode, action, mods);
// 
//   // ---------------------------------------------
//   // car handling
//   
// 
// 
//   
// }

void DeepLearningCarApp::drawCoordSys()
{

}

void DeepLearningCarApp::drawGrid()
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


  glm::mat4x4 wvp = m_cam->proj() * m_cam->view();


  m_singleColorProg->use();
  m_singleColorProg->setUniform4f("color", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
  m_singleColorProg->setUniformMatrix4f("WVP", wvp);


  m_gridVBO->bind();
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12, 0);

  glDrawArrays(GL_LINES, 0, n * 2 * 2);
}
