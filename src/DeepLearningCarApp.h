#pragma once

#include "Application.h"

#include "CameraController.h"
#include "GLObjects.h"

#include <AntTweakBar.h>

#include "BulletInterface.h"


#include <BulletDynamics/Vehicle/btRaycastVehicle.h>




class DeepLearningCarApp : public Application
{
public:
  DeepLearningCarApp(int w = 640, int h = 480, int glMajor = 2, int glMinor = 0, double physicsTimeStep = 1.0 / 60.0);
  virtual ~DeepLearningCarApp();

  virtual void init();

  void initVehicle();

  void initTrack();

  virtual void draw(double time);
  virtual void updatePhysics(double dt);


private:


  void drawCoordSys();

  void drawGrid();

private:

  TwBar* m_bar;


  // camera user controls
  Camera* m_cam;
  CameraController* m_camControl;


  // bullet simulation interface
  BulletInterface* m_bullet;

  // bullet ground plane body
  btRigidBody* m_groundBody;

  // bullet sphere body
  btRigidBody* m_sphereBody;

  std::shared_ptr<btCollisionShape> m_vehicleChassisShape;
  std::shared_ptr<btCompoundShape> m_vehicleChassisCompound;
  btVector3 m_vehicleChassisOffset;
  btVector3 m_vehicleChassisExtents;
  btRaycastVehicle* m_vehicle;
  VehicleControllerUser* m_vehicleController;

  std::vector<unsigned char> m_trackHeights;
  btRigidBody* m_trackBody;


  GL::Program* m_singleColorProg;

  GL::VertexBuffer* m_gridVBO;
  GL::VertexBuffer* m_coordsysVBO;

  GL::Mesh* m_sphereMesh;
  GL::Mesh* m_boxMesh;
  GL::Mesh* m_trackMesh;
};

