#pragma once

#include "../BulletInterface.h"

#include "Vehicle.h"

#include <string>


class Application;

class Simulation
{
public:

  struct Desc 
  {
    int numCars;
    std::string trackFilename;
  };

  Simulation(const Desc& desc, Application* app);
  virtual ~Simulation();



  void update(double dt);



  void initTrack();
  Vehicle* createVehicle();


public:

  Desc m_desc;

  Application* m_app;

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

  Vehicle* m_vehicle;

  std::vector<unsigned char> m_trackHeights;
  btRigidBody* m_trackBody;
};