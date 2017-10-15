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
    Desc() : numCars(20), trackScale(1.0f) {}

    int numCars;

    std::string trackFilename;
    float trackScale;
  };

  Simulation(const Desc& desc, Application* app);
  virtual ~Simulation();



  void update(double dt);



  void initTrack();
  void initSensors();

  Vehicle* createVehicle();


  int numVehicles() const { return static_cast<int>(m_vehicles.size()) + (m_vehicleUser ? 1 : 0); }

  Vehicle* vehicle(int i) { return i < static_cast<int>(m_vehicles.size()) ? m_vehicles[i] : m_vehicleUser; }



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

  Vehicle* m_vehicleUser;
  std::vector<Vehicle*> m_vehicles;

  // distance sensor configuration for each vehicle
  std::vector<btVector3> m_sensorConfigStart;
  std::vector<btVector3> m_sensorConfigEnd;


  std::vector<unsigned char> m_trackHeights;
  btRigidBody* m_trackBody;
};