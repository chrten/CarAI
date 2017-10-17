#pragma once

#include "../BulletInterface.h"

#include "Vehicle.h"

#include <string>

#include <AntTweakBar.h>


class Application;

class Simulation
{
public:

  struct Desc 
  {
    Desc() : numCars(20), trackScale(1.0f), trackGroundLevel(1.0f) {}

    int numCars;

    std::string trackHeightsFilename;
    std::string trackSegmentsFilename;
    float trackScale;
    float trackGroundLevel;
  };

  Simulation(const Desc& desc, Application* app);
  virtual ~Simulation();



  void initTweakVars(TwBar* bar);


  void update(double dt);

  btDynamicsWorld* world() { return m_bullet->world; }

  int numVehicles() const { return static_cast<int>(m_vehicles.size()) + (m_vehicleUser ? 1 : 0); }
  Vehicle* vehicle(int i) { return i < static_cast<int>(m_vehicles.size()) ? m_vehicles[i] : m_vehicleUser; }

  Vehicle* bestVehicle() const;

  const std::vector<btVector3>& trackSegments() const { return m_trackSegments; }
  const std::vector<float>& trackSegmentDist() const { return m_trackSegmentDist; }


private:

  void initTrack();
  void initSensors();

  Vehicle* createVehicle();
  btRaycastVehicle* createVehiclePhysics();

  void applyEvolution();

  void resetVehicles();

  static void subtickCallback(btDynamicsWorld* world, btScalar timeStep);


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

  std::vector<EvolutionProcess::Chromosome*> m_chromosomes;
  std::vector<EvolutionProcess::Chromosome*> m_chromosomesNext;
  float m_avgDrivenDistance;

  EvolutionProcess* m_evolution;

  // distance sensor configuration for each vehicle
  std::vector<btVector3> m_sensorConfigStart;
  std::vector<btVector3> m_sensorConfigEnd;


  std::vector<unsigned char> m_trackHeights;
  btRigidBody* m_trackBody;
  std::vector<btVector3> m_trackSegments;
  std::vector<float> m_trackSegmentDist; // accumulated distance from start to segment
};