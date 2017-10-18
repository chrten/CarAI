#pragma once

#include "../BulletInterface.h"

#include "Vehicle.h"

#include <string>

#include <AntTweakBar.h>


class Application;

class Simulation
{
public:

  Simulation(INIReader* settings, Application* app);
  virtual ~Simulation();



  void initTweakVars(TwBar* bar);


  void update(double dt);

  btDynamicsWorld* world() { return m_bullet->world; }

  int numVehicles() const { return static_cast<int>(m_vehicles.size()) + (m_vehicleUser ? 1 : 0); }
  Vehicle* vehicle(int i) { return i < static_cast<int>(m_vehicles.size()) ? m_vehicles[i] : m_vehicleUser; }

  Vehicle* bestVehicle() const;

  const std::vector<btVector3>& trackSegments() const { return m_trackSegments; }
  const std::vector<float>& trackSegmentDist() const { return m_trackSegmentDist; }


  btRigidBody* trackBody() { return m_trackBody; }

  Vehicle* userVehicle() { return m_vehicleUser; }

private:

  void initTrack();

  Vehicle* createVehicle();
  btRaycastVehicle* createVehiclePhysics();

  void applyEvolution();

  void resetVehicles();

  // iterate over all contact pairs in bullet collision lib
  static void subtickCallback(btDynamicsWorld* world, btScalar timeStep);


private:

  struct Desc
  {
    Desc() : numCars(20), trackScale(1.0f), trackGroundLevel(1.0f) {}

    int numCars;

    std::string trackHeightsFilename;
    std::string trackSegmentsFilename;
    float trackScale;
    float trackGroundLevel;
  };

  Desc m_desc;
  INIReader* m_settings;

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
  float m_bestDrivenDistance;
  int m_numVehiclesAlive;

  EvolutionProcess* m_evolution;

  std::vector<unsigned char> m_trackHeights;
  btRigidBody* m_trackBody;
  std::vector<btVector3> m_trackSegments;
  std::vector<float> m_trackSegmentDist; // accumulated distance from start to segment
};