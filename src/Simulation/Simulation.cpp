
#include <glad/glad.h>

#include "Simulation.h"


#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>


#include <lodepng.h>
#include <iostream>


Simulation::Simulation(const Simulation::Desc& desc, Application* app)
  : m_desc(desc), m_app(app), m_bullet(0), m_groundBody(0), m_sphereBody(0), m_vehicleUser(0),
  m_trackBody(0)
{
  m_bullet = new BulletInterface();
  m_bullet->world->setGravity(btVector3(0, -10, 0));

  //m_groundBody = m_bullet->createManagedRigidBody(std::make_shared<btStaticPlaneShape>(btVector3(0, 1, 0), 1), 0.0, btVector3(0, -1, 0), false);
  //m_groundBody = m_bullet->createManagedRigidBody(std::make_shared<btBoxShape>(btVector3(100, 1, 100)), 0.0, btVector3(0, -1, 0), false);
//   m_sphereBody = m_bullet->createManagedRigidBody(std::make_shared<btSphereShape>(0.5f), 1.0, btVector3(-3, 2, -1), true);
//   m_sphereBody = m_bullet->createManagedRigidBody(std::make_shared<btSphereShape>(0.5f), 1.0, btVector3(3, 2, 1), true);
//   m_sphereBody = m_bullet->createManagedRigidBody(std::make_shared<btSphereShape>(0.5f), 1.0, btVector3(3, 2, -1), true);


  m_vehicleUser = createVehicle();
  m_vehicleUser->setControllerUser(m_app);


  m_vehicles.resize(desc.numCars, 0);
  std::vector<int> internalNetworkLayers(2);
  internalNetworkLayers[0] = 4;
  internalNetworkLayers[1] = 3;
  for (int i = 0; i < desc.numCars; ++i)
  {
    m_vehicles[i] = createVehicle();
    m_vehicles[i]->initNeuralNetwork(internalNetworkLayers);
    m_vehicles[i]->setControllerNeuralNet();
  }

  initTrack();
}

Simulation::~Simulation()
{
  if (m_bullet)
  {
    if (m_vehicleUser)
    {
      m_bullet->world->removeRigidBody(m_vehicleUser->physics()->getRigidBody());
      delete m_vehicleUser->physics()->getRigidBody();
      m_bullet->world->removeVehicle(m_vehicleUser->physics());
      delete m_vehicleUser;
    }

    delete m_bullet;
  }

  for (size_t i = 0; i < m_vehicles.size(); ++i)
    delete m_vehicles[i];
}

void Simulation::update(double dt)
{
  // update bullet world
  m_bullet->world->stepSimulation(static_cast<btScalar>(dt), 10);


  int n = numVehicles();
  for (int i = 0; i < n; ++i)
    vehicle(i)->update(dt, m_bullet->world);
}

void Simulation::initTrack()
{
  // load from heightfield
  unsigned int w, h;
  if (!lodepng::decode(m_trackHeights, w, h, m_desc.trackFilename, LCT_GREY, 8u))
  {
    std::shared_ptr<btHeightfieldTerrainShape> trackShape = std::make_shared<btHeightfieldTerrainShape>(w, h, &m_trackHeights[0], 10.0f / 256.0f, 0.0f, 10.0f, 1, PHY_UCHAR, false);

    trackShape->setLocalScaling(btVector3(m_desc.trackScale, m_desc.trackScale, m_desc.trackScale));

    // bullet shifts the heightfield, so that the aabb center is the origin
    btTransform idt;
    idt.setIdentity();
    btVector3 aabbMin, aabbMax;
    trackShape->getAabb(idt, aabbMin, aabbMax);
    btVector3 diag = (aabbMax - aabbMin);

    // shift ground level to 1
    btVector3 shift(0.0f, diag[1] * 0.5f + 1.0f, 0.0f);

    m_trackBody = m_bullet->createManagedRigidBody(trackShape, 0.0f, shift, false);
  }
  else
    std::cout << "failed to load track" << std::endl;
}

void Simulation::initSensors()
{
  m_sensorConfigStart.resize(3);
  m_sensorConfigEnd.resize(3);

  float config[] =
  {
    0.0209f, 1.5000f, 1.0072f,
    0.0209f, 1.5000f, 5.0666f,
    -0.5070f, 1.5000f, 0.9990f,
    -3.1516f, 1.5000f, 4.2972f,
    0.4965f, 1.5000f, 1.0095f,
    3.3649f, 1.5000f, 4.4035f
  };

//   float config[] =
//   {
//     0,0,0,  1,0,0,
//     0,0,0,  0,1,0,
//     0,0,0,  0,0,1
//   };

  float scale = 5.0f;

  for (int i = 0; i < 3; ++i)
  { 
    m_sensorConfigStart[i] = btVector3(config[i * 6 + 0], config[i * 6 + 1], config[i * 6 + 2]);
    m_sensorConfigEnd[i] = btVector3(config[i * 6 + 3], config[i * 6 + 4], config[i * 6 + 5]);

    m_sensorConfigEnd[i] = m_sensorConfigStart[i] + (m_sensorConfigEnd[i] - m_sensorConfigStart[i]) * scale;
  }
}

Vehicle* Simulation::createVehicle()
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


  if (m_sensorConfigStart.empty())
    initSensors();


  btRaycastVehicle* bvehicle = m_bullet->createUnmanagedVehicle(m_vehicleChassisCompound, 1200, btVector3(0.0, 1.0, 0.0), Vehicle::collisionGroup(), ~Vehicle::collisionGroup());
  Vehicle* vehicle = new Vehicle(bvehicle);

  for (size_t i = 0; i < m_sensorConfigStart.size(); ++i)
    vehicle->addSensor(m_sensorConfigStart[i], m_sensorConfigEnd[i]);

  return vehicle;
}
