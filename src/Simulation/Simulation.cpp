
#include <glad/glad.h>

#include "Simulation.h"


#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>


#include <lodepng.h>
#include <iostream>


Simulation::Simulation(const Simulation::Desc& desc, Application* app)
  : m_desc(desc), m_app(app), m_bullet(0), m_groundBody(0), m_sphereBody(0), m_vehicle(0),
  m_trackBody(0)
{
  m_bullet = new BulletInterface();
  m_bullet->world->setGravity(btVector3(0, -10, 0));

  //m_groundBody = m_bullet->createManagedRigidBody(std::make_shared<btStaticPlaneShape>(btVector3(0, 1, 0), 1), 0.0, btVector3(0, -1, 0), false);
  //m_groundBody = m_bullet->createManagedRigidBody(std::make_shared<btBoxShape>(btVector3(100, 1, 100)), 0.0, btVector3(0, -1, 0), false);
  m_sphereBody = m_bullet->createManagedRigidBody(std::make_shared<btSphereShape>(0.5f), 1.0, btVector3(-3, 2, -1), true);
  m_sphereBody = m_bullet->createManagedRigidBody(std::make_shared<btSphereShape>(0.5f), 1.0, btVector3(3, 2, 1), true);
  m_sphereBody = m_bullet->createManagedRigidBody(std::make_shared<btSphereShape>(0.5f), 1.0, btVector3(3, 2, -1), true);

  m_vehicle = createVehicle();
  m_vehicle->setControllerUser(m_app);

  initTrack();
}

Simulation::~Simulation()
{
  if (m_bullet)
  {
    if (m_vehicle)
    {
      m_bullet->world->removeRigidBody(m_vehicle->physics()->getRigidBody());
      delete m_vehicle->physics()->getRigidBody();
      m_bullet->world->removeVehicle(m_vehicle->physics());
      delete m_vehicle;
    }

    delete m_bullet;
  }
}

void Simulation::update(double dt)
{
  // update bullet world
  m_bullet->world->stepSimulation(static_cast<btScalar>(dt), 10);

  if (m_vehicle && m_vehicle->controller())
    m_vehicle->controller()->update(dt);
}

void Simulation::initTrack()
{
  // load from heightfield
  unsigned int w, h;
  if (!lodepng::decode(m_trackHeights, w, h, m_desc.trackFilename, LCT_GREY, 8u))
  {
    std::shared_ptr<btHeightfieldTerrainShape> trackShape = std::make_shared<btHeightfieldTerrainShape>(w, h, &m_trackHeights[0], 10.0f / 256.0f, 0.0f, 10.0f, 1, PHY_UCHAR, false);
    m_trackBody = m_bullet->createManagedRigidBody(trackShape, 0.0f, btVector3(0.0f, 0.0f, 0.0f), false);
  }
  else
    std::cout << "failed to load track" << std::endl;
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

  btRaycastVehicle* bvehicle = m_bullet->createUnmanagedVehicle(m_vehicleChassisCompound, 1200, btVector3(0.0, 1.0, 0.0), 4, ~4);
  return new Vehicle(bvehicle);
}
