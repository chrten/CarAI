
#include <glad/glad.h>

#include "BulletInterface.h"

BulletInterface::BulletInterface()
  : broadphase(0), collisionConfiguration(0),
  dispatcher(0), solver(0), world(0)
{
  broadphase = new btDbvtBroadphase();

  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);

  solver = new btSequentialImpulseConstraintSolver;

  world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
}

BulletInterface::~BulletInterface()
{
  for (size_t i = 0; i < rigidBodies.size(); ++i)
  {
    btRigidBody* body = rigidBodies[i];

    world->removeRigidBody(body);
    delete body->getMotionState();
    delete body;
  }
  
  rigidBodies.clear();
  collisionShapes.clear();


  delete world;
  delete solver;
  delete collisionConfiguration;
  delete dispatcher;
  delete broadphase;
}

btRigidBody* BulletInterface::createManagedRigidBody(std::shared_ptr<btCollisionShape> shape, 
  btScalar mass,
  const btVector3& pos,
  bool computeInertia)
{
  btRigidBody* body = createUnmanagedRigidBody(shape, mass, pos, computeInertia);

  collisionShapes.push_back(shape);
  rigidBodies.push_back(body);

  return body;
}

btRigidBody* BulletInterface::createUnmanagedRigidBody(std::shared_ptr<btCollisionShape> shape, btScalar mass, const btVector3& pos, bool computeInertia /*= true*/)
{
  btVector3 inertia(0.0, 0.0, 0.0);

  if (computeInertia)
    shape->calculateLocalInertia(mass, inertia);

  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(pos);

  btRigidBody::btRigidBodyConstructionInfo ci(mass,
    new btDefaultMotionState(transform),
    shape.get(), inertia);

  btRigidBody* body = new btRigidBody(ci);

  world->addRigidBody(body);


  return body;
}

btRigidBody* BulletInterface::createUnmanagedRigidBody(std::shared_ptr<btCollisionShape> shape, btScalar mass, const btVector3& pos, int group, int mask, bool computeInertia /*= true*/)
{
  btVector3 inertia(0.0, 0.0, 0.0);

  if (computeInertia)
    shape->calculateLocalInertia(mass, inertia);

  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(pos);

  btRigidBody::btRigidBodyConstructionInfo ci(mass,
    new btDefaultMotionState(transform),
    shape.get(), inertia);

  btRigidBody* body = new btRigidBody(ci);

  world->addRigidBody(body, group, mask);

  return body;
}

btRaycastVehicle* BulletInterface::createUnmanagedVehicle(std::shared_ptr<btCollisionShape> chassisShape, btScalar mass, const btVector3& pos, int group, int mask)
{
  // create rigid body for chassis
  btRigidBody* chassisBody = createUnmanagedRigidBody(chassisShape, mass, pos, group, mask, true);

  // create vehicle
  btVehicleRaycaster* vehicleRayCaster = new btDefaultVehicleRaycaster(world);

  btRaycastVehicle::btVehicleTuning tuning;
  btRaycastVehicle* vehicle = new btRaycastVehicle(tuning, chassisBody, vehicleRayCaster);

  // never deactivate the vehicle
  chassisBody->setActivationState(DISABLE_DEACTIVATION);

  world->addVehicle(vehicle);

  // get chassis box size
  btTransform chassisTransform;
  chassisTransform.setIdentity();
  btVector3 chassisMin, chassisMax;
  chassisShape->getAabb(chassisTransform, chassisMin, chassisMax);
  btVector3 halfExtents = (chassisMax - chassisMin) * 0.5;



  // add wheels
  //The direction of the raycast, the btRaycastVehicle uses raycasts instead of simiulating the wheels with rigid bodies
  btVector3 wheelDirectionCS0(0, -1, 0);

  //The axis which the wheel rotates arround
  btVector3 wheelAxleCS(-1, 0, 0);

  btScalar suspensionRestLength(0.7f);

  btScalar wheelWidth(0.4f);

  btScalar wheelRadius(0.5f);

  //The height where the wheels are connected to the chassis
  btScalar connectionHeight(1.2f);

  //All the wheel configuration assumes the vehicle is centered at the origin and a right handed coordinate system is used
  btVector3 wheelConnectionPoint(halfExtents.x() - wheelRadius, connectionHeight, halfExtents.z() - wheelWidth);

  //Adds the front wheels
  vehicle->addWheel(wheelConnectionPoint, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);

  vehicle->addWheel(wheelConnectionPoint * btVector3(-1, 1, 1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);

  //Adds the rear wheels
  vehicle->addWheel(wheelConnectionPoint* btVector3(1, 1, -1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);

  vehicle->addWheel(wheelConnectionPoint * btVector3(-1, 1, -1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);

  //Configures each wheel of our vehicle, setting its friction, damping compression, etc.
  //For more details on what each parameter does, refer to the docs
  for (int i = 0; i < vehicle->getNumWheels(); i++)
  {
    btWheelInfo& wheel = vehicle->getWheelInfo(i);
    wheel.m_suspensionStiffness = 50;
    wheel.m_wheelsDampingCompression = btScalar(0.3) * 2 * btSqrt(wheel.m_suspensionStiffness);//btScalar(0.8);
    wheel.m_wheelsDampingRelaxation = btScalar(0.5) * 2 * btSqrt(wheel.m_suspensionStiffness);//1;
                                                                                              //Larger friction slips will result in better handling
    wheel.m_frictionSlip = btScalar(1.2);
    wheel.m_rollInfluence = 1;
  }

  return vehicle;
}












GLDebugDrawer::GLDebugDrawer()
  : m_debugMode(0), m_program(0)
{

}


GLDebugDrawer::~GLDebugDrawer()
{
  delete m_program;
}

void    GLDebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
{
  m_lineBuf.push_back(from.getX());
  m_lineBuf.push_back(from.getY());
  m_lineBuf.push_back(from.getZ());

  m_lineBuf.push_back(to.getX());
  m_lineBuf.push_back(to.getY());
  m_lineBuf.push_back(to.getZ());
}

void    GLDebugDrawer::setDebugMode(int debugMode)
{
  m_debugMode = debugMode;
}


void GLDebugDrawer::endDraw()
{
  if (!m_program)
  {
    m_program = new GL::Program();
    m_program->linkFromFile("../data/shaders/simple_vs.glsl", "../data/shaders/simple_fs.glsl");
  }

  if (m_program && m_program->valid())
  {
    m_program->use();
    m_program->setUniform4f("color", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
    m_program->setUniformMatrix4f("WVP", m_viewProj);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12, &m_lineBuf[0]);

    GLsizei n = static_cast<GLsizei>(m_lineBuf.size() / 3);

    glPointSize(5.0f);
    glDrawArrays(GL_POINTS, 0, n);
    glDrawArrays(GL_LINES, 0, n);

    glDisableVertexAttribArray(0);
  }
}

void    GLDebugDrawer::draw3dText(const btVector3& location, const char* textString)
{
  //glRasterPos3f(location.x(),  location.y(),  location.z());
  //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),textString);
}

void    GLDebugDrawer::reportErrorWarning(const char* warningString)
{
  printf(warningString);
}

void    GLDebugDrawer::drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
{
  {
    //btVector3 to=pointOnB+normalOnB*distance;
    //const btVector3&from = pointOnB;
    //glColor4f(color.getX(), color.getY(), color.getZ(), 1.0f);   

    //GLDebugDrawer::drawLine(from, to, color);

    //glRasterPos3f(from.x(),  from.y(),  from.z());
    //char buf[12];
    //sprintf(buf," %d",lifeTime);
    //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
  }
}




void BulletMeshInterface::getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts, PHY_ScalarType& type, int& vertexStride, unsigned char **indexbase, int & indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart)
{
  numverts = m_mesh->numVertices();
  numfaces = m_mesh->numTriangles();

  vertexStride = m_mesh->vertexStride();
  indexstride = 4;

  type = PHY_FLOAT;
  indicestype = PHY_INTEGER;

  *vertexbase = m_mesh->vertexBuffer()->mapBuffer(GL_READ_WRITE);
  *indexbase = m_mesh->indexBuffer()->mapBuffer(GL_READ_WRITE);
}

void BulletMeshInterface::getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts, PHY_ScalarType& type, int& vertexStride, const unsigned char **indexbase, int & indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart) const
{
  numverts = m_mesh->numVertices();
  numfaces = m_mesh->numTriangles();

  vertexStride = m_mesh->vertexStride();
  indexstride = 4;

  type = PHY_FLOAT;
  indicestype = PHY_INTEGER;

  *vertexbase = m_mesh->vertexBuffer()->mapBuffer(GL_READ_ONLY);
  *indexbase = m_mesh->indexBuffer()->mapBuffer(GL_READ_ONLY);
}

void BulletMeshInterface::unLockVertexBase(int subpart)
{
  m_mesh->vertexBuffer()->unmapBuffer();
  m_mesh->indexBuffer()->unmapBuffer();
}

void BulletMeshInterface::unLockReadOnlyVertexBase(int subpart) const
{
  m_mesh->vertexBuffer()->unmapBuffer();
  m_mesh->indexBuffer()->unmapBuffer();
}
