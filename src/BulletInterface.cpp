
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

btRaycastVehicle* BulletInterface::createUnmanagedVehicle(std::shared_ptr<btCollisionShape> chassisShape, btScalar mass, const btVector3& pos)
{
  // create rigid body for chassis
  btRigidBody* chassisBody = createUnmanagedRigidBody(chassisShape, mass, pos, true);

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

  btScalar suspensionRestLength(0.7);

  btScalar wheelWidth(0.4);

  btScalar wheelRadius(0.5);

  //The height where the wheels are connected to the chassis
  btScalar connectionHeight(1.2);

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
  //      if (m_debugMode > 0)

  if (!m_program)
  {
    m_program = new GL::Program();
    m_program->linkFromFile("../data/shaders/simple_vs.glsl", "../data/shaders/simple_fs.glsl");
  }

  if (m_program && m_program->valid())
  {
    float tmp[6] = { from.getX(), from.getY(), from.getZ(),
      to.getX(), to.getY(), to.getZ() };

    m_program->use();
    m_program->setUniform4f("color", glm::vec4(color[0], color[1], color[2], 1.0f));
    m_program->setUniformMatrix4f("WVP", m_viewProj);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12, tmp);

    glPointSize(5.0f);
    glDrawArrays(GL_POINTS, 0, 2);
    glDrawArrays(GL_LINES, 0, 2);

    glDisableVertexAttribArray(0);
  }
}

void    GLDebugDrawer::setDebugMode(int debugMode)
{
  m_debugMode = debugMode;
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




VehicleControllerUser::VehicleControllerUser(btRaycastVehicle* vehicle) 
  : m_vehicle(vehicle),
  m_steer(0.3f),
  m_engineForceFwd(5000.0f), m_engineForceRev(-3000.0f),
  m_brake(500.0f)
{

}

void VehicleControllerUser::keyEvent(GLFWwindow* wnd, int key, int scancode, int action, int mods)
{
  if (m_vehicle)
  {
    if (action != GLFW_RELEASE)
    {
      if (key == GLFW_KEY_LEFT)
      {
        m_vehicle->setSteeringValue(m_steer, 0);
        m_vehicle->setSteeringValue(m_steer, 1);
      }

      if (key == GLFW_KEY_RIGHT)
      {
        m_vehicle->setSteeringValue(-m_steer, 0);
        m_vehicle->setSteeringValue(-m_steer, 1);
      }

      if (key == GLFW_KEY_UP)
      {
        m_vehicle->applyEngineForce(m_engineForceFwd, 2);
        m_vehicle->applyEngineForce(m_engineForceFwd, 3);
      }

      if (key == GLFW_KEY_DOWN)
      {
        m_vehicle->applyEngineForce(m_engineForceRev, 2);
        m_vehicle->applyEngineForce(m_engineForceRev, 3);
      }

      //Handbrake
      if (key == GLFW_KEY_SPACE)
      {
        m_vehicle->setBrake(m_brake, 2);
        m_vehicle->setBrake(m_brake, 3);
      }
    }
    else
    {
      if (key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT)
      {
        m_vehicle->setSteeringValue(0, 0);
        m_vehicle->setSteeringValue(0, 1);
      }

      if (key == GLFW_KEY_UP || key == GLFW_KEY_DOWN)
      {
        m_vehicle->applyEngineForce(0, 2);
        m_vehicle->applyEngineForce(0, 3);

        //Default braking force, always added otherwise there is no friction on the wheels
        m_vehicle->setBrake(10, 2);
        m_vehicle->setBrake(10, 3);
      }

      //Handbrake
      if (key == GLFW_KEY_SPACE)
      {
        m_vehicle->setBrake(0, 2);
        m_vehicle->setBrake(0, 3);
      }
    }
  }
}
