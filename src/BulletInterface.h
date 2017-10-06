#pragma once

#include <btBulletDynamicsCommon.h>
#include "UserInputController.h"

#include "GLObjects.h"

#include <memory>
#include <vector>

struct BulletInterface
{
  BulletInterface();
  virtual ~BulletInterface();

  // managed rigid bodies are immediately added to the world and freed on destructor of BulletInterface
  btRigidBody* createManagedRigidBody(std::shared_ptr<btCollisionShape> shape, btScalar mass, const btVector3& pos, bool computeInertia = true);
  btRigidBody* createUnmanagedRigidBody(std::shared_ptr<btCollisionShape> shape, btScalar mass, const btVector3& pos, bool computeInertia = true);

  btRaycastVehicle* createUnmanagedVehicle(std::shared_ptr<btCollisionShape> chassisShape, btScalar mass, const btVector3& pos);


  btBroadphaseInterface* broadphase;
  btDefaultCollisionConfiguration* collisionConfiguration;
  btCollisionDispatcher* dispatcher;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* world;


  std::vector<std::shared_ptr<btCollisionShape>> collisionShapes;
  std::vector<btRigidBody*> rigidBodies;
};






class GLDebugDrawer : public btIDebugDraw
{
public:

  GLDebugDrawer();
  ~GLDebugDrawer();

  virtual void   drawLine(const btVector3& from, const btVector3& to, const btVector3& color);

  virtual void   drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color);

  virtual void   reportErrorWarning(const char* warningString);

  virtual void   draw3dText(const btVector3& location, const char* textString);

  virtual void   setDebugMode(int debugMode);

  virtual int      getDebugMode() const { return m_debugMode; }


  void setViewProj(const glm::mat4& viewProj) { m_viewProj = viewProj; }


private:

  int m_debugMode;

  glm::mat4 m_viewProj;

  GL::Program* m_program;
};




// ARROWS: accelerate, steer and reverse,  SPACE: handbrake
class VehicleControllerUser : public UserInputController
{
public:
  VehicleControllerUser(btRaycastVehicle* vehicle);

  void keyEvent(GLFWwindow* wnd, int key, int scancode, int action, int mods);

  btRaycastVehicle* m_vehicle;

  btScalar m_steer;
  btScalar m_engineForceFwd;
  btScalar m_engineForceRev;
  btScalar m_brake;
};