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
  btRigidBody* createUnmanagedRigidBody(std::shared_ptr<btCollisionShape> shape, btScalar mass, const btVector3& pos, int group, int mask, bool computeInertia = true);

  btRaycastVehicle* createUnmanagedVehicle(std::shared_ptr<btCollisionShape> chassisShape, btScalar mass, const btVector3& pos, int group, int mask);


  btBroadphaseInterface* broadphase;
  btDefaultCollisionConfiguration* collisionConfiguration;
  btCollisionDispatcher* dispatcher;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* world;


  std::vector<std::shared_ptr<btCollisionShape>> collisionShapes;
  std::vector<btRigidBody*> rigidBodies;
};




class BulletMeshInterface : public btStridingMeshInterface
{
public:

  BulletMeshInterface(GL::Mesh* mesh) : m_mesh(mesh) {}
  virtual ~BulletMeshInterface() {}


  void getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts, PHY_ScalarType& type, int& vertexStride, unsigned char **indexbase, int & indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart);
  void getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts, PHY_ScalarType& type, int& vertexStride, const unsigned char **indexbase, int & indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart) const;

  virtual void	unLockVertexBase(int subpart);
  virtual void	unLockReadOnlyVertexBase(int subpart) const;


  virtual int		getNumSubParts() const { return 1; }

  virtual void	preallocateVertices(int numverts) {}
  virtual void	preallocateIndices(int numindices) {}


  GL::Mesh* mesh() const { return m_mesh; }

private:
  GL::Mesh* m_mesh;
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


  void beginDraw() { m_lineBuf.clear(); }
  void endDraw();


private:

  int m_debugMode;

  glm::mat4 m_viewProj;

  GL::Program* m_program;


  std::vector<float> m_lineBuf;
};