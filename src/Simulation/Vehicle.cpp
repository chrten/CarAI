
#include <glad/glad.h>

#include "Vehicle.h"
#include "Simulation.h"

#include "../Application.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstdio>



float Vehicle::Chromosome::mutationMaxChange = 0.6f;

std::string Vehicle::m_sensorConfigFile = "";
std::vector<btVector3> Vehicle::m_sensorConfig;


Vehicle::Vehicle(btRaycastVehicle* bvehicle, INIReader* settings)
  : m_vehicle(bvehicle), m_controller(0), m_neuralNetwork(0),
  m_steerMax(0.6f),
  m_engineForceFwdMax(5000.0f), m_engineForceRevMax(-3000.0f),
  m_brakeMax(500.0f),
  m_bestSegment(0), m_curSegment(0), m_travelDir(0), m_bestDistance(0.0f), m_curDistance(0.0f), m_curLap(0),
  m_alive(true)
{
  m_birthTime = glfwGetTime();
  m_curSegmentEntryTime = m_birthTime;

  initSensors(settings);


  m_steerMax = static_cast<float>(settings->GetReal("vehicle", "steerMax", m_steerMax));
  m_engineForceFwdMax = static_cast<float>(settings->GetReal("vehicle", "engineForceFwdMax", m_engineForceFwdMax));
  m_engineForceRevMax = static_cast<float>(settings->GetReal("vehicle", "engineForceRevMax", m_engineForceRevMax));
  m_brakeMax = static_cast<float>(settings->GetReal("vehicle", "brakeMax", m_brakeMax));
}

Vehicle::~Vehicle()
{
  delete m_vehicle;
  delete m_controller;
  delete m_neuralNetwork;
}




void Vehicle::setControllerRand()
{
  delete m_controller;
  m_controller = new VehicleControllerRand(this);
}


void Vehicle::setControllerUser(Application* app)
{
  delete m_controller;
  m_controller = new VehicleControllerUser(this, app);
}


void Vehicle::setControllerNeuralNet(bool enableBrake)
{
  delete m_controller;
  m_controller = new VehicleControllerNeuralNet(this, enableBrake);
}

void Vehicle::update(double dt, Simulation* sim)
{
  // update sensors
  for (int i = 0; i < numSensors(); ++i)
  {
    Sensor* s = sensor(i);

    s->startWS = m_vehicle->getChassisWorldTransform() * s->startOS;
    s->endWS = m_vehicle->getChassisWorldTransform() * s->endOS;

    btCollisionWorld::ClosestRayResultCallback hit(s->startWS, s->endWS);

    // ignore other vehicles in the simulation
    hit.m_collisionFilterGroup = collisionGroup();
    hit.m_collisionFilterMask = ~collisionGroup();

    sim->world()->rayTest(hit.m_rayFromWorld, hit.m_rayToWorld, hit);
    
    s->dist = s->maxDist;
    if (hit.hasHit())
      s->dist *= hit.m_closestHitFraction;
  }

  // update performance
  updateTrackPerformance(sim);


  if (m_controller)
    m_controller->update(dt);
}


void Vehicle::replacePhysics(btRaycastVehicle* vehicle, btDynamicsWorld* world)
{
  if (m_vehicle)
  {
    world->removeRigidBody(m_vehicle->getRigidBody());
    world->removeVehicle(m_vehicle);

    delete m_vehicle->getRigidBody()->getMotionState();
    delete m_vehicle->getRigidBody();

    delete m_vehicle;
  }
  
  m_vehicle = vehicle;
}

void Vehicle::addSensor(const btVector3& start, const btVector3& end)
{
  Sensor s;
  s.startOS = start;
  s.endOS = end;

  s.maxDist = (start - end).norm();
  s.dist = s.maxDist;

  s.startWS = m_vehicle->getChassisWorldTransform() * s.startOS;
  s.endWS = m_vehicle->getChassisWorldTransform() * s.endOS;

  m_sensors.push_back(s);
}


void Vehicle::initNeuralNetwork(const std::vector<int>& internalLayerSize)
{
  delete m_neuralNetwork;
  m_neuralNetwork = new NeuralNetwork();


  // input layer:  distance sensors + speed
  m_neuralNetwork->addLayer(numSensors() + 1);

  // internal layers
  for (size_t i = 0; i < internalLayerSize.size(); ++i)
    m_neuralNetwork->addLayer(internalLayerSize[i]);

  // output layer
  m_neuralNetwork->addLayer(dynamic_cast<VehicleControllerNeuralNet*>(m_controller)->dof());

  // init with random weights
  for (int i = 0; i < m_neuralNetwork->numLinks(); ++i)
    m_neuralNetwork->links(i)->randomize(-1.0f, 1.0f);
}


void Vehicle::updateTrackPerformance(Simulation* sim)
{
  const std::vector<btVector3>& segments = sim->trackSegments();
  const std::vector<float>& distances = sim->trackSegmentDist();


  // brute force: find nearest segment
  int nearestSeg = -1;
  float nearestSegDist = -1.0f;
  btVector3 nearestSegProj;
  float trackDist = -1.0f;

  btVector3 vpos = m_vehicle->getChassisWorldTransform().getOrigin();

  int nsegs = static_cast<int>(sim->trackSegments().size());
  for (int i = 0; i < nsegs; ++i)
  {
    // project vehicle pos onto segment
    btVector3 a = segments[i];
    btVector3 n = segments[(i + 1) % nsegs] - a;
    btScalar len = n.safeNorm();
    n.safeNormalize();

    btScalar p = (vpos - a).dot(n);

    if (0.0f <= p && p <= len)
    {
      btVector3 segProj = a + n * p;
      float segDist = (segProj - vpos).norm();

      // segment found
      if (nearestSeg < 0 || segDist < nearestSegDist)
      {
        nearestSeg = i;
        nearestSegProj = segProj;
        nearestSegDist = segDist;

        trackDist = distances[i] + p;
      }
    }
  }

  if (nearestSeg >= 0)
  {
    // update travel direction
    const btVector3& vel = m_vehicle->getRigidBody()->getLinearVelocity();
    btVector3 n = segments[(nearestSeg + 1) % nsegs] - segments[nearestSeg];

    btScalar vdotn = vel.dot(n);

    if (vel.dot(vel) < 1e-6f)
      m_travelDir = 0;
    else if (vdotn > 0)
      m_travelDir = 1;
    else if (vdotn < 0)
      m_travelDir = -1;

    // update segment info
    if (m_travelDir > 0 && (nearestSeg < m_bestSegment + 5))
    {
      if (!nearestSeg && m_curSegment == nsegs - 1)
        ++m_curLap;

      m_bestSegment = std::max(m_bestSegment, nearestSeg);
      m_bestDistance = std::max(m_bestDistance, trackDist);

      m_curDistance = trackDist + static_cast<float>(m_curLap) * distances.back();

      if (m_curSegment != nearestSeg)
        m_curSegmentEntryTime = glfwGetTime();
    
      m_curSegment = nearestSeg;
    }
  }
}


void Vehicle::initSensors(INIReader* settings)
{
  if (m_sensorConfig.empty())
  {
    std::string filename = settings->Get("vehicle", "sensors", "../data/obj/sensors.obj");
    float scale = static_cast<float>(settings->GetReal("vehicle", "sensorScale", 5.0f));

    std::ifstream file(filename);
    if (file.is_open())
    {
      int start = 0;
      for (std::string line; std::getline(file, line);)
      {
        float v[3];
        if (sscanf(line.c_str(), "v %f %f %f", v, v + 1, v + 2) == 3)
          m_sensorConfig.push_back(btVector3(v[0], v[1], v[2]));
      }

      file.close();
    }
    else
    {
      m_sensorConfig.resize(6);

      float config[] =
      {
        0.0209f, 1.5000f, 1.0072f,
        0.0209f, 1.5000f, 5.0666f,
        -0.5070f, 1.5000f, 0.9990f,
        -3.1516f, 1.5000f, 4.2972f,
        0.4965f, 1.5000f, 1.0095f,
        3.3649f, 1.5000f, 4.4035f
      };

      //      float config[] =
      //      {
      //        0,0,0,  1,0,0,
      //        0,0,0,  0,1,0,
      //        0,0,0,  0,0,1
      //      };

      for (int i = 0; i < 6; ++i)
        m_sensorConfig[i] = btVector3(config[i * 3], config[i * 3 + 1], config[i * 3 + 2]);
    }

    // scale sensor max distance
    for (size_t i = 0; i < m_sensorConfig.size() / 2; ++i)
      m_sensorConfig[i * 2 + 1] = m_sensorConfig[i * 2] + (m_sensorConfig[i * 2 + 1] - m_sensorConfig[i * 2]) * scale;
  }

  for (size_t i = 0; i < m_sensorConfig.size() / 2; ++i)
    addSensor(m_sensorConfig[i * 2], m_sensorConfig[i * 2 + 1]);
}


void Vehicle::reset()
{
  // reanimate vehicle and restore initial state of simulation
  m_bestDistance = 0.0f;
  m_curDistance = 0.0f;
  m_bestSegment = 0;
  m_curSegment = 0;
  m_curLap = 0;

  m_alive = true;
  m_birthTime = glfwGetTime();
  m_curSegmentEntryTime = m_birthTime;

  btRigidBody* body = m_vehicle->getRigidBody();

  body->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
  //body->setAngularFactor(0.0f);
  body->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));

  btTransform initState;
  initState.setIdentity();
  body->setWorldTransform(initState);
  body->getMotionState()->setWorldTransform(initState);
  body->forceActivationState(DISABLE_DEACTIVATION);
  body->clearForces();


  for (int i = 0; i < m_vehicle->getNumWheels(); ++i)
  {
    m_vehicle->setSteeringValue(0, i);
    m_vehicle->applyEngineForce(0, i);
    m_vehicle->setBrake(0, i);
  }

  m_birthTime = glfwGetTime();
}

VehicleController::VehicleController(Vehicle* vehicle)
  : m_vehicle(vehicle)
{

}

VehicleControllerUser::VehicleControllerUser(Vehicle* vehicle, Application* app)
  : VehicleController(vehicle)
{
  app->addUserInputController(this);
}

void VehicleControllerUser::keyEvent(GLFWwindow* wnd, int key, int scancode, int action, int mods)
{
  if (m_vehicle)
  {
    if (action != GLFW_RELEASE)
    {
      if (key == GLFW_KEY_LEFT)
      {
        m_vehicle->physics()->setSteeringValue(m_vehicle->steerMax(), 0);
        m_vehicle->physics()->setSteeringValue(m_vehicle->steerMax(), 1);
      }

      if (key == GLFW_KEY_RIGHT)
      {
        m_vehicle->physics()->setSteeringValue(-m_vehicle->steerMax(), 0);
        m_vehicle->physics()->setSteeringValue(-m_vehicle->steerMax(), 1);
      }

      if (key == GLFW_KEY_UP)
      {
        m_vehicle->physics()->applyEngineForce(m_vehicle->engineForceFwdMax(), 2);
        m_vehicle->physics()->applyEngineForce(m_vehicle->engineForceFwdMax(), 3);
      }

      if (key == GLFW_KEY_DOWN)
      {
        m_vehicle->physics()->applyEngineForce(m_vehicle->engineForceRevMax(), 2);
        m_vehicle->physics()->applyEngineForce(m_vehicle->engineForceRevMax(), 3);
      }

      //Handbrake
      if (key == GLFW_KEY_SPACE)
      {
        m_vehicle->physics()->setBrake(m_vehicle->brakeMax(), 2);
        m_vehicle->physics()->setBrake(m_vehicle->brakeMax(), 3);
      }
    }
    else
    {
      if (key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT)
      {
        m_vehicle->physics()->setSteeringValue(0, 0);
        m_vehicle->physics()->setSteeringValue(0, 1);
      }

      if (key == GLFW_KEY_UP || key == GLFW_KEY_DOWN)
      {
        m_vehicle->physics()->applyEngineForce(0, 2);
        m_vehicle->physics()->applyEngineForce(0, 3);

        //Default braking force, always added otherwise there is no friction on the wheels
        m_vehicle->physics()->setBrake(10, 2);
        m_vehicle->physics()->setBrake(10, 3);
      }

      //Handbrake
      if (key == GLFW_KEY_SPACE)
      {
        m_vehicle->physics()->setBrake(0, 2);
        m_vehicle->physics()->setBrake(0, 3);
      }
    }
  }
}

VehicleControllerRand::VehicleControllerRand(Vehicle* vehicle) : VehicleController(vehicle)
{

}

void VehicleControllerRand::update(double dt)
{
  float u = static_cast<float>(std::rand()) / RAND_MAX;

  m_vehicle->physics()->setSteeringValue((-1.0f + 2.0f * u) * m_vehicle->steerMax(), 0);
  m_vehicle->physics()->setSteeringValue((-1.0f + 2.0f * u) * m_vehicle->steerMax(), 1);

  u = static_cast<float>(std::rand()) / RAND_MAX;
  m_vehicle->physics()->applyEngineForce(u * m_vehicle->engineForceFwdMax(), 2);
  m_vehicle->physics()->applyEngineForce(u * m_vehicle->engineForceFwdMax(), 3);
}


VehicleControllerNeuralNet::VehicleControllerNeuralNet(Vehicle* vehicle, bool enableBrake)
  : VehicleController(vehicle), m_enableBrake(enableBrake)
{
  vehicle->steerMax(1.0f);
  vehicle->engineForceFwdMax(4000.0f);
  vehicle->engineForceRevMax(-2000.0f);
}

void VehicleControllerNeuralNet::update(double dt)
{
  // get sensor data
  int ns = m_vehicle->numSensors();
  std::vector<float> input(ns + 1);

  for (int i = 0; i < ns; ++i)
    input[i] = m_vehicle->sensor(i)->dist;

  // get speed
  input[ns] = m_vehicle->physics()->getRigidBody()->getLinearVelocity().norm();


  // get output from neural network
  std::vector<float> output;
  if (m_vehicle->neuralNetwork()->compute(input, output))
  {
    // apply to vehicle
    btRaycastVehicle* v = m_vehicle->physics();


    // clamp to allowed values
    float steer = output[0] * m_vehicle->steerMax();
    
    float force = output[1] * 0.5f + 0.5f; // map [-1,1] to [0,1]
    force = m_vehicle->engineForceRevMax() + (m_vehicle->engineForceFwdMax() - m_vehicle->engineForceRevMax()) * force; // lerp


    v->setSteeringValue(steer, 0);
    v->setSteeringValue(steer, 1);

    v->applyEngineForce(force, 2);
    v->applyEngineForce(force, 3);

    if (m_enableBrake)
    {
      float brake = output[2] * m_vehicle->brakeMax();
      v->setBrake(brake, 2);
      v->setBrake(brake, 3);
    }
  }
  else
    std::cerr << "error: could not execute neural network" << std::endl;
}

int VehicleControllerNeuralNet::dof()
{
  // dofs: steer, engine force, brake
  return m_enableBrake ? 3 : 2;
}


void Vehicle::Chromosome::crossover(const EvolutionProcess::Chromosome* _other, float prob, EvolutionProcess::Chromosome* _resultA, EvolutionProcess::Chromosome* _resultB) const
{
  const Chromosome* other = dynamic_cast<const Chromosome*>(_other);
  Chromosome* resultA = dynamic_cast<Chromosome*>(_resultA);
  Chromosome* resultB = dynamic_cast<Chromosome*>(_resultB);

  size_t n = genes.size();

  for (size_t i = 0; i < n; ++i)
  {
    const Chromosome* a = this;
    const Chromosome* b = other;

    float u = static_cast<float>(std::rand()) / RAND_MAX;
    if (u < prob)
      std::swap(a, b);
    
    resultA->genes[i] = a->genes[i];
    if (resultB)
      resultB->genes[i] = b->genes[i];
  }
}

void Vehicle::Chromosome::mutate(float prob)
{
  size_t n = genes.size();

  for (size_t i = 0; i < n; ++i)
  {
    float u = static_cast<float>(std::rand()) / RAND_MAX;
    if (u < prob)
    {
      float m = static_cast<float>(std::rand()) / RAND_MAX * 2.0f - 1.0f;
      genes[i] += m * mutationMaxChange;
    }
  }
}

float Vehicle::Chromosome::fitness() const
{
  return vehicle->curTrackDistance() / *avgDrivenDistance;
}

void Vehicle::Chromosome::readGenesFromVehicle()
{
  NeuralNetwork* net = vehicle->neuralNetwork();
  int nl = net->numLinks();

  genes.clear();
  for (int i = 0; i < nl; ++i)
  {
    NeuralNetwork::LayerLinks* links = net->links(i);
    genes.insert(genes.end(), links->weights.begin(), links->weights.end());
  }
}

void Vehicle::Chromosome::transferGenesToVehicle() const
{
  NeuralNetwork* net = vehicle->neuralNetwork();
  int nl = net->numLinks();

  int iter = 0;
  for (int i = 0; i < nl; ++i)
  {
    NeuralNetwork::LayerLinks* links = net->links(i);
    for (size_t k = 0; k < links->weights.size(); ++k)
      links->weights[k] = genes[iter++];
  }
}
