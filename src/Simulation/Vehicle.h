#pragma once

#include "../UserInputController.h"

#include "NeuralNetwork.h"

#include "../BulletInterface.h"


class VehicleController;
class Application;
class Simulation;

class Vehicle
{
public:

  Vehicle(btRaycastVehicle* bvehicle);
  virtual ~Vehicle();


  void setControllerRand();
  void setControllerUser(Application* app);
  void setControllerNeuralNet();

  void update(double dt, Simulation* sim);


  btRaycastVehicle* physics() { return m_vehicle; }
  VehicleController* controller() { return m_controller; }

  NeuralNetwork* neuralNetwork() { return m_neuralNetwork; }


  struct Sensor
  {
    // object space
    btVector3 startOS;
    btVector3 endOS;


    // world space
    btVector3 startWS;
    btVector3 endWS;


    btScalar maxDist;

    btScalar dist;
  };

  void addSensor(const btVector3& start, const btVector3& end);

  int numSensors() const { return static_cast<int>(m_sensors.size()); }
  Sensor* sensor(int i) { return &m_sensors[i]; }


  // Init sensors before neural network!
  // Pass number of neurons for each internal layer.
  // Input layer is given by the number of sensors.
  // The output layer is given by the controller dof.
  void initNeuralNetwork(const std::vector<int>& internalLayerSize);

  static int collisionGroup() { return (1<<3); }


  const int& bestTrackSegment() const { return m_bestSegment; }
  const int& curTrackSegment() const { return m_curSegment; }

  const float& bestTrackDistance() const { return m_bestDistance; }
  const float& curTrackDistance() const { return m_curDistance; }

  const int& travelDir() const { return m_travelDir; }

  const bool& alive() const { return m_alive; }
  void kill() { m_alive = false; }

private:

  // compute distance from start to current position
  // (distance from start to projection of vehicle onto nearest track segment)
  void updateTrackPerformance(Simulation* sim);


private:
  
  btRaycastVehicle* m_vehicle;
  VehicleController* m_controller;

  std::vector<Sensor> m_sensors;

  NeuralNetwork* m_neuralNetwork;


  // performance on track
  int m_bestSegment;
  int m_curSegment;
  int m_travelDir; // -1 : reverse, 0 : stop, 1 : forward
  float m_bestDistance;
  float m_curDistance;

  bool m_alive;
};


class VehicleController
{
public:
  VehicleController(Vehicle* vehicle);
  virtual ~VehicleController() {}

  virtual void update(double dt) {}

protected:

  Vehicle* m_vehicle;

  btScalar m_steerMax;
  btScalar m_engineForceFwdMax;
  btScalar m_engineForceRevMax;
  btScalar m_brakeMax;
};


// ARROWS: accelerate, steer and reverse,  SPACE: handbrake
class VehicleControllerUser : public VehicleController, public UserInputController
{
public:
  VehicleControllerUser(Vehicle* vehicle, Application* app);

  void keyEvent(GLFWwindow* wnd, int key, int scancode, int action, int mods);
};


class VehicleControllerRand : public VehicleController
{
public:

  VehicleControllerRand(Vehicle* vehicle);

  void update(double dt);
};


class VehicleControllerNeuralNet : public VehicleController
{
public:

  VehicleControllerNeuralNet(Vehicle* vehicle);

  void update(double dt);

  // return degrees of freedom
  static int dof();
};