#pragma once

#include "../UserInputController.h"

#include "NeuralNetwork.h"

#include "../BulletInterface.h"


class VehicleController;
class Application;



class Vehicle
{
public:

  Vehicle(btRaycastVehicle* bvehicle);
  virtual ~Vehicle();


  void setControllerRand();
  void setControllerUser(Application* app);
  void setControllerNeuralNet();

  void update(double dt, btDynamicsWorld* world);


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

private:

  
  btRaycastVehicle* m_vehicle;
  VehicleController* m_controller;

  std::vector<Sensor> m_sensors;

  NeuralNetwork* m_neuralNetwork;
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