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

  btRaycastVehicle* physics() { return m_vehicle; }
  VehicleController* controller() { return m_controller; }

  NeuralNetwork* neuralNetwork() { return m_neuralNetwork; }

private:

  btRaycastVehicle* m_vehicle;
  VehicleController* m_controller;

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