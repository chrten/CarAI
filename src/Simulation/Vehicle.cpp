
#include <glad/glad.h>

#include "Vehicle.h"

#include "../Application.h"

#include <cstdio>

Vehicle::Vehicle(btRaycastVehicle* bvehicle)
  : m_vehicle(bvehicle), m_controller(0)
{

}

Vehicle::~Vehicle()
{
  delete m_vehicle;
  delete m_controller;
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

VehicleController::VehicleController(Vehicle* vehicle)
  : m_vehicle(vehicle),
  m_steerMax(0.6f),
  m_engineForceFwdMax(5000.0f), m_engineForceRevMax(-3000.0f),
  m_brakeMax(500.0f)
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
        m_vehicle->physics()->setSteeringValue(m_steerMax, 0);
        m_vehicle->physics()->setSteeringValue(m_steerMax, 1);
      }

      if (key == GLFW_KEY_RIGHT)
      {
        m_vehicle->physics()->setSteeringValue(-m_steerMax, 0);
        m_vehicle->physics()->setSteeringValue(-m_steerMax, 1);
      }

      if (key == GLFW_KEY_UP)
      {
        m_vehicle->physics()->applyEngineForce(m_engineForceFwdMax, 2);
        m_vehicle->physics()->applyEngineForce(m_engineForceFwdMax, 3);
      }

      if (key == GLFW_KEY_DOWN)
      {
        m_vehicle->physics()->applyEngineForce(m_engineForceRevMax, 2);
        m_vehicle->physics()->applyEngineForce(m_engineForceRevMax, 3);
      }

      //Handbrake
      if (key == GLFW_KEY_SPACE)
      {
        m_vehicle->physics()->setBrake(m_brakeMax, 2);
        m_vehicle->physics()->setBrake(m_brakeMax, 3);
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

  m_vehicle->physics()->setSteeringValue((-1.0f + 2.0f * u) * m_steerMax, 0);
  m_vehicle->physics()->setSteeringValue((-1.0f + 2.0f * u) * m_steerMax, 1);

  u = static_cast<float>(std::rand()) / RAND_MAX;
  m_vehicle->physics()->applyEngineForce(u * m_engineForceFwdMax, 2);
  m_vehicle->physics()->applyEngineForce(u * m_engineForceFwdMax, 3);
}

