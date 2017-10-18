#pragma once

#include "../UserInputController.h"

#include "NeuralNetwork.h"
#include "Evolution.h"

#include "../BulletInterface.h"

#include <IniReader.h>


class VehicleController;
class Application;
class Simulation;

class Vehicle
{
public:

  Vehicle(btRaycastVehicle* bvehicle, INIReader* settings);
  virtual ~Vehicle();


  void setControllerRand();
  void setControllerUser(Application* app);
  void setControllerNeuralNet(bool enableBrake);

  void update(double dt, Simulation* sim);


  void replacePhysics(btRaycastVehicle* vehicle, btDynamicsWorld* world);
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
  const double& curTrackSegmentEntryTime() const { return m_curSegmentEntryTime; }
  const int& curLap() const { return m_curLap; }

  const float& bestTrackDistance() const { return m_bestDistance; }
  const float& curTrackDistance() const { return m_curDistance; }

  const int& travelDir() const { return m_travelDir; }


  const bool& alive() const { return m_alive; }
  void kill() { m_alive = false; }
  double birthTime() const { return m_birthTime; }
  void reset();


  struct Chromosome : public EvolutionProcess::Chromosome
  {
    Chromosome(Vehicle* v, float* _avgDrivenDistance) : vehicle(v), avgDrivenDistance(_avgDrivenDistance) { readGenesFromVehicle(); }

    void crossover(const EvolutionProcess::Chromosome* other, float prob, EvolutionProcess::Chromosome* resultA, EvolutionProcess::Chromosome* resultB) const;

    void mutate(float prob);

    float fitness() const;


    void readGenesFromVehicle();
    void transferGenesToVehicle() const;


    Vehicle* vehicle;
    std::vector<float> genes;

    float* avgDrivenDistance;

    static float mutationMaxChange;
  };




  float steerMax() const { return m_steerMax; }
  void steerMax(float f) { m_steerMax = f; }

  float engineForceFwdMax() const { return m_engineForceFwdMax; }
  void engineForceFwdMax(float f) { m_engineForceFwdMax = f; }

  float engineForceRevMax() const { return m_engineForceRevMax; }
  void engineForceRevMax(float f) { m_engineForceRevMax = f; }

  float brakeMax() const { return m_brakeMax; }
  void brakeMax(float f) { m_brakeMax = f; }


private:

  // compute distance from start to current position
  // (distance from start to projection of vehicle onto nearest track segment)
  void updateTrackPerformance(Simulation* sim);


  void initSensors(INIReader* settings);


private:
  
  btRaycastVehicle* m_vehicle;
  VehicleController* m_controller;

  std::vector<Sensor> m_sensors;

  NeuralNetwork* m_neuralNetwork;


  // distance sensor configuration
  static std::string m_sensorConfigFile;
  static std::vector<btVector3> m_sensorConfig; // interleaved line endpoints of distance sensors



  // vehicle limits
  btScalar m_steerMax;
  btScalar m_engineForceFwdMax;
  btScalar m_engineForceRevMax;
  btScalar m_brakeMax;


  // performance on track
  int m_bestSegment;
  int m_curSegment;
  double m_curSegmentEntryTime;
  int m_travelDir; // -1 : reverse, 0 : stop, 1 : forward
  float m_bestDistance;
  float m_curDistance;
  int m_curLap;

  bool m_alive;
  double m_birthTime;
};


class VehicleController
{
public:
  VehicleController(Vehicle* vehicle);
  virtual ~VehicleController() {}

  virtual void update(double dt) {}

protected:

  Vehicle* m_vehicle;
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

  VehicleControllerNeuralNet(Vehicle* vehicle, bool enableBrake = false);

  void update(double dt);

  // return degrees of freedom
  int dof();

private:

  bool m_enableBrake;
};