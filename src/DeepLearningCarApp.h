#pragma once

#include "Renderer.h"

#include "Application.h"

#include "CameraController.h"

#include "Simulation/Simulation.h"

class DeepLearningCarApp : public Application
{
public:
  DeepLearningCarApp(int w = 640, int h = 480, int glMajor = 2, int glMinor = 0, double physicsTimeStep = 1.0 / 60.0);
  virtual ~DeepLearningCarApp();

  virtual void init();

  void initVehicle();

  void initTrack();

  virtual void draw(double time);
  virtual void updatePhysics(double dt);


private:

  // camera user controls
  Camera* m_cam;
  CameraController* m_camControl;

  Simulation* m_simulation;
  
  Renderer* m_renderer;
};

