
#include "UserInputController.h"

#include <AntTweakbar.h>


void AntTweakbarInputController::resizeEvent(GLFWwindow* wnd, int w, int h)
{
  TwWindowSize(w, h);
}

void AntTweakbarInputController::mouseButtonEvent(GLFWwindow* wnd, int button, int action, int mods)
{
  TwEventMouseButtonGLFW(button, action);
}

void AntTweakbarInputController::cursorPosEvent(GLFWwindow* wnd, double x, double y)
{
  TwEventMousePosGLFW(static_cast<int>(x), static_cast<int>(y));
}

void AntTweakbarInputController::keyEvent(GLFWwindow* wnd, int key, int scancode, int action, int mods)
{
  TwEventKeyGLFW(key, action);
}

void AntTweakbarInputController::scrollEvent(GLFWwindow* wnd, double x, double y)
{
  TwEventMouseWheelGLFW(static_cast<int>(y));
}
