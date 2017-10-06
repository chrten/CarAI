
#include "DeepLearningCarApp.h"


int main(void)
{
  DeepLearningCarApp* app = new DeepLearningCarApp();

  app->init();

  app->exec();

  delete app;


  return 0;
}