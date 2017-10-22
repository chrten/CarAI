
#include "NeuralNetwork.h"

#include <cstdlib>
#include <cmath>
#include <random>
#include <functional>
#include <algorithm>
#include <iostream>

NeuralNetwork::NeuralNetwork()
  : m_numInputs(0), m_numOutputs(0)
{

}

NeuralNetwork::~NeuralNetwork()
{
  std::for_each(m_links.begin(), m_links.end(), [](auto x) {delete x; });
}

NeuralNetwork::LayerLinks* NeuralNetwork::addLayer(int numNeurons)
{
  LayerLinks* r = 0;
  if (!m_numInputs)
    m_numInputs = numNeurons;
  else
  {
    int prevCount = m_numOutputs ? m_numOutputs : m_numInputs;

    r = new LayerLinks(prevCount, numNeurons);
    m_links.push_back(r);

    m_numOutputs = numNeurons;
  }
  return r;
}

bool NeuralNetwork::LayerLinks::compute(const std::vector<float>& input, std::vector<float>& output) const
{
  if (input.size() != static_cast<size_t>(numInputs))
  {
    std::cerr << "size mismatch between input array and neural layer" << std::endl;
    return false;
  }

  output.resize(numOutputs);

  for (int k = 0; k < numOutputs; ++k)
  {
    float p = 0.0f;

    for (int i = 0; i < numInputs; ++i)
      p += weight(i, k) * input[i];

    output[k] = activate(p);
  }

  return true;
}


// float rng(float _min, float _max)
// {
//   static std::default_random_engine re;
//   static std::uniform_real_distribution<float> dist;
// 
//   return dist(re, std::uniform_real_distribution<float>::param_type{ _min, _max });
// }


void NeuralNetwork::LayerLinks::randomize(float xmin, float xmax)
{
  float range = xmax - xmin;
  std::generate(weights.begin(), weights.end(), [xmin, range] {return xmin + range * static_cast<float>(std::rand()) / RAND_MAX; });

  //std::generate(weights.begin(), weights.end(), [xmin, xmax]() {return rng(xmin, xmax); });

//   auto r = std::bind(rng, xmin, xmax);
//   std::generate(weights.begin(), weights.end(), r);

//   for (int k = 0; k < numOutputs; ++k)
//   {
//     for (int i = 0; i < numInputs; ++i)
//     {
//       float u = static_cast<float>(std::rand()) / RAND_MAX;
//       weight(i, k) = xmin + (xmax - xmin) * u;
//     }
//   }
}

bool NeuralNetwork::compute(const std::vector<float>& input, std::vector<float>& output) const
{
  if (input.size() != static_cast<size_t>(m_numInputs))
  {
    std::cerr << "size mismatch between input array and neural layer" << std::endl;
    return false;
  }

  std::vector<float> temp;
  output = input;

  for (size_t i = 0; i < m_links.size(); ++i)
  {
    temp.swap(output);
    if (!m_links[i]->compute(temp, output))
      return false;
  }

  return true;
}