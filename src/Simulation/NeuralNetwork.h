#pragma once

#include <vector>

class NeuralNetwork
{
public:
  NeuralNetwork();
  virtual ~NeuralNetwork();

  struct LayerLinks 
  {
    LayerLinks(int n, int m) : numInputs(n), numOutputs(m) { weights.resize(n*m, 0.0f); }

    int numInputs;
    int numOutputs;
    std::vector<float> weights; // weight from i to j = w[j * numInputs + i]

    bool compute(const std::vector<float>& input, std::vector<float>& output) const;

    float weight(int i, int j) const { return weights[j * numInputs + i]; }
    float& weight(int i, int j) { return weights[j * numInputs + i]; }
    float activate(float x) const { return std::tanh(x); }

    void randomize(float xmin = 0.0f, float xmax = 1.0f); // randomize weights
  };

  // returns address to links from previous to new layer (if more than one layer available)
  LayerLinks* addLayer(int numNeurons);

  bool compute(const std::vector<float>& input, std::vector<float>& output) const;

  LayerLinks* links(int i) { return m_links[i]; }
  const LayerLinks* links(int i) const { return m_links[i]; }

private:

  int m_numInputs;
  int m_numOutputs;

  std::vector<LayerLinks*> m_links;

};