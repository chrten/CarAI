
#include "Evolution.h"

#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <iostream>

EvolutionProcess::EvolutionProcess(float chromosomeCrossRate, float chromosomeMutationRate, float geneMutationRate)
  : m_generation(0),
  m_crossRate(chromosomeCrossRate), m_mutationRate(chromosomeMutationRate), m_mutationGeneRate(geneMutationRate)
{

}

EvolutionProcess::~EvolutionProcess()
{
}

void EvolutionProcess::initTweakVars(TwBar* bar)
{
  TwAddVarRO(bar, "Generation", TW_TYPE_INT32, &m_generation, "group=Evolution");
  TwAddVarRW(bar, "CrossRate", TW_TYPE_FLOAT, &m_crossRate, "min=0 max=1 step=0.01 group=Evolution");
  TwAddVarRW(bar, "ChromMutRate", TW_TYPE_FLOAT, &m_mutationRate, "min=0 max=1 step=0.01 group=Evolution");
  TwAddVarRW(bar, "GeneMutRate", TW_TYPE_FLOAT, &m_mutationGeneRate, "min=0 max=1 step=0.01 group=Evolution");
}

void EvolutionProcess::computeNewPopulation(const std::vector<Chromosome*>& population, std::vector<Chromosome*>& newPopulation)
{
  size_t n = population.size();

  if (n != newPopulation.size())
  {
    std::cerr << "error: in and out population size must be equal" << std::endl;
    return;
  }

  // compute total fitness of population
  float totalFitness = 0.0f;
  for (size_t i = 0; i < n; ++i)
    totalFitness += population[i]->fitness();

  // genetic algorithm
  for (size_t i = 0; i<n/2+1; ++i)
  {
    // select two chromosomes for crossover
    Chromosome* a = 0;
    Chromosome* b = 0;
    selection(population, totalFitness, &a, &b);

    // apply genetic operations
    size_t indexA = i*2, indexB = i*2+1;
    if (indexA >= newPopulation.size()) 
      break;
    a->crossover(b, m_crossRate, newPopulation[indexA], indexB >= newPopulation.size() ? 0 : newPopulation[indexB]);

    if (static_cast<float>(std::rand()) / RAND_MAX < m_mutationRate)
      a->mutate(m_mutationGeneRate);
  }

  ++m_generation;
}

void EvolutionProcess::selection(const std::vector<Chromosome*>& population, float totalFitness, Chromosome** a, Chromosome** b) const
{
  int found = 0;
  size_t n = population.size();
  int maxTries = 10;

  for (int t = 0; t < maxTries; ++t)
  {
    // use fitness as probability distribution
    for (size_t i = 0; i < n; ++i)
    {
      Chromosome* c = population[i];

      // take from uniform[0,1]
      float u = static_cast<float>(std::rand()) / RAND_MAX;
      float r = c->fitness() / totalFitness;

      if (u < r)
      {
        if (found++ < 1)
          *a = c;
        else
        {
          *b = c;
          return;
        }
      }
    }
  }
}

