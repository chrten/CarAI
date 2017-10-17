#pragma once

#include <vector>
#include <AntTweakBar.h>

// steps of a genetic algorithm
// - initialization
// - selection based on fitness
// - genetic operations: crossover, mutation
// - termination




class EvolutionProcess
{
public:
  EvolutionProcess(float chromosomeCrossRate, float chromosomeMutationRate, float geneMutationRate);
  virtual ~EvolutionProcess();


  void initTweakVars(TwBar* bar);

  struct Chromosome
  {
    Chromosome() {}
    virtual ~Chromosome() {}

    // Compute crossover of this and other chromosome.
    // prob is the probability of swapping a gene.
    virtual void crossover(const Chromosome* other, float prob, Chromosome* resultA, Chromosome* resultB) const = 0;

    // prob is the probability of mutating a gene.
    virtual void mutate(float prob) = 0;

    virtual float fitness() const = 0;
  };

  // apply genetic algorithm to compute a new population
  // newPopulation has to be allocated and different from population.
  void computeNewPopulation(const std::vector<Chromosome*>& population, std::vector<Chromosome*>& newPopulation);

  // get current generation id
  int generation() const { return m_generation; }

private:

  // select two chromosomes for crossover
  void selection(const std::vector<Chromosome*>& population, float totalFitness, Chromosome** a, Chromosome** b) const;

private:

  // generation counter
  int m_generation;

  // probability of swapping a gene on crossover operator
  float m_crossRate;

  // probability of mutating a chromosome
  float m_mutationRate;

  // probability of mutating a gene
  float m_mutationGeneRate;
};