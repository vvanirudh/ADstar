/**
A header file for the 3D environment class

Needs to provide interfaces for creating an environment and accessing it
Also must store all the required values for each state in the environment

Environment is in the form of a 3D grid.

**/
#ifndef ADSTAR_ENVIRONMENT_
#define ADSTAR_ENVIRONMENT_


#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string>
#include "adstar/state.h"

using namespace std;

class State;

class Environment {

 public:
  size_t xlen, ylen, zlen; // dimensions of x,y,z
  vector<State> cData; // the vector of all the grid cells. Index calculated when needed
  //State* cData;
  int clow, chigh;

  /** Constructor */
  Environment(size_t x, size_t y, size_t z);

  /** Accessing operator */
  State& operator()(size_t x, size_t y, size_t z);

  /** Fill the grid with random cost values */
  void randInitialize(int clo, int chigh);

  /** Read costs from a file and fill the grid */
  void readCosts(ifstream& file);

  /** Resets flags of all states */
  void resetAll();

  /** Sets the seed randomly for random number generator */
  void setSeed();

  /** Changes costs of a given percentage of states */
  vector<State*> changeCosts(double fraction);
};
#endif
