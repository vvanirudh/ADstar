/**
A header file for the state class


Needs to provide interfaces for creating state and accessing it
Must store all the cost, cost-to-go and heuristic values associated with each state

It is part of the environment class (as the 3D grid cell)

**/

#ifndef ADSTAR_STATE_
#define ADSTAR_STATE_

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <limits>

using namespace std;

#define MAXVALUE 10000000

class State {
 public:
  int cost;
  int gval;
  int hval;
  int rhsval;
  int x;
  int y;
  int z;
  bool visited, open, closed, inSolution;
  vector<int> k;
  
  /** Empty constructor */
  State() {
    cost = 0;
    gval = MAXVALUE;
    hval = 0;
    rhsval = MAXVALUE;
    visited = false;
    open = closed = false;
    inSolution = false;
  }

  /** Constructor with cost value */
  State(int c) {
    cost = c;
    gval = MAXVALUE;
    hval = 0;
    rhsval = MAXVALUE;
    visited = false;
    open = closed = false;
    inSolution = false;
  }

  

  /*bool operator==(const State& other) {
    return (*this->x == other.x) && (*this->y == other.y) && (*this->z == other.z);
  }

  bool operator!=(const State& other) {
    return !(*this==other);
    }*/

};
#endif
