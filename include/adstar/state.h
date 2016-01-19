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
#include <math.h>
#include <set>
//#include "adstar/key.h"

using namespace std;

//#define MAXVALUE INFINITY
#define MAXVALUE std::numeric_limits<double>::infinity()


class State {
 public:
  int cost;
  double gval;
  double hval;
  double rhsval;
  int x;
  int y;
  int z;
  bool visited, open, closed, inSolution, incons;
  //keys k;
  double k1, k2;
  State *succ, *succb;

  int heapindex;
  
  //std::set<State*>::iterator it_open, it_closed;
  /** Empty constructor */
  State() {
    heapindex = 0;
    cost = 0;
    gval = MAXVALUE;
    hval = 0;
    rhsval = MAXVALUE;
    visited = false;
    open = closed = incons = false;
    inSolution = false;
    k1 = k2 = MAXVALUE;
  }

  /** Constructor with cost value */
  State(int c) {
    heapindex = 0;
    cost = c;
    gval = MAXVALUE;
    hval = 0;
    rhsval = MAXVALUE;
    visited = false;
    open = closed = incons = false;
    inSolution = false;
    k1 = k2 = MAXVALUE;
  }

  void printState() {
    std::cout<<"("<<x<<","<<y<<","<<z<<")";
  }
  

  /*bool operator==(const State& other) {
    return (*this->x == other.x) && (*this->y == other.y) && (*this->z == other.z);
  }

  bool operator!=(const State& other) {
    return !(*this==other);
    }*/

};

#endif
