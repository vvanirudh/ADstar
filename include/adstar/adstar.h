/**
A header file for the AD* planner

Needs to have interfaces for all the important functions in the planner
Use environment and state classes

**/

#include <iostream>
#include <vector>
#include <stdlib.h>
#include "adstar/state.h"
#include "adstar/environment.h"
#include <set>

using namespace std;

class State;
class Environment;

class ADstar {
 public:
  class stateCompare
  {
  public:
    bool operator() (State *s1, State *s2) {
      vector<int> k1 = s1->k;
      vector<int> k2 = s2->k;
      
      if(k1[0] < k2[0]) {
	return true;
      }
      else if (k2[0] < k1[0]) {
	return false;
      }
      else { // k1[0] == k2[0]
	if(k1[1] < k2[1]) {
	  return true;
	}
	else {
	  return false;
	}
      }
    }
  };
 
  int xstart, ystart, zstart; // start state
  int xgoal, ygoal, zgoal; // goal state

  State *start, *goal;

  Environment env3D;
  
  double epsilon, epsilon_start;
  
  //priority_queue<State, vector<State>, stateCompare> open, closed, incons;
  set<State*, stateCompare> open, closed, incons;

  /** Constructor */
  ADstar(size_t xlen, size_t ylen, size_t zlen, int xs, int ys, int zs, int xg, int yg, int zg, double eps);

  /** Set low cost and high cost values */
  void setCosts (int clo, int chigh);
  
  /** Key function */
  vector<int> key(State *s);

  /** Heuristic function */
  int heuristic(State *s1, State *s2);

  /** Update state function */
  void updateState(State *s);

  /** ComputeOrImprovePath function */
  void computeOrImprovePath();

  /** Main solve function */
  void solve(bool print);

  /** Min one-step lookahead search */
  int minSucc(State *s);

  /** Motion Cost to travel to s2 from s1 */
  int motionCost(State *s1, State *s2);

  /** Update all predecessor states */
  void updateAllPredStates(State *s);

  /** Prints the path */
  void printPath(State *g);

};
