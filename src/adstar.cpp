/**

Author: Anirudh Vemula
Date: 14th Jan 2016
*/

/** TODO
 * Improve efficiency of minSucc and updatePred methods by only considering successors and predecessors
 * Add replanning capabilities
 */


#include <iostream>
#include <vector>
#include <stdlib.h>
#include "adstar/adstar.h"
#include <limits>
#include <cmath>
#include <string>
#include <stdio.h>
#include <fstream>

using namespace std;

// For execution time measurements
#include <sys/time.h>
typedef unsigned long long timestamp_t;

static timestamp_t get_timestamp() {
  struct timeval now;
  gettimeofday (&now, NULL);
  return now.tv_usec + (timestamp_t)now.tv_sec * 1000000;
}
// End

ADstar::ADstar(size_t xlen, size_t ylen, size_t zlen, int xs, int ys, int zs, int xg, int yg, int zg, double eps): env3D(xlen, ylen, zlen) {
  xstart = xs; ystart = ys; zstart = zs;
  xgoal = xg; ygoal = yg; zgoal = zg;
  epsilon_start = eps;
  
  start = &env3D(xs, ys, zs);
  goal = &env3D(xg, yg, zg);
  changed = false;
}

void ADstar::setCosts(int clo, int chigh) {
  this->clo = clo;
  this->chi = chigh;
  env3D.randInitialize(clo, chigh);
}

void ADstar::key(State *s) {
  //vector<double> res;
  if(s->gval > s->rhsval) {
    //res.push_back(s->rhsval + epsilon*heuristic(start, s));
    s->k1 = s->rhsval + epsilon*heuristic(start, s);
    //res.push_back(s->rhsval);
    s->k2 = s->rhsval;
  }
  else {
    //res.push_back(s->gval + heuristic(start, s));
    s->k1 = s->gval + heuristic(start, s);
    //res.push_back(s->gval);
    s->k2 = s->gval;
  }
  //return res;
}

double ADstar::heuristic(State *s1, State *s2) {
  //return 0;
  int x1 = s1->x; int y1 = s1->y; int z1 = s1->z;
  int x2 = s2->x; int y2 = s2->y; int z2 = s2->z;

  //return sqrt(pow(x2-x1,2) + pow(y2-y1,2) + pow(z2-z1,2)); // Euclidean distance heuristic
  //return 0;
  double dx = abs(x2-x1); double dy = abs(y2-y1); double dz = abs(z2-z1);
  double numDiagonalSteps = min(dz, min(dx, dy));
  double manhattanDistance = dx + dy + dz;
  double numStraightSteps = manhattanDistance - 3*numDiagonalSteps;
  //return (numStraightSteps + numDiagonalSteps)*clo;
  //return (abs(x2-x1) + abs(y2-y1) + abs(z2-z1))*clo;
  //return 1;
  //return (numStraightSteps + numDiagonalSteps);
  double dx2 = dx*dx; double dy2 = dy*dy; double dz2 = dz*dz;
  return (clo*sqrt(dx2+dy2+dz2)) - dx - dy - dz;
}

void ADstar::updateState(State *s) {
  if(!s->visited) { // s is not visited
    s->gval = MAXVALUE;
    s->visited = true; // mark s as visited
  }

  if(s != goal) { // s is not the goal state
    s->rhsval = minSucc(s); // minimum one-step lookahead    
    //std::cout<<" rhs: "<<s->rhsval;
  }

  if(s->open) { // s in open
    open.erase(s); // remove from open
    s->open = false;
  }

  if(s->gval != s->rhsval) { // s is inconsistent
    if (!s->closed) { // s not in closed
      key(s);
      open.insert(s);
      s->open = true;
    }
    else { // s in closed
      //key(s);
      incons.insert(s); // put it in incons
      //incons.push_back(s);
      s->incons = true;
    }
  }
}

void ADstar::computeOrImprovePath() {
  // TODO key evaluations done as needed. Should it be the priority instead? Verify
  
  while (
	 ((*open.begin())->k1 < start->k1) ||
	 (((*open.begin())->k1 == start->k1) && ((*open.begin())->k2 < start->k2)) ||
	 (start->rhsval != start->gval)
	 )
    {
    
    State *s = *open.begin();
    /*if(!s->open) {
      open.erase(open.begin());
      continue;
      }*/
    open.erase(open.begin());
    s->open = false;
    // DEBUG
    //s->printState();
    //cout<<endl;
    //std::cout<<"("<<s->x<<","<<s->y<<","<<s->z<<")"<<std::endl;

    if(s->gval > s->rhsval) {
      s->succ = s->succb;
      s->gval = s->rhsval;
      //key(s);
      closed.insert(s);
      //closed.push_back(s);
      s->closed = true;
      updateAllPredStates(s);
    }
    else {
      s->gval = MAXVALUE;
      updateState(s);
      updateAllPredStates(s);
    }
  }
}


double ADstar::minSucc(State *s) {
  // Successors are treated same as neighbors. TODO verify
  int x = s->x; int y = s->y; int z = s->z;
  double minVal = MAXVALUE;

  for (int i=-1; i<=1; i++) {
    for (int j=-1; j<=1; j++) {
      for (int k=-1; k<=1; k++) {
	if (i==0 && j==0 && k==0) {
	  continue;
	}
	if(x+i >= env3D.xlen || y+j >= env3D.ylen || z+k >= env3D.zlen  || x+i < 0 || y+j < 0 || z+k < 0) {
	  continue;
	}
	State *s1 = &env3D(x+i, y+j, z+k);

	//Doubtful stuff
	if(s1->gval > s->gval) // Farther to goal than s. So must be predecessor
	  continue;
	
	double lcost = motionCost(s, s1) + s1->gval;
	//if(s1==goal) cout<<"goal seen "<<lcost<<endl;
	
	if (minVal > lcost) {
	  minVal = lcost;
	  s->succb = s1;
	  //std::cout<<"inside ~ "<<minVal<<endl;
	}
      }
    }
  }
  return minVal;
}

double ADstar::motionCost(State *s1, State *s2) {
  //return s1->cost + s2->cost;
  // TODO Need to include distance as well in the motion cost

  int x1 = s1->x; int y1 = s1->y; int z1 = s1->z;
  int x2 = s2->x; int y2 = s2->y; int z2 = s2->z;

  double dist = sqrt(pow(x2-x1,2) + pow(y2-y1,2) + pow(z2-z1,2)); // Euclidean distance
  return dist*(s1->cost + s2->cost); //STUPID CHANGE
  //return s1->cost + s2->cost;
}

void ADstar::updateAllPredStates(State *s) {
  // Predecessors are treated same as neighbors. TODO verify. Slow
  int x = s->x; int y = s->y; int z = s->z;

  for (int i=-1; i<=1; i++) {
    for (int j=-1; j<=1; j++) {
      for (int k=-1; k<=1; k++) {
	if (i==0 && j==0 && k==0) {
	  continue;
	}
	if(x+i >= env3D.xlen || y+j >= env3D.ylen || z+k >= env3D.zlen || x+i < 0 || y+j < 0 || z+k < 0)
	  continue;
	State *s1 = &env3D(x+i, y+j, z+k);
	
	//std::cout<<"("<<s1->x<<","<<s1->y<<","<<s1->z<<")";
	
	// Doubtful stuff
	if(s1->gval < s->gval) // Closer to goal than s. So must be successor state
	    continue;
	   	
	updateState(s1); // Can probably check if the state is consistent. If so, then ditch updating it
	//s1->printState();
	//cout<<s1->gval<<" "<<s1->rhsval<<" "<<s1->open<<" "<<s1->k1<<" "<<s1->k2<<endl;
      }
    }
  }
  //std::cout<<std::endl;
}

void ADstar::plan(bool print, ofstream& file) {
  cout<<"Plan started"<<endl;
  double secsTaken = 0;
  timestamp_t t0 = get_timestamp();
  start->gval = start->rhsval = MAXVALUE;
  goal->gval = MAXVALUE;
  goal->rhsval = 0;
  epsilon = epsilon_start;

  open.clear(); closed.clear(); incons.clear();
  key(goal);
  //std::cout<<goal->k[0]<<" , "<<goal->k[1]<<std::endl;
  goal->visited = true;
  goal->open = true;
  open.insert(goal);
  
  computeOrImprovePath();
  cout<<"open, closed, incons:";
  std::cout<<open.size() <<" , "<<closed.size() <<" , "<<incons.size()<<std::endl;
  //std::cout<<start->gval<<" , "<<start->rhsval<< std::endl;
  //std::cout<<goal->gval<<" , "<<goal->rhsval<< std::endl;
  // A sub-optimal path already found
  std::cout<<"Epsilon value : "<<epsilon<<endl;
  if(print)
    printPath(start, file);
  //printPathIneff(goal);
  
  std::cout<<"Cost to go from start: "<<start->gval<<endl;
  timestamp_t t1 = get_timestamp();
  secsTaken = (t1 - t0)/1000000.0L;
  std::cout<<"Time taken (in secs) : "<<secsTaken<<endl;
  std::cout<<std::endl;
  
  while(epsilon>1) {
    epsilon--; // Decrease epsilon
    env3D.resetAll();
    // Move all states from open to incons
    while(open.size()!=0) {
      State *s = *open.begin();
      s->open = false;
      incons.insert(s);
      //incons.push_back(s);
      s->incons = false;
      open.erase(open.begin());
    }
    
    // Move states from incons to open with updated keys
    while(incons.size()!=0) {
      State *s = *incons.begin();
      key(s);
      s->open = true;
      open.insert(s);
      s->incons = false;
      incons.erase(incons.begin());
      //incons.pop_back();
    }

    while(closed.size()!=0) {
      State *s = *closed.begin();
      s->closed = false;
      closed.erase(closed.begin());
      //closed.pop_back();
    }
    
    cout<<"Start"<<endl;
    computeOrImprovePath();
    // Sub-optimal (or possibly optimal) path found again
    std::cout<<"Epsilon value : "<<epsilon<<endl;
    if(print)
      printPath(start, file);
    //printPathIneff(goal);
    std::cout<<"Cost to go from start: "<<start->gval<<endl;
    t1 = get_timestamp();
    secsTaken = (t1 - t0)/1000000.0L;
    std::cout<<"Time taken (in secs) : "<<secsTaken<<endl;
    std::cout<<std::endl;
    //int a;
    //cin>>a;
    }
}

void ADstar::printPath(State *g, ofstream& file) {
  std::cout<<"("<<g->x<<","<<g->y<<","<<g->z<<")"<<" "<<g->gval<<";";
  //file<<g->x<<" "<<g->y<<" "<<g->z<<std::endl;
  if(g==goal)
    return;
  State *ms = g->succ;
  if(ms!=NULL)
    printPath(ms, file);
}

// TODO Incorrectly working. Not needed for now.
void ADstar::printPathIneff(State *g) {
  g->inSolution = true;
  std::cout<<"("<<g->x<<","<<g->y<<","<<g->z<<")"<<" "<<g->gval<<";";
  if(g==start)
    return;
  int x = g->x; int y = g->y; int z = g->z;
  double minVal = MAXVALUE;
  State *ms = NULL;
  for(int i=-1; i<=1; i++) {
    for(int j=-1; j<=1; j++) {
      for(int k=-1; k<=1; k++) {
	if(i==0 && j==0 && k==0)
	  continue;
	if(x+i >= env3D.xlen || y+j >= env3D.ylen || z+k >= env3D.zlen  || x+i < 0 || y+j < 0 || z+k < 0)
	  continue;
	State *s = &env3D(x+i, y+j,z+k);
	if(minVal > s->gval && !s->inSolution) {
	  minVal = s->gval;
	  ms = s;
	}
      }
    }
  }
  if(ms!=NULL)
    printPathIneff(ms);
}

void ADstar::setSeed() {
  env3D.setSeed();
}

void ADstar::changeCosts(double fraction) {
  changedStates = env3D.changeCosts(fraction);
  changed = true;
}

void ADstar::replan(bool print, ofstream& file) {
  double secsTaken = 0;
  timestamp_t t0 = get_timestamp();
  std::cout<<"Number of changed states : "<<changedStates.size()<<endl;
  for (int i=0; i<changedStates.size(); i++) {
    updateState(changedStates[i]);
  }

  //while(epsilon>=1) {
  env3D.resetAll();
  // Move all states from open to incons
  while(open.size()!=0) {
    State *s = *open.begin();
    s->open = false;
    incons.insert(s);
    //incons.push_back(s);
    s->incons = false;
    open.erase(open.begin());
  }
  // Move states from incons to open
  while(incons.size()!=0) {
    State *s = *incons.begin();
    key(s);
    s->open = true;
    open.insert(s);
    s->incons = false;
    incons.erase(incons.begin());
    
  }
  
  while(closed.size()!=0) {
    State *s = *closed.begin();
    s->closed = false;
    closed.erase(closed.begin());
  }
  
  computeOrImprovePath();
  // Sub-optimal (or possibly optimal) path found again
  std::cout<<"Epsilon value : "<<epsilon<<endl;
  if(print)
    printPath(start, file);
  //printPathIneff(goal);
  std::cout<<std::endl;
  std::cout<<"Cost to go from start: "<<start->gval<<endl;
  timestamp_t t1 = get_timestamp();
  secsTaken = (t1 - t0)/1000000.0L;
  std::cout<<"Time taken (in secs) : "<<secsTaken<<endl;
  //epsilon--;
  //int a;
  //cin>>a;
  //}
  
  changed = false;
  changedStates.clear();
}

void ADstar::readCosts(ifstream& file) {
  clo = 255;
  chi = 65535;
  env3D.readCosts(file);
}
