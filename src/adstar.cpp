#include <iostream>
#include <vector>
#include <stdlib.h>
#include "adstar/adstar.h"
#include <limits>
#include <cmath>

using namespace std;


ADstar::ADstar(size_t xlen, size_t ylen, size_t zlen, int xs, int ys, int zs, int xg, int yg, int zg, double eps): env3D(xlen, ylen, zlen) {
  xstart = xs; ystart = ys; zstart = zs;
  xgoal = xg; ygoal = yg; zgoal = zg;
  epsilon_start = eps;
  
  start = &env3D(xs, ys, zs);
  goal = &env3D(xg, yg, zg);
}

void ADstar::setCosts(int clo, int chigh) {
  env3D.randInitialize(clo, chigh);
}

vector<int> ADstar::key(State *s) {
  vector<int> res;
  if(s->gval > s->rhsval) {
    res.push_back(s->rhsval + epsilon*heuristic(start, s));
    res.push_back(s->rhsval);
  }
  else {
    res.push_back(s->gval + heuristic(start, s));
    res.push_back(s->gval);
  }
  return res;
}

int ADstar::heuristic(State *s1, State *s2) {
  //return 0;
  int x1 = s1->x; int y1 = s1->y; int z1 = s1->z;
  int x2 = s2->x; int y2 = s2->y; int z2 = s2->z;

  return sqrt(pow(x2-x1,2) + pow(y2-y1,2) + pow(z2-z1,2)); // Euclidean distance heuristic
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
      s->k = key(s);
      open.insert(s);
      s->open = true;
    }
    else { // s in closed
      s->k = key(s);
      incons.insert(s); // put it in incons
    }
  }
}

void ADstar::computeOrImprovePath() {
  // TODO key evaluations done as needed. Should it be the priority instead? Verify
  while ((key(*open.begin())[0] < key(start)[0]) || ((key(*open.begin())[0] == key(start)[0]) && (key(*open.begin())[1] < key(start)[1])) || (start->rhsval != start->gval)) {
    State *s = *open.begin();
    open.erase(open.begin());
    s->open = false;
    //std::cout<<"("<<s->x<<","<<s->y<<","<<s->z<<")"<<std::endl;

    if(s->gval > s->rhsval) {
      s->gval = s->rhsval;
      s->k = key(s);
      closed.insert(s);
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


int ADstar::minSucc(State *s) {
  // Successors are treated same as neighbors. TODO verify
  int x = s->x; int y = s->y; int z = s->z;
  int minVal = MAXVALUE;

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
	
	int lcost = motionCost(s, s1) + s1->gval;
	//if(s1==goal) cout<<"goal seen "<<lcost<<endl;
	
	if (minVal > lcost) {
	  minVal = lcost;
	  //std::cout<<"inside ~ "<<minVal<<endl;
	}
      }
    }
  }
  return minVal;
}

int ADstar::motionCost(State *s1, State *s2) {
  return s1->cost + s2->cost;
}

void ADstar::updateAllPredStates(State *s) {
  // Predecessors are treated same as neighbors. TODO verify
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
	updateState(s1);
      }
    }
  }
  //std::cout<<std::endl;
}

void ADstar::solve(bool print) {

  start->gval = start->rhsval = MAXVALUE;
  goal->gval = MAXVALUE;
  goal->rhsval = 0;
  epsilon = epsilon_start;

  open.clear(); closed.clear(); incons.clear();
  goal->k = key(goal);
  //std::cout<<goal->k[0]<<" , "<<goal->k[1]<<std::endl;
  goal->visited = true;
  goal->open = true;
  open.insert(goal);

  computeOrImprovePath();
  //std::cout<<open.size() <<" , "<<closed.size() <<" , "<<incons.size()<<std::endl;
  //std::cout<<start->gval<<" , "<<start->rhsval<< std::endl;
  //std::cout<<goal->gval<<" , "<<goal->rhsval<< std::endl;
  // A sub-optimal path already found
  std::cout<<"Epsilon value : "<<epsilon<<endl;
  if(print)
    printPath(goal); // TODO implement this
  std::cout<<std::endl;
  std::cout<<"Cost to go from start: "<<start->gval<<endl;

  while(epsilon>1) {
    epsilon--; // Decrease epsilon
    env3D.resetAll();
    // Move states from incons to open
    while(incons.size()!=0) {
      State *s = *incons.begin();
      s->k = key(s);
      s->open = true;
      open.insert(s);
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
      printPath(goal);
    std::cout<<std::endl;
    std::cout<<"Cost to go from start: "<<start->gval<<endl;
  }
}

void ADstar::printPath(State *g) {
  // print the current state
  g->inSolution = true;
  std::cout<<"("<<g->x<<","<<g->y<<","<<g->z<<")"<<" ";
  if(g==start)
    return;
  int x = g->x; int y = g->y; int z = g->z;
  int minVal = MAXVALUE;
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
    printPath(ms);
}
// TODO reset functions to set all flags for all states to default values
