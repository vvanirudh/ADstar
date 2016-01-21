#include "adstar/environment.h"
//#include "adstar/state.h"
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <fstream>

using namespace std;

Environment::Environment(size_t x, size_t y, size_t z) :
  xlen(x), ylen(y), zlen(z) {
  cData.resize(x*y*z);
  for (int i=0; i<x; i++) {
    for (int j=0; j<y; j++) {
      for (int k=0; k<z; k++) {
	cData[i+x*j+x*y*k].x = i;
	cData[i+x*j+x*y*k].y = j;
	cData[i+x*j+x*y*k].z = k;
      }
    }
  }
  clow = 1;
  chigh = 10;
}

State& Environment::operator()(size_t x, size_t y, size_t z) {
  return cData.at(x+y*xlen+z*xlen*ylen);
}

void Environment::randInitialize(int clo, int chi) {
  clow = clo;
  chigh = chi;
  for (int i=0; i<xlen; i++) {
    for (int j=0; j<ylen; j++) {
      for (int k=0; k<zlen; k++) {
	// STUPID CHANGE
	//if(i==j)
	  //cData[i+j*xlen+k*xlen*ylen].cost = chi;
	//else
	  cData[i+j*xlen+k*xlen*ylen].cost = rand()%(chi-clo+1) + clo;
	  //cData[i+j*xlen+k*xlen*ylen].cost = clo;
      }
    }
  }
}

void Environment::resetAll() {
  for (int i=0; i<xlen; i++) {
    for (int j=0; j<ylen; j++) {
      for (int k=0; k<zlen; k++) {
	cData[i+j*xlen+k*xlen*ylen].open = false;
	//cData[i+j*xlen+k*xlen*ylen]. = false;
	cData[i+j*xlen+k*xlen*ylen].closed = false;
	cData[i+j*xlen+k*xlen*ylen].incons = false;
	cData[i+j*xlen+k*xlen*ylen].inSolution = false;
	//cData[i+j*xlen+k*xlen*ylen].succ = NULL;
      }
    }
  }
}

void Environment::setSeed() {
  srand(time(NULL));
}

vector<State*> Environment::changeCosts(double fraction) {
  vector<State*> changedStates;
  for (int i=0; i<xlen; i++) {
    for (int j=0; j<ylen; j++) {
      for (int k=0; k<zlen; k++) {
	if (((double)rand()/RAND_MAX) < fraction) {
	  cData[i+j*xlen+k*xlen*ylen].cost = rand()%(chigh-clow+1) + clow;
	  changedStates.push_back(&cData[i+j*xlen+k*xlen*ylen]);
	}
      }
    }
  }
  return changedStates;
}

void Environment::readCosts(ifstream& file) {
  cout<<"Costs read start"<<endl;
  for (int i=0; i<xlen; i++) {
    for (int j=0; j<ylen; j++) {
      for (int k=0; k<zlen; k++) {
	file>>cData[i+j*xlen+k*xlen*ylen].cost;
      }
    }
  }
  cout<<"Costs read end"<<endl;
}
