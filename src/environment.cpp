#include "adstar/environment.h"
#include "adstar/state.h"
#include <stdlib.h>
#include <iostream>

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
}

State& Environment::operator()(size_t x, size_t y, size_t z) {
  return cData.at(x+y*xlen+z*xlen*ylen);
}

void Environment::randInitialize(int clo, int chigh) {
  for (int i=0; i<xlen; i++) {
    for (int j=0; j<ylen; j++) {
      for (int k=0; k<zlen; k++) {
	cData[i+j*xlen+k*xlen*ylen].cost = rand()%(chigh-clo+1) + clo;
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
	cData[i+j*xlen+k*xlen*ylen].inSolution = false;
      }
    }
  }
}
