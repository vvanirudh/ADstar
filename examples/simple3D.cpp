#include <iostream>
#include <stdlib.h>
#include "adstar/adstar.h"

using namespace std;


int main() {

  ADstar planner(100, 100, 10, 1, 1, 1, 9, 9, 9, 10);
  
  planner.setCosts(500,20000);

  planner.solve(false);
  return 0;
}
