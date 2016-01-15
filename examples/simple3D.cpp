#include <iostream>
#include <stdlib.h>
#include "adstar/adstar.h"

using namespace std;


int main() {

  ADstar planner(10, 10, 10, 1, 1, 1, 9, 9, 9, 10);
  planner.setCosts(0,2);

  planner.solve();
  return 0;
}
