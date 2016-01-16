#include <iostream>
#include <stdlib.h>
#include "adstar/adstar.h"
#include <fstream>

using namespace std;


int main() {

  /** Read parameters from file */
  
  ifstream file;
  file.open("/home/anirudh/catkin_ws/data/array.txt", ifstream::in);
  string line;

  if(!file.is_open()) {
    cout<<"File cannot be opened"<<endl;
    return 0;
  }

  getline(file, line); // dimensions
  int xlen = atoi (line.c_str());
  getline(file, line);
  int ylen = atoi (line.c_str());
  getline(file, line);
  int zlen = atoi (line.c_str());

  int xs,ys,zs,xg,yg,zg;
  file>>xs>>ys>>zs>>xg>>yg>>zg;

  cout<<xlen<<" "<<ylen<<" "<<zlen<<" "<<xs<<" "<<ys<<" "<<zs<<" "<<xg<<" "<<yg<<" "<<zg<<endl;

  int epsilon_start = 10;
  cout<<"Parameters read"<<endl;

  /** Setting up the planner */
  
  //ADstar planner(xlen, ylen, zlen, xs, ys, zs, xg, yg, zg, epsilon_start);
  ADstar planner(10, 10, 10, 1, 1, 1, 9, 9, 9, 10);
  planner.setSeed();
  planner.setCosts(1,2);
  //planner.readCosts(file);
  file.close();


  
  
  /** Planning */
  ofstream ofile;
  ofile.open("/home/anirudh/catkin_ws/data/path_adstar.txt", std::ofstream::out);
  planner.plan(false, ofile);
  ofile.close();
  cout<<endl;

  /* Replanning */
  /*std::cout<<"Costs changed"<<endl;
  planner.changeCosts(0.1);
  planner.replan(false);*/
  
  return 0;
}
