/**

Author: Anirudh Vemula
Date: 18th Jan 2016

My implementation of a heap for faster execution of the anytime D* algorithm.

Note: The implementation is heavily inspired from the publicly-available SBPL implementation of a heap. Due credits given.

**/

#ifndef ADSTAR_MYHEAP_
#define ADSTAR_MYHEAP_

#include <iostream>
#include <stdlib.h>
#include "adstar/state.h"

using namespace std;

bool key_less (State *s1, State *s2);

bool key_greater (State *s1, State *s2);

class MyHeap
{
 public:
  State **heap;
  int currentsize;
  int allocated_size;


  MyHeap();
  ~MyHeap();

  int size();
  void makeemptyheap();
  void insertheap(State *s);
  void eraseheap(State *s);
  State *getminheap();
  State *deleteminheap();
  void makeheap();

 private:
  
  void percolatedown(int hole, State *s);
  void percolateup(int hole, State *s);
  void percolateupordown(int hole, State *s);

  //void growheap();
  //void sizecheck();
};

#endif
