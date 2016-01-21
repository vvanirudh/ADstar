/**

Author: Anirudh Vemula
Date: 18th Jan 2016

My implementation of a list for faster execution of the anytime D* algorithm.

Note: The implementation is heavily inspired from the publicly-available SBPL implementation of a heap. Due credits given.

**/

#ifndef ADSTAR_MYLIST_
#define ADSTAR_MYLIST_


#include <iostream>
#include <stdlib.h>
#include "adstar/state.h"

using namespace std;


struct element {
  
  element() {
    //s = new State();
  }
    
  State* s;
  struct element *prev;
  struct element *next;
};

typedef struct element element;

class MyList {
 public:
  element* first;
  element* last;
  int currentsize;

  MyList();

  ~MyList();

  void insert(State *s);

  State* remove();

  void clear();

  int size();
};

#endif
