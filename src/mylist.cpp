#include <iostream>
#include <stdlib.h>
#include "adstar/mylist.h"

using namespace std;

MyList::MyList() {
  first = NULL;
  last = NULL;
  currentsize = 0;
}

MyList::~MyList() {}

void MyList::insert(State *s) {
  element* e = new element();
  e->s = s;
  if(first==NULL) {
    e->prev = NULL;
    e->next = NULL;
    first = e;
    last = e;
  }
  else {
    e->prev = last;
    last->next = e;
    last = e;
    e->next = NULL;
  }
  currentsize++;
}

State* MyList::remove() {
  if(currentsize==0)
    return NULL;
  State* s = last->s;
  if(last==first) {
    last = NULL;
    first = NULL;
  }
  else {
    last = last->prev;
    last->next = NULL;
  }
  currentsize--;
  return s;
}

int MyList::size() {
  return currentsize;
}

void MyList::clear() {
  first = NULL;
  last = NULL;
  currentsize = 0;
}
