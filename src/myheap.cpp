/**

Author: Anirudh Vemula
Date: 18th Jan 2016


Implementation of a custom heap for efficiency purposes 

Note: This implementation is heavily inspired from the publicly-available SBPL implementation of a heap. Due credits given.

**/

#include "adstar/myheap.h"
#include <iostream>
#include <stdlib.h>

using namespace std;

MyHeap::MyHeap() {
  allocated_size = 5000000; // initial heap size
  currentsize = 0;

  heap = new State*[allocated_size];
}

MyHeap::~MyHeap() {
  for (int i=1; i<=currentsize; i++)
    heap[i]->heapindex = 0;

  delete[] heap;
}

void MyHeap::percolatedown(int hole, State *s) {
  int child;

  if (currentsize!=0) {
    for (; 2*hole <= currentsize; hole = child) {
      child = 2*hole;

      if (child!=currentsize && key_less(heap[child+1], heap[child])) ++child;

      if(key_less(heap[child], s)) {
	heap[hole] = heap[child];
	heap[hole]->heapindex = hole;
      }
      else
	break;
    }
    heap[hole] = s;
    heap[hole]->heapindex = hole;
  }
}

void MyHeap::percolateup(int hole, State *s) {
  if (currentsize!=0) {
    for (; hole > 1 && key_less(s, heap[hole/2]); hole/=2) {
      heap[hole] = heap[hole/2];
      heap[hole]->heapindex = hole;
    }
    heap[hole] = s;
    heap[hole]->heapindex = hole;
  }
}

void MyHeap::percolateupordown(int hole, State *s) {
  if(currentsize!=0) {
    if (hole>1 && key_greater(heap[hole/2], s))
      percolateup(hole, s);
    else
      percolatedown(hole,s);
  }
}

void MyHeap::makeemptyheap() {
  for (int i=1; i<=currentsize; i++)
    heap[i]->heapindex = 0;
  currentsize = 0;
}

void MyHeap::makeheap() {
  for (int i=currentsize/2; i>0; i--)
    percolatedown(i, heap[i]);
}


void MyHeap::insertheap(State *s) {
  if(s->heapindex!=0) {
    cout<<"State already in heap : "<<s->x<<" "<<s->y<<" "<<s->z<<endl;
    return;
  }
  percolateup(++currentsize, s);
}

void MyHeap::eraseheap(State *s) {
  if(s->heapindex==0) {
    cout<<"State not in heap"<<endl;
    return;
  }
  percolateupordown(s->heapindex, heap[currentsize--]);
  s->heapindex=0;
}

State* MyHeap::getminheap() {
  if(currentsize==0) {
    cout<<"Heap is empty"<<endl;
    return NULL;
  }

  return heap[1];
}

State* MyHeap::deleteminheap() {
  State* s;
  if(currentsize==0) {
    cout<<"Heap is empty. Attempted to delete min"<<endl;
    return NULL;
  }

  s = heap[1];
  s->heapindex = 0;
  percolatedown(1, heap[currentsize--]);
  return s;
}

int MyHeap::size() {
  return currentsize;
}

bool key_less (State *s1, State *s2) {
  if (s1->k1 < s2->k1)
    return true;
  else if (s1->k1 > s2->k1)
    return false;
  else { // s1->k1 == s2->k1
    if (s1->k2 < s2->k2)
      return true;
    else
      return false;
  }
}

bool key_greater (State *s1, State *s2) {
  if (s1->k1 > s2->k1)
    return true;
  else if (s1->k1 < s2->k1)
    return false;
  else { // s1->k1 == s2->k1
    if (s1->k2 > s2->k2)
      return true;
    else
      return false;
  }
}
