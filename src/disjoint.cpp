/**
* @file disjoint.cpp
* @brief Implement functions to segment image into regions
* @date Nov. 6th 2016
* @author Hang Liu
*/

#include "disjoint.h"

/* ************************************************************************* */
DisJoint::DisJoint(int elements){
	elts = new DisjElem[elements];
	num = elements;

	for (int i = 0; i < elements; i++) {
		elts[i].rank = 0;
		elts[i].size = 1;
		elts[i].p = i;
	}
}

/* ************************************************************************* */
DisJoint::~DisJoint() {
	delete[] elts;
}

/* ************************************************************************* */
int DisJoint::find(int x){
	int y = x;
	while (y != elts[y].p){
		y = elts[y].p;
	}
	elts[x].p = y;

	return y;
}

/* ************************************************************************* */
void DisJoint::join(int x, int y){
	if (elts[x].rank > elts[y].rank){
		elts[y].p = x;
		elts[x].size += elts[y].size;
	}
	else{
		elts[x].p = y;
		elts[y].size += elts[x].size;
		if (elts[x].rank == elts[y].rank){
			elts[y].rank++;
		}
	}

	num--;
}