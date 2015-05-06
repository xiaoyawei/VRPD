//
//  Solver.h
//  VRPD
//
//  Created by Xiaoya Wei on 4/28/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#ifndef __VRPD__Solver__
#define __VRPD__Solver__

#include <stdio.h>
#include "VRPD.h"
#include "VRPH.h"

class Solver{

    friend class VRPD;
    friend class VRP;
    
public:
    bool readFile(const char *filename);
    void main(const char* filename);
    ~Solver();

private:
    int newIndex(int index) const;
    int originalIndex(int index) const;
    void setupHelper();
    void createNodeInfo();
    void trackBestSolution();
    double getSolution();
    
    int numFleet, numDrone, truckCap, depotIndex, numCustomer, **nextNode, **pathLength, *degree;
    double alpha, tDrone, **dist, **cost, *radius, bestObject = 1e30;
    const double INF = 1e30;
    bool *droned, *bestDroned;
    Coordinate *nodes;
    VRPD *vrpd;
    VRP *vrp;
};

#endif /* defined(__VRPD__Solver__) */
