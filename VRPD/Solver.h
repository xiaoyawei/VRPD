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

private:
    int newIndex(int index) const;
    int originalIndex(int index) const;
    
    
    int numFleet, numDrone, truckCap, depotIndex, numCustomer;
    double alpha, tDrone, **dist, **cost;
    const double INF = 1e30;
    bool *droned;
    Coordinate *nodes;
    VRPD *vrpd;
};

#endif /* defined(__VRPD__Solver__) */
