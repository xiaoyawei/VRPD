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
#include "set"

class Solver{

    friend class VRPD;
    friend class VRP;
    
public:
    void main(const char* filename);
    void setVerbose();
    Solver();
    ~Solver();

private:
    bool readFile(const char *filename);
    int newIndex(int index) const;
    int originalIndex(int index) const;
    void setupHelper();
    void createNodeInfo();
    void trackBestSolution();
    double getSolution();
    void displayDebugInfo(int type = 1);
    void findDroneAssignment();
    DroneAssignmentHelper* createDroneSiteInfo();
    void flipDroneSite(int index);
    void outputTruckRoute(int truckID);
    void displayResult();
    void setParameters();
    
    int numFleet, numDrone, truckCap, depotIndex, numCustomer, **nextNode, **pathLength, *degree;
    double alpha, tDrone, **dist, **cost, *radius, bestObject = std::numeric_limits<double>::max();
    int numOfLoops, numOfItersPerLoop, numOfListSize;
    double startingTemperatur, coolRatio;
    bool debug1, debug2;
    int theHeuristics;
    const double INF = std::numeric_limits<double>::max();
    bool *droned, *bestDroned;
    int *bestNextArray, *bestPreArray;
    Coordinate *nodes;
    VRPD *vrpd;
    VRP *vrp;
};

#endif /* defined(__VRPD__Solver__) */
