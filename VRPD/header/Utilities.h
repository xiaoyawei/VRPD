//
//  Utilities.h
//  VRPD
//
//  Created by Xiaoya Wei on 4/26/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#ifndef __VRPD__Coordinate__
#define __VRPD__Coordinate__

#include <stdio.h>

class Coordinate{
public:
    Coordinate();
    Coordinate(double xCoor, double yCoor);
    void set(double xCoor, double yCoor);
    void setX(double xCoor);
    void setY(double yCoor);
    double getX();
    double getY();
private:
    double x, y;
};

class DroneDeployment{
public:
    int start, end, routeID, nodeID;
    double addedTime;
    DroneDeployment(int s, int e, int r, int n, double a);
    DroneDeployment();
    void set(int s, int e, double a);
    bool operator==(const DroneDeployment& d2) const;
    bool operator<(const DroneDeployment& d2) const;
    DroneDeployment& operator=(const DroneDeployment& rhs);
    void reset();

private:
    const double beta = 0;
};

class InitialNodeAssignmentHelper{
public:
    double length;
    int nodeID;
};

int initialNodeHelperCmp(const void *a, const void *b);

class DroneAssignmentHelper{
public:
    int degree, nodeID;
    double distance;
};

int droneAssignHelperCmp(const void *a, const void *b);

#endif /* defined(__VRPD__Coordinate__) */
