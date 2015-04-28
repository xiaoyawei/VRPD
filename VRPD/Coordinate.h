//
//  Coordinate.h
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
};
#endif /* defined(__VRPD__Coordinate__) */
