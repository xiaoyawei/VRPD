//
//  Coordinate.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 4/26/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include "Coordinate.h"
Coordinate::Coordinate(){
    x = y = 0;
}

Coordinate::Coordinate(double xCoor, double yCoor): x(xCoor), y(yCoor){}

void Coordinate::set(double xCoor, double yCoor){
    x = xCoor;
    y = yCoor;
}

void Coordinate::setX(double xCoor){
    x = xCoor;
}

void Coordinate::setY(double yCoor){
    y = yCoor;
}

double Coordinate::getX(){
    return x;
}

double Coordinate::getY(){
    return y;
}

DroneDeployment::DroneDeployment(int s, int e, int r, int n, double a)
    :start(s), end(e), routeID(r), nodeID(n), addedTime(a){}

DroneDeployment::DroneDeployment(){}

void DroneDeployment::set(int s, int e, double a){
    start = s;
    end = e;
    addedTime = a;
}