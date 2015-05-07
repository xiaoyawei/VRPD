//
//  Utilities.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 4/26/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include "Utilities.h"
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

bool DroneDeployment::operator==(const DroneDeployment& d2) const{
    return this->start == d2.start && this->end == d2.end && this->routeID == d2.routeID && this->nodeID == d2.nodeID;
}

bool DroneDeployment::operator<(const DroneDeployment& d2) const{
    if (addedTime <= 0 && d2.addedTime > 0){
        return true;
    }else if (addedTime > 0 && d2.addedTime <= 0){
        return false;
    }else if (addedTime <= 0 && d2.addedTime <= 0){
        if (end - start == d2.end - d2.start)
            return addedTime < d2.addedTime;
        else
            return end - start < d2.end - d2.start;
    }else{
        return addedTime - beta * (end - start) < d2.addedTime - beta * (d2.end - d2.start);
    }
}

DroneDeployment& DroneDeployment::operator=(const DroneDeployment& rhs){
    start = rhs.start;
    end = rhs.end;
    routeID = rhs.routeID;
    nodeID = rhs.nodeID;
    addedTime = rhs.addedTime;
    return *this;
}

void DroneDeployment::reset(){
    start = end = routeID = nodeID = -1;
    addedTime = 1e30;
}

int initialNodeHelperCmp(const void *a, const void *b){
    const InitialNodeAssignmentHelper *s1, *s2;
    s1 = reinterpret_cast<const InitialNodeAssignmentHelper*>(a);
    s2 = reinterpret_cast<const InitialNodeAssignmentHelper*>(b);
    return s1->length - s2->length;
}

int droneAssignHelperCmp(const void *a, const void *b){
    const DroneAssignmentHelper *s1, *s2;
    s1 = reinterpret_cast<const DroneAssignmentHelper*>(a);
    s2 = reinterpret_cast<const DroneAssignmentHelper*>(b);
    int distCmp = s1->distance > s2->distance ? -1 : 1;
    return s1->degree == s2->degree ? distCmp : s1->degree - s2->degree;
}