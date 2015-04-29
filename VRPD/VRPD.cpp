//
//  VRPD.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 4/26/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include "VRPD.h"

int VRPD::getNumByTruck(){
    return numByTruck;
}

void VRPD::getDroneAssignment(bool *isDroned){
    droned = isDroned;
    numByDrone = 0;
    for (int i = 0; i < numCustomer; ++i) {
        if (droned[i]) {
            ++numByDrone;
        }
    }
    numByTruck = numCustomer - numByDrone;
}

VRPD::VRPD(const Solver& solver){
    alpha = solver.alpha;
    cost = solver.cost;
    depotIndex = solver.depotIndex;
    dist = solver.dist;
    droned = solver.droned;
    numCustomer = solver.numCustomer;
    numDrone = solver.numDrone;
    numFleet = solver.numFleet;
    tDrone = solver.tDrone;
    truckCap = solver.truckCap;
    getDroneAssignment(solver.droned);
}

VRPD::~VRPD(){
    reset();
    delete [] cost[0];
    delete [] cost;
    delete [] dist[0];
    delete [] dist;
    delete [] nodes;
    delete [] droned;
}

//int VRPD::newIndex(int index) const{
//    if (index == depotIndex) {
//        return 0;
//    } else {
//        return index > depotIndex ? index - 1: index;
//    }
//}
//
//int VRPD::originalIndex(int index) const{
//    if (index == 0) {
//        return depotIndex;
//    } else {
//        return index > depotIndex ? index + 1: index;
//    }
//}
//
//bool VRPD::readFile(const char *filename){
//    int m;
//    const int buffSize = 256;
//    char line[buffSize];
//    std::ifstream in(filename);
//    //skip the words
//    for (int i = 0; i < 10; ++i) in.getline(line, buffSize);
//    
//    in.getline(line, buffSize);
//    sscanf(line, "Fleet Size (k):%d\n", &numFleet);
//    in.getline(line, buffSize);
//    in.getline(line, buffSize);
//    sscanf(line, "Alpha:%lf\n", &alpha);
//    in.getline(line, buffSize);
//    sscanf(line, "Drones Per Vehicle (l):%d\n", &numDrone);
//    in.getline(line, buffSize);
//    sscanf(line, "Drone Flight Duration (t_drone):%lf\n", &tDrone);
//    in.getline(line, buffSize);
//    sscanf(line, "Truck Capacity (cap_truck):%d\n", &truckCap);
//    in.getline(line, buffSize);
//    sscanf(line, "Depot ID(s) (d):%d\n", &depotIndex);
//    in.getline(line, buffSize);
//    sscanf(line, "N:%d\n", &numCustomer);
//    in.getline(line, buffSize);
//    sscanf(line, "M:%d\n", &m);
//    droned = new bool[numCustomer];
//    for (int i = 0; i < numCustomer; ++i) {
//        droned[i] = false;
//    }
//    for (int i = 0; i < numCustomer; ++i) {
//        waitingTime[i] = 0;
//    }
//    droneDplyAtNode = new DroneDeployment[numCustomer];
//    
//    dist = new double*[numCustomer];
//    dist[0] = new double[numCustomer * numCustomer];
//    for (int i = 1; i < numCustomer; ++i)
//        dist[i] = dist[i - 1] + numCustomer;
//    cost = new double*[numCustomer];
//    cost[0] = new double[numCustomer * numCustomer];
//    for (int i = 1; i < numCustomer; ++i)
//        cost[i] = cost[i - 1] + numCustomer;
//    for (int i = 0; i < numCustomer; ++i) {
//        for (int j = 0; j < numCustomer; ++j) {
//            dist[i][j] = cost[i][j] = INF;
//        }
//    }
//    for (int i = 0; i < numCustomer; ++i) {
//        dist[i][i] = cost[i][i] = 0;
//    }
//    for (int i = 0; i < 4; ++i) in.getline(line, buffSize);
//    for (int i = 0; i < m; ++i){
//        int a, b;
//        double c;
//        in.getline(line, buffSize);
//        sscanf(line, "%d,%d,%lf,false\n", &a, &b, &c);
//        a = newIndex(a);
//        b = newIndex(b);
//        dist[a][b] = c;
//        dist[b][a] = c;
//        cost[a][b] = c;
//        cost[b][a] = c;
//    }
//    for (int i = 0; i < numCustomer; ++i) {
//        for (int j = 0; j < numCustomer; ++j) {
//            for (int k = 0; k < numCustomer; ++k) {
//                if (cost[i][j] > cost[i][k] + cost[k][j]) {
//                    cost[i][j] = cost[i][k] + cost[k][j];
//                }
//            }
//        }
//    }
//    
//    nodes = new Coordinate[numCustomer];
//    for (int i = 0; i < 4; ++i) in.getline(line, buffSize);
//    for (int i = 0; i < numCustomer; ++i) {
//        double x, y;
//        in.getline(line, buffSize);
//        sscanf(line, "%lf,%lf\n", &x, &y);
//        nodes[newIndex(i)].set(x, y);
//    }
//    for (int i = 1; i < numCustomer; ++i) {
//        nodes[i].set(nodes[i].getX() - nodes[0].getX(), nodes[i].getY() - nodes[0].getY());
//    }
//    nodes[0].set(0, 0);
//    
//    in.close();
////    for (int i = 0; i < numCustomer; ++i) {
////        for (int j = 0; j < numCustomer; ++j) {
////            std::cout << cost[i][j] << " ";
////        }
////        std::cout << std::endl;
////    }
//    return true;
//}
//
//
//bool VRPD::closerThan(const double x1, const double x2) const{
//    if (x1 <= 0 && x2 <= 0)
//        return x1 > x2;
//    else
//        return x1 < x2;
//}
//




//  get the "best" deployment schedule to attach drone destination
//  dest with the route with route index routeID
DroneDeployment VRPD::getDroneDeployment(const int dest, const int *route, const int *deployedDrones, const double *waitingTime, const int length, const int routeID) const{
    int time[length][length];
    
    //calculate the route time schedule
    DroneDeployment deply(-1, -1, routeID, dest, INF);
    for (int len = 0; len < length; ++len){
        for (int i = 0; i + len < length; ++i) {
            if (len == 0) {
                time[i][i + len] = waitingTime[i];
            } else {
                time[i][i + len] = time[i][i + len - 1] + dist[route[i + len - 1]][route[i + len]] + waitingTime[i];
            }
            bool outOfDrone = false;
            for (int j = i; j < i + len; ++j) {
                if (deployedDrones[j] >= numDrone){
                    outOfDrone = true;
                    break;
                }
            }
            if (outOfDrone) {
                continue;
            }
            double droneTime = alpha * (cost[route[i]][dest] + cost[dest][route[i + len]]);
            DroneDeployment tmp(i, i + len, routeID, dest, droneTime - time[i][i + len]);
            if (droneTime <= tDrone && tDrone >= time[i][i + len] - waitingTime[i + len] && tmp < deply){
                deply = tmp;
            }
        }
    }
    return deply;
    
}


void VRPD::sendDrone(int *route, int *deployedDrone, double *waitingTime, const DroneDeployment& deply){
    for (int i = deply.start; i < deply.end; ++i) {
        ++deployedDrone[i];
    }
    double extraTime = alpha * (cost[route[deply.start]][deply.nodeID] + cost[deply.nodeID][route[deply.end]]) - waitingTime[deply.start];
    for (int i = deply.start + 1; i <= deply.end; ++i) {
        extraTime -= (dist[route[i - 1]][route[i]] + waitingTime[i]);
    }
    if (extraTime > 0) {
        waitingTime[deply.end] += extraTime;
        timeOfRoute[deply.routeID] += extraTime;
    }
    droneDplyAtNode[deply.nodeID] = deply;
    droneDplyAtRoute[deply.routeID].push_back(deply);
}

void VRPD::withdrawDrone(int *route, int *deployedDrone, double *waitingTime, const DroneDeployment& deply){
    for (int i = 0; i < lengthOfRoute[deply.routeID]; ++i) {
        waitingTime[i] = 0;
        deployedDrone[i] = 0;
    }
    timeOfRoute[deply.routeID] = getBasicRouteTime(route, lengthOfRoute[deply.routeID]);
    droneDplyAtNode[deply.nodeID].reset();
    for (std::vector<DroneDeployment>::iterator it = droneDplyAtRoute[deply.routeID].begin(); it != droneDplyAtRoute[deply.routeID].end(); ++it) {
        if (*it == deply) {
            droneDplyAtRoute[deply.routeID].erase(it);
            --it;
        }
    }
    for (std::vector<DroneDeployment>::iterator it = droneDplyAtRoute[deply.routeID].begin(); it != droneDplyAtRoute[deply.routeID].end(); ++it) {
        sendDrone(route, deployedDrone, waitingTime, *it);
    }
}

void VRPD::reset(){
    if (numOfRoute > 0) {
        for (int i = 0; i < numOfRoute; ++i) {
            delete [] waitingTime[i];
            delete [] deployedDrone[i];
            delete [] truckRoutes[i];
        }
        delete [] timeOfRoute;
        delete [] lengthOfRoute;
        delete [] droneDplyAtRoute;
        numOfRoute = 0;
    }
}

void VRPD::createRouteInfo(){
    
    for (int i = 0; i < numByTruck; ++i) {
        if (nextArray[i] < 0) {
            ++numOfRoute;
        }
    }
    lengthOfRoute = new int[numOfRoute];
    waitingTime = new double*[numOfRoute];
    deployedDrone = new int*[numOfRoute];
    truckRoutes = new int*[numOfRoute];
    timeOfRoute = new double[numOfRoute];
    droneDplyAtRoute = new std::vector<DroneDeployment>[numOfRoute];
    int routeIndex = 0;
    for (int i = 0; nextArray[i] != 0;) {
        int len = 3, j;
        for (j = abs(nextArray[i]); nextArray[j] > 0; j = nextArray[j]) {
            ++len;
        }
        i = j;
        lengthOfRoute[routeIndex++] = len;
    }
    
    routeIndex = 0;
    for (int i = 0; i < numOfRoute; ++i) {
        timeOfRoute[i] = 0;
        waitingTime[i] = new double[lengthOfRoute[i]];
        deployedDrone[i] = new int[lengthOfRoute[i]];
        truckRoutes[i] = new int[lengthOfRoute[i]];
        waitingTime[i][0] = waitingTime[i][lengthOfRoute[i] - 1] = 0;
        deployedDrone[i][0] = deployedDrone[i][lengthOfRoute[i] - 1] = 0;
        truckRoutes[i][0] = truckRoutes[i][lengthOfRoute[i] - 1] = 0;
        for (int j = 1; j < lengthOfRoute[i] - 1; ++j) {
            waitingTime[i][j] = 0;
            deployedDrone[i][j] = 0;
            truckRoutes[i][j] = routeIndex;
            routeIndex = abs(nextArray[routeIndex]);
        }
        timeOfRoute[i] = getBasicRouteTime(truckRoutes[i], lengthOfRoute[i]);
    }
    
}

void VRPD::assignInitialDrones(int method){
    
    reset();
    createRouteInfo();
    if (method == 1) {
        for (int i = 1; i < numCustomer; ++i) {
            if (droned[i]) {
                DroneDeployment minDply(-1, -1, 0, i, INF);
                for (int j = 0; j < numOfRoute; ++j) {
                    DroneDeployment dply = getDroneDeployment(i, truckRoutes[j], deployedDrone[j], waitingTime[j], lengthOfRoute[j], j);
                    if (dply.addedTime + timeOfRoute[j] < minDply.addedTime + timeOfRoute[minDply.routeID]) {
                        minDply = dply;
                    }
                }
//                droneDplyAtNode[i] = tmp;
//                droneDplyAtRoute[minDply.routeID].push_back(minDply);
                sendDrone(truckRoutes[minDply.routeID], deployedDrone[minDply.routeID], waitingTime[minDply.routeID], minDply);
            }
        }
    } else {
        // TO-DO: other initial assignment mehtods
    }
    return;
}

double VRPD::getBasicRouteTime(const int *route, const int len) const{
    double time = 0;
    for (int i = 0; i < len - 1; ++i) {
        time += dist[route[i]][route[i + 1]];
    }
    return time;
}

double VRPD::deployAllDrones(const int *next, const int *pre){
    delete nextArray;
    delete preArray;
    nextArray = new int[numByTruck];
    preArray = new int[numByTruck];
    for (int i = 0; i < numByTruck; ++i) {
        nextArray[i] = next[i];
        preArray[i] = pre[i];
    }
    assignInitialDrones();
    //other stuff
    
    double max = -INF;
    for (int i = 0; i < numOfRoute; ++i) {
        max = max > timeOfRoute[i] ? max : timeOfRoute[i];
    }
    return max;
}