//
//  VRPD.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 4/26/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include "VRPD.h"

int VRPD::getNumByTruck() const{
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

VRPD::VRPD(){}

int VRPD::localIndexToGlobal(const int index)const{
    return index >= numByTruck ? -1 : localToGlobal[index];
}

int VRPD::globalIndexToLocal(const int index) const {
    return droned[index] ? -1 : globalToLocal[index];
}

void VRPD::setup(const Solver& solver){
    reset();
    alpha = solver.alpha;
    cost = solver.cost;
    depotIndex = solver.depotIndex;
    dist = solver.dist;
    nextNode = solver.nextNode;
    pathLength = solver.pathLength;
    droned = solver.droned;
    numCustomer = solver.numCustomer;
    numDrone = solver.numDrone;
    numFleet = solver.numFleet;
    tDrone = solver.tDrone;
    truckCap = solver.truckCap;
    getDroneAssignment(solver.droned);
    nodes = solver.nodes;
    int count = 0;
    globalToLocal[0] = localToGlobal[0] = 0;
    for (int i = 0; i < solver.numCustomer; ++i) {
        if (!droned[i]) {
            globalToLocal[i] = i - count;
            localToGlobal[i - count] = i;
        }else{
            ++count;
        }
    }
    
    solution = new int[numByTruck + 1];
}

void VRPD::reset(){
//    delete [] solution;
}

VRPD::VRPD(const Solver& solver){
    localToGlobal = new int[solver.numCustomer];
    globalToLocal = new int[solver.numCustomer];
    nextArray = new int[solver.numCustomer];
    preArray = new int[solver.numCustomer];
    setup(solver);
}

VRPD::~VRPD(){
    clearRouteInfo();
    delete [] localToGlobal;
    delete [] globalToLocal;
    delete [] nextArray;
    delete [] preArray;
    delete [] solution;
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
    if (loadAtRoute[routeID] >= truckCap) {
        return deply;
    }
    for (int len = 0; len < length; ++len){
        for (int i = 0; i + len < length; ++i) {
            if (len == 0) {
                time[i][i + len] = waitingTime[i];
            } else {
                time[i][i + len] = time[i][i + len - 1] + dist[route[i + len - 1]][route[i + len]] + waitingTime[i + len];
            }
            bool outOfDrone = false;
            for (int j = i; j <= i + len; ++j) { // further modification
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
    if (deply.start < 0 || deply.end < 0){
        loadAtRoute[deply.routeID] = truckCap + 1;
        timeOfRoute[deply.routeID] = INF;
        return;
    }
    for (int i = deply.start; i <= deply.end; ++i) { // need fine-granuity modification
        ++deployedDrone[i];
    }
    double extraTime = alpha * (cost[route[deply.start]][deply.nodeID] + cost[deply.nodeID][route[deply.end]]) - waitingTime[deply.start];
    for (int i = deply.start + 1; i <= deply.end; ++i) {
        extraTime -= dist[route[i - 1]][route[i]] + waitingTime[i];
    }
    if (extraTime > 0) {
        waitingTime[deply.end] += extraTime;
        timeOfRoute[deply.routeID] += extraTime;
    }
    droneDplyAtNode[deply.nodeID] = deply;
    droneDplyAtRoute[deply.routeID].push_back(deply);
    ++loadAtRoute[deply.routeID];
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

void VRPD::clearRouteInfo(){
    if (routeInfoCreated) {
        routeInfoCreated = false;
    } else {
        return;
    }
    if (numOfRoute > 0) {
        for (int i = 0; i < numOfRoute; ++i) {
            delete [] waitingTime[i];
            delete [] deployedDrone[i];
            delete [] truckRoutes[i];
        }
        delete [] timeOfRoute;
        delete [] lengthOfRoute;
        delete [] loadAtRoute;
        delete [] waitingTime;
        delete [] deployedDrone;
        delete [] truckRoutes;
        delete [] droneDplyAtRoute;
        delete [] droneDplyAtNode;
        numOfRoute = 0;
    }
}

void VRPD::createRouteInfo(){
    
    if (routeInfoCreated) {
        return;
    } else {
        routeInfoCreated = true;
    }
    
    for (int i = 0; nextArray[i] != 0; i = abs(nextArray[i])) {
        if (nextArray[i] < 0) {
            ++numOfRoute;
        }
    }
    droneDplyAtNode = new DroneDeployment[numCustomer];
    droneDplyAtRoute = new std::vector<DroneDeployment>[numOfRoute];
    lengthOfRoute = new int[numOfRoute];
    waitingTime = new double*[numOfRoute];
    deployedDrone = new int*[numOfRoute];
    truckRoutes = new int*[numOfRoute];
    timeOfRoute = new double[numOfRoute];
    loadAtRoute = new int[numOfRoute];
    int routeIndex = 0;
    for (int i = abs(nextArray[0]); i > 0;) {
        int load = 1, len = pathLength[0][i], j, pre = i;
        for (j = nextArray[i]; j > 0; j = nextArray[j]) {
            ++load;
            len += pathLength[pre][j];
            pre = j;
        }
        len += pathLength[pre][0];
        i = abs(j);
        lengthOfRoute[routeIndex] = len + 1;
        loadAtRoute[routeIndex] = load;
        ++routeIndex;
    }
    
    int nodeIndex = abs(nextArray[0]);
    for (int i = 0; i < numOfRoute; ++i) {
        timeOfRoute[i] = 0;
        int pointer = 0;
        waitingTime[i] = new double[lengthOfRoute[i]];
        deployedDrone[i] = new int[lengthOfRoute[i]];
        truckRoutes[i] = new int[lengthOfRoute[i]];
        waitingTime[i][0] = waitingTime[i][lengthOfRoute[i] - 1] = 0;
        deployedDrone[i][0] = deployedDrone[i][lengthOfRoute[i] - 1] = 0;
        truckRoutes[i][lengthOfRoute[i] - 1] = 0;
        for (int j = 0; j < lengthOfRoute[i]; ++j) {
            waitingTime[i][j] = 0;
            deployedDrone[i][j] = 0;
        }
        for (int k = 0; k != nodeIndex; k = nextNode[k][nodeIndex]) {
            truckRoutes[i][pointer++] = k;
        }
        for (int j = nextArray[nodeIndex]; j > 0; j = nextArray[j]) {
            for (int k = nodeIndex; k != j; k = nextNode[k][j]) {
                truckRoutes[i][pointer++] = k;
            }
            nodeIndex = j;
        }
        for (int k = nodeIndex; k != 0; k = nextNode[k][0]) {
            truckRoutes[i][pointer++] = k;
        }
        nodeIndex = abs(nextArray[nodeIndex]);
        timeOfRoute[i] = getBasicRouteTime(truckRoutes[i], lengthOfRoute[i]);
    }
    
}

void VRPD::assignInitialDrones(int method){
    
    if (method == 1) {
        for (int i = 1; i < numCustomer; ++i) {
            if (droned[i]) {
                DroneDeployment minDply(-1, -1, 0, i, INF);
                for (int j = 0; j < numOfRoute; ++j) {
                    DroneDeployment dply = getDroneDeployment(i, truckRoutes[j], deployedDrone[j], waitingTime[j], lengthOfRoute[j], j);
                    if (hideNegative(dply.addedTime) + timeOfRoute[j] < hideNegative(minDply.addedTime) + timeOfRoute[minDply.routeID]) {
                        minDply = dply;
                    }
                }
//                droneDplyAtNode[i] = tmp;
//                droneDplyAtRoute[minDply.routeID].push_back(minDply);
                if (minDply.start < 0 || minDply.end < 0) {
                    for (int j = 0; j < numOfRoute; ++j) {
                        loadAtRoute[j] = truckCap + 1;
                        timeOfRoute[j] = INF;
                    }
                    return;
                }
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

void VRPD::importTruckRoute(const int *next, const int *pre){
    for (int i = 0; i < numByTruck; ++i) {
        int sign = next[i] >= 0 ? 1 : -1;
        nextArray[localIndexToGlobal(i)] = sign * localIndexToGlobal(abs(next[i]));
    }
    for (int i = 0; i < numByTruck; ++i) {
        int sign = pre[i] >= 0 ? 1 : -1;
        preArray[localIndexToGlobal(i)] = sign * localIndexToGlobal(abs(pre[i]));
    }
}

double VRPD::deployAllDrones(){
    clearRouteInfo();
    createRouteInfo();
    assignInitialDrones();
    //other stuff
    
    double max = -INF;
    for (int i = 0; i < numOfRoute; ++i) {
        max = max > timeOfRoute[i] ? max : timeOfRoute[i];
    }
    return max;
}

int* VRPD::generateInitialTruckRoute(){
    using std::vector;
    int index = 0;
    InitialNodeAssignmentHelper *initialNodes = new InitialNodeAssignmentHelper[numByTruck - 1];
    for (int i = 1; i < numCustomer; ++i) {
        if (!droned[i]) {
            initialNodes[index].nodeID = i;
            initialNodes[index].length = cost[0][i];
            ++index;
        }
    }
    qsort(initialNodes, numByTruck - 1, sizeof(class InitialNodeAssignmentHelper), initialNodeHelperCmp);
    vector<int> *routes = new vector<int>[numFleet];
    int routeLength[numFleet], routeLoad[numFleet];
    for (int i = 0; i < numFleet; ++i) {
        routes[i].push_back(0);
        routes[i].push_back(0);
        routeLength[i] = routeLoad[i] = 0;
    }
    index = 0;
    for (int i = 1; i < numCustomer; ++i) {
        if (!droned[i]) {
            int routeID = -1, insertID = -1;
            double min = INF;
            for (int j = 0; j < numFleet; ++j) {
                if (routeLoad[j] >= truckCap) {
                    continue;
                }
                for (int k = 0; k < routes[j].size() - 1; ++k) {
                    int a = routes[j][k], b = routes[j][k + 1], c = initialNodes[index].nodeID;
                    double addedLength = this->cost[a][c] + this->cost[c][b] - this->cost[a][b];
                    if (addedLength + routeLength[j] < min) {
                        routeID = j;
                        insertID = k;
                        min = routeLength[j] + addedLength;
                    }
                }
            }
            routes[routeID].insert(routes[routeID].begin() + insertID + 1, initialNodes[index].nodeID);
            routeLength[routeID] = min;
            ++routeLoad[routeID];
            ++index;
        }
    }
    solution[0] = numByTruck - 1;
    index = 0;
    for (int i = 0; i < numFleet; ++i) {
        for (int j = 1; j < routes[i].size() - 1; ++j) {
            if (j == 1) {
                solution[++index] = -globalIndexToLocal(routes[i][j]);
            } else {
                solution[++index] = globalIndexToLocal(routes[i][j]);
            }
        }
    }
    solution[++index] = 0;
    isGlobalSolution = false;
    return solution;
}

double VRPD::getDroneDeploymentsolution(const VRP& vrp){
    importTruckRoute(vrp.getNextArray(), vrp.getPreArray());
    double max = deployAllDrones();
    return max;
}

void VRPD::localSolutionToGlobal(){
    if (!isGlobalSolution) {
        isGlobalSolution = true;
        for (int i = 1; i <= numByTruck; ++i) {
            int sign = solution[i] >= 0 ? 1 : -1;
            solution[i] = sign * localIndexToGlobal(abs(solution[i]));
        }
    }
}

void VRPD::globalSolutionToLocal(){
    if (isGlobalSolution){
        isGlobalSolution = false;
        for (int i = 1; i <= numByTruck; ++i){
            int sign = solution[i] >= 0 ? 1 : -1;
            solution[i] = sign * globalIndexToLocal(abs(solution[i]));
        }
    }
}

double VRPD::hideNegative(const double x) const{
    return x < 0 ? 0 : x;
}