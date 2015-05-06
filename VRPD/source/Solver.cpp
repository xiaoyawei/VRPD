//
//  Solver.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 4/28/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include "Solver.h"

int Solver::newIndex(int index) const{
    if (index == depotIndex) {
        return 0;
    } else {
        return index > depotIndex ? index - 1 : index;
    }
}

int Solver::originalIndex(int index) const{
    if (index == 0) {
        return depotIndex;
    } else {
        return index >= depotIndex ? index + 1 : index;
    }
}

bool Solver::readFile(const char *filename){
    int m;
    const int buffSize = 256;
    char line[buffSize];
    std::ifstream in(filename);
    //skip the words
    for (int i = 0; i < 10; ++i) in.getline(line, buffSize);
    
    in.getline(line, buffSize);
    sscanf(line, "Fleet Size (k):%d\n", &numFleet);
    in.getline(line, buffSize);
    in.getline(line, buffSize);
    sscanf(line, "Alpha:%lf\n", &alpha);
    in.getline(line, buffSize);
    sscanf(line, "Drones Per Vehicle (l):%d\n", &numDrone);
    in.getline(line, buffSize);
    sscanf(line, "Drone Flight Duration (t_drone):%lf\n", &tDrone);
    in.getline(line, buffSize);
    sscanf(line, "Truck Capacity (cap_truck):%d\n", &truckCap);
    in.getline(line, buffSize);
    sscanf(line, "Depot ID(s) (d):%d\n", &depotIndex);
    in.getline(line, buffSize);
    sscanf(line, "N:%d\n", &numCustomer);
    in.getline(line, buffSize);
    sscanf(line, "M:%d\n", &m);
    droned = new bool[numCustomer];
    bestDroned = new bool[numCustomer];
    for (int i = 0; i < numCustomer; ++i) {
        droned[i] = false;
    }
//    droneDplyAtNode = new DroneDeployment[numCustomer];
    
    dist = new double*[numCustomer];
    dist[0] = new double[numCustomer * numCustomer];
    for (int i = 1; i < numCustomer; ++i)
        dist[i] = dist[i - 1] + numCustomer;
    nextNode = new int*[numCustomer];
    nextNode[0] = new int[numCustomer * numCustomer];
    for (int i = 1; i < numCustomer; ++i) {
        nextNode[i] = nextNode[i - 1] + numCustomer;
    }
    cost = new double*[numCustomer];
    cost[0] = new double[numCustomer * numCustomer];
    for (int i = 1; i < numCustomer; ++i)
        cost[i] = cost[i - 1] + numCustomer;
    pathLength = new int*[numCustomer];
    pathLength[0] = new int[numCustomer * numCustomer];
    for (int i = 1; i < numCustomer; ++i)
        pathLength[i] = pathLength[i - 1] + numCustomer;
    for (int i = 0; i < numCustomer; ++i) {
        for (int j = 0; j < numCustomer; ++j) {
            dist[i][j] = cost[i][j] = INF;
            nextNode[i][j] = 0;
            pathLength[i][j] = 0;
        }
    }
    for (int i = 0; i < numCustomer; ++i) {
        dist[i][i] = cost[i][i] = 0;
    }
    for (int i = 0; i < 4; ++i) in.getline(line, buffSize);
    for (int i = 0; i < m; ++i){
        int a, b;
        double c;
        in.getline(line, buffSize);
        sscanf(line, "%d,%d,%lf,false\n", &a, &b, &c);
        a = newIndex(a);
        b = newIndex(b);
        dist[a][b] = dist[b][a] = cost[a][b] = cost[b][a] = c;
//        dist[b][a] = c;
//        cost[a][b] = c;
//        cost[b][a] = c;
        nextNode[a][b] = b;
        nextNode[b][a] = a;
        pathLength[a][b] = pathLength[b][a] = 1;
    }
    for (int k = 0; k < numCustomer; ++k) {
        for (int i = 0; i < numCustomer; ++i) {
            for (int j = 0; j < numCustomer; ++j) {
                if (cost[i][j] > cost[i][k] + cost[k][j]) {
                    cost[i][j] = cost[i][k] + cost[k][j];
                    nextNode[i][j] = nextNode[i][k];
                    pathLength[i][j] = pathLength[i][k] + pathLength[k][j];
                }
            }
        }
    }
    
    nodes = new Coordinate[numCustomer];
    for (int i = 0; i < 4; ++i) in.getline(line, buffSize);
    for (int i = 1; i <= numCustomer; ++i) {
        double x, y;
        in.getline(line, buffSize);
        sscanf(line, "%lf,%lf\n", &x, &y);
        nodes[newIndex(i)].set(x, y);
    }
    for (int i = 1; i < numCustomer; ++i) {
        nodes[i].set(nodes[i].getX() - nodes[0].getX(), nodes[i].getY() - nodes[0].getY());
    }
    nodes[0].set(0, 0);
    
    in.close();
    createNodeInfo();
//    for (int i = 0; i < numCustomer; ++i) {
//        for (int j = 0; j < numCustomer; ++j) {
//            std::cout << cost[i][j] << " ";
//        }
//        std::cout << std::endl;
//    }
    return true;
}

//void Solver::main(const char* filename){
//    readFile(filename);
//    setupHelper();
//    
//    
//    vrpd = new VRPD(*this);
//    vrpd->generateInitialTruckRoute();
//    for (int i = 0; i < vrpd->numByTruck + 1; ++i) {
//        if (vrpd->solution[i] <= 0) {
//            std::cout << std::endl;
//        }
//        std::cout << (abs(vrpd->solution[i])) << " ";
//    }
//    vrp = new VRP(*this);
//    vrp->import_solution_buff(vrpd->solution);
//    vrpd->importTruckRoute(vrp->getNextArray(), vrp->getPreArray());
////    vrpd->createRouteInfo();
//    double max = vrpd->deployAllDrones();
//    vrp->updateCurrentObject(max);
//    for (int i = 0; i < vrpd->numOfRoute; ++i) {
//        std::cout << vrpd->timeOfRoute[i] << std::endl;
//    }
//    std::cout << max << std::endl;
//    double startingTemp = 2, coolingRatio = 0.9;
//    int numLoops = 50, itersPerLoop = 2, nListSize = 10;
////    int heuristics = ONE_POINT_MOVE + TWO_POINT_MOVE + TWO_OPT;
//    int heuristics = ONE_POINT_MOVE + TWO_POINT_MOVE;
//    bool verbose = false;
//    vrp->SA_solve(heuristics, startingTemp, coolingRatio, itersPerLoop, numLoops, nListSize, verbose);
//    std::cout << vrp->getCurrentObject() << std::endl;
//    std::cout << vrp->vrpd->getDroneDeploymentsolution(*vrp) << std::endl;
//    vrp->export_solution_buff(vrpd->solution);
//    for (int i = 0; i < vrpd->numByTruck + 1; ++i) {
//        if (vrpd->solution[i] <= 0) {
//            std::cout << std::endl;
//        }
//        std::cout << (abs(vrpd->solution[i])) << " ";
//    }
//    vrpd->importTruckRoute(vrp->getNextArray(), vrp->getPreArray());
//    max = vrpd->deployAllDrones();
//    vrp->updateCurrentObject(max);
//    for (int i = 0; i < vrpd->numOfRoute; ++i) {
//        std::cout << vrpd->timeOfRoute[i] << std::endl;
//    }
//    delete vrpd;
//    delete vrp;
//    std::cout << getSolution() << std::endl;
//}

void Solver::main(const char *filename){
    readFile(filename);
    setupHelper();
}

double Solver::getSolution(){
    vrpd = new VRPD(*this);
    vrp = new VRP(*this);
    vrp->initHelper();
    
    double startingTemp = 2, coolingRatio = 0.9;
    int numLoops = 50, itersPerLoop = 2, nListSize = 10;
    int heuristics = ONE_POINT_MOVE + TWO_POINT_MOVE + TWO_OPT;
    bool verbose = false;
    vrp->SA_solve(heuristics, startingTemp, coolingRatio, itersPerLoop, numLoops, nListSize, verbose);
    
    trackBestSolution();
    double result = vrp->getCurrentObject();
    delete vrpd;
    delete vrp;
    return result;
}

void Solver::setupHelper(){
    for (int i = 1; i < numCustomer; ++i) {
        if (degree[i] == 1) {
            droned[i] = true;
        }
    }
}

void Solver::createNodeInfo(){
    degree = new int[numCustomer];
    radius = new double[numCustomer];
    for (int i = 0; i < numCustomer; ++i) {
        int count = 0;
        for (int j = 0; j < numCustomer; ++j) {
            if (dist[i][j] < INF) {
                ++count;
            }
        }
        degree[i] = count - 1;
        radius[i] = cost[0][i];
    }
}

Solver::~Solver(){
    delete [] degree;
    delete [] radius;
    delete [] droned;
    delete [] cost[0];
    delete [] dist[0];
    delete [] cost;
    delete [] dist;
    delete [] nodes;
    delete [] nextNode[0];
    delete [] pathLength[0];
    delete [] nextNode;
    delete [] pathLength;
}

void Solver::trackBestSolution(){
    if (bestObject > vrp->getCurrentObject()) {
        bestObject = vrp->getCurrentObject();
        for (int i = 0; i < numCustomer; ++i) {
            bestDroned[i] = droned[i];
        }
    }
}