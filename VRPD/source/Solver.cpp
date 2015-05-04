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
        return index > depotIndex ? index - 1: index;
    }
}

int Solver::originalIndex(int index) const{
    if (index == 0) {
        return depotIndex;
    } else {
        return index > depotIndex ? index + 1: index;
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
    for (int i = 1; i < numCustomer; ++i)
        cost[i] = cost[i - 1] + numCustomer;
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
        dist[a][b] = c;
        dist[b][a] = c;
        cost[a][b] = c;
        cost[b][a] = c;
        nextNode[a][b] = b;
        nextNode[b][a] = a;
        pathLength[a][b] = pathLength[b][a] = 1;
    }
    for (int i = 0; i < numCustomer; ++i) {
        for (int j = 0; j < numCustomer; ++j) {
            for (int k = 0; k < numCustomer; ++k) {
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
    for (int i = 0; i < numCustomer; ++i) {
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
    for (int i = 0; i < numCustomer; ++i) {
        for (int j = 0; j < numCustomer; ++j) {
            std::cout << cost[i][j] << " ";
        }
        std::cout << std::endl;
    }
    return true;
}