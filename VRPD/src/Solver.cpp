//
//  Solver.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 4/28/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include "Solver.h"

double DefaultValue::startingTemp = 2;
double DefaultValue::coolingRatio = 0.7;
double DefaultValue::stopObject = 0;
int DefaultValue::numLoop = 5;
int DefaultValue::numIter = 2;
int DefaultValue::numListSize = 10;
int DefaultValue::heuristic = 7;
int DefaultValue::verboseLevel = 0;

void Solver::setParameters(){
    startingTemperatur = 1;
    coolRatio = 0.1;
    numOfLoops = 3;
    numOfItersPerLoop = 1;
    numOfListSize = 10;
    theHeuristics = ONE_POINT_MOVE + TWO_POINT_MOVE + TWO_OPT;
    stoptingObject = 0;
}

void Solver::setParameters(const cmdline::parser &parser){
    startingTemperatur = parser.get<double>("startTemp");
    coolRatio = parser.get<double>("coolRatio");
    numOfLoops = parser.get<int>("numLoop");
    numOfItersPerLoop = parser.get<int>("numIter");
    numOfListSize = parser.get<int>("listSize");
    int h = parser.get<int>("heuristic");
    theHeuristics = 0;
    theHeuristics += h & 4 ? TWO_OPT : 0;
    theHeuristics += h & 2 ? TWO_POINT_MOVE : 0;
    theHeuristics += h & 1 ? ONE_POINT_MOVE : 0;
    if (parser.get<int>("verboseLevel") > 0) {
        debug2 = true;
    }
    if (parser.get<int>("verboseLevel") > 1) {
        debug1 = true;
    }
    if (parser.get<int>("verboseLevel") > 2) {
        debug3 = true;
    }
    stoptingObject = parser.get<double>("stop");
    
}

//void Solver::setVerbose(){
//    debug2 = true;
//}

Solver::Solver(){
    debug1 = false;
    debug2 = false;
    debug3 = false;
}

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
    
    if (!in.is_open()) {
        std::cout << "FILE NOT FOUND!" << std::endl;
        exit(-1);
    }
    
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
    bestNextArray = new int[numCustomer];
    bestPreArray = new int[numCustomer];
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


void Solver::main(const char *filename, const char* outfile){
    readFile(filename);
    findDroneAssignment();
    std::ofstream out(outfile);
    if (!out.good())
    {
        std::cerr << "Error opening the output file " << outfile << std::endl;
        std::cerr << "Displaying results in terminal!" << std::endl;
        displayResult(std::cout);
    }
    else
        displayResult(out);
    out.close();
}

void Solver::displayResult(std::ostream &os) {
    for (int i = 0; i < numCustomer; ++i) {
        droned[i] = bestDroned[i];
    }
    vrpd = new VRPD(*this);
    vrpd->importTruckRoute(bestNextArray, bestPreArray);
    vrpd->createRouteInfo();
    vrpd->assignInitialDrones();
    for (int i = 0; i < vrpd->numOfRoute - 1; ++i) {
        outputTruckRoute(os, i);
        os << "END ROUTE" << std::endl;
    }
    outputTruckRoute(os, vrpd->numOfRoute - 1);
    delete vrpd;
}

void Solver::displayDebugInfo(int type){
    bool debug = debug3;
    vrp->export_solution_buff(vrpd->solution);
    if (debug) {
        for (int i = 0; i < vrpd->numByTruck + 1; ++i) {
        if (vrpd->solution[i] <= 0 && i > 0) {
            std::cout << std::endl;
        }
        if (vrpd->solution[i] == 0) {
            continue;
        }
        int index = originalIndex(vrpd->localIndexToGlobal(abs(vrpd->solution[i])));
        std::cout << index  << " ";
    }
    for (int i = 0; i < vrpd->numOfRoute; ++i) {
        std::cout << vrpd->timeOfRoute[i] << " ";
    }
    std::cout << std::endl;
    }
    std::cout << "Best nextArray: " << std::endl;;
    for (int i = 0; i < numCustomer; ++i) {
        std::cout << bestNextArray[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "Best preArray: " << std::endl;;
    for (int i = 0; i < numCustomer; ++i) {
        std::cout << bestPreArray[i] << " ";
    }
    std::cout << std::endl << std::endl;
}

double Solver::getSolution(){
    const bool debug = debug1;
    vrpd = new VRPD(*this);
    vrp = new VRP(*this);
    vrp->initHelper();
    
    double startingTemp = startingTemperatur, coolingRatio = coolRatio;
    int numLoops = numOfLoops, itersPerLoop = numOfItersPerLoop, nListSize = numOfListSize;
    int heuristics = theHeuristics;
    bool verbose = false;
    vrp->SA_solve(heuristics, startingTemp, coolingRatio, itersPerLoop, numLoops, nListSize, verbose);
    
    if (debug) {
        displayDebugInfo();
    }
    
    trackBestSolution();
    double result = vrp->getCurrentObject();
    delete vrpd;
    delete vrp;
    return result;
}

void Solver::outputTruckRoute(std::ostream &os, int routeID) {
    bool *isServed = new bool [vrpd->lengthOfRoute[routeID]];
    for (int i = 0; i < vrpd->lengthOfRoute[routeID]; ++i) {
        isServed[i] = false;
    }
    for (int i = 0; i < vrpd->truckServiceNum[routeID]; ++i) {
        isServed[vrpd->serviceRoute[routeID][i]] = true;
    }
    std::sort(vrpd->droneDplyAtRoute[routeID].begin(), vrpd->droneDplyAtRoute[routeID].end(), DroneDeploymentLessThan);
    std::vector<DroneDeployment> *dronesSent = new std::vector<DroneDeployment> [vrpd->lengthOfRoute[routeID]];
    std::vector<int> *dronesReturned = new std::vector<int> [vrpd->lengthOfRoute[routeID]];
    std::set<int> availableDrones;
    for (int i = 1; i <= numDrone; ++i) {
        availableDrones.insert(i);
    }
    int point = 0;
    for (std::vector<DroneDeployment>::iterator it = vrpd->droneDplyAtRoute[routeID].begin(); it != vrpd->droneDplyAtRoute[routeID].end(); ++it) {
        for (int i = point; i < it->start; ++i) {
            for (std::vector<int>::iterator it1 = dronesReturned[i].begin(); it1 != dronesReturned[i].end(); ++it1) {
                availableDrones.insert(*it1);
            }
            ++point;
        }
        it->droneID = *availableDrones.begin();
        dronesSent[it->start].push_back(*it);
        dronesReturned[it->end].push_back(it->droneID);
        availableDrones.erase(it->droneID);
    }
    for (int i = 0; i < vrpd->lengthOfRoute[routeID]; ++i) {
        os << originalIndex(vrpd->truckRoutes[routeID][i]) << ",";
        os << (isServed[i] ? "TRUE" : "FALSE") << std::endl;
        for (std::vector<DroneDeployment>::iterator it = dronesSent[i].begin(); it != dronesSent[i].end(); ++it) {
            os << "DEPLOY(Drone" << it->droneID << "," << originalIndex(it->nodeID) << ")" << std::endl;
        }
        if (vrpd->waitingTime[routeID][i] > 0) {
            os << "WAIT(" << vrpd->waitingTime[routeID][i] << ")" << std::endl;
        }
        for (std::vector<int>::iterator it = dronesReturned[i].begin(); it != dronesReturned[i].end(); ++it) {
            os << "RETURN(Drone" << *it << ")" << std::endl;
        }
    }
    delete [] dronesReturned;
    delete [] dronesSent;
    delete [] isServed;
}

DroneAssignmentHelper* Solver::createDroneSiteInfo(){
    DroneAssignmentHelper* droneSite = new DroneAssignmentHelper[numCustomer - 1];
    for (int i = 0; i < numCustomer - 1; ++i) {
        droneSite[i].nodeID = i + 1;
        droneSite[i].distance = cost[0][i + 1];
        int count = 0;
        for (int j = 0; j < numCustomer; ++j) {
            if (dist[i + 1][j] < INF) {
                ++count;
            }
        }
        droneSite[i].degree = count - 1;
    }
    qsort(droneSite, numCustomer - 1, sizeof(class DroneAssignmentHelper), droneAssignHelperCmp);
    return droneSite;
}

void Solver::flipDroneSite(int index){
    droned[index] = !droned[index];
}

void Solver::findDroneAssignment(){
    DroneAssignmentHelper *droneSite = createDroneSiteInfo();
    bool toContinue = true;
    const bool debug = debug2;
    int count = 0;
    const int threshold = 10;
    double best = getSolution();
    while (toContinue && count < threshold) {
        toContinue = false;
        ++count;
        if (debug) std::cout << "Round " << count << std::endl;
        for (int i = 0; i < numCustomer - 1; ++i) {
            if (debug) std::cout << "Checked site " << i << ", Best = " << best << "." << std::endl;
            int index = droneSite[i].nodeID;
            flipDroneSite(index);
            double obj = getSolution();
            if (obj < best) {
                toContinue = true;
                best = obj;
            } else {
                flipDroneSite(index);
            }
            if (best <= stoptingObject) {
                delete [] droneSite;
                return;
            }
        }
    }
    delete [] droneSite;
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
    delete [] bestDroned;
    delete [] bestNextArray;
    delete [] bestPreArray;
}

void Solver::trackBestSolution(){
    if (bestObject > vrp->getCurrentObject()) {
        bestObject = vrp->getCurrentObject();
        for (int i = 0; i < numCustomer; ++i) {
            bestDroned[i] = droned[i];
        }
        int *next = vrp->getNextArray(), *pre = vrp->getPreArray();
        for (int i = 0; i < vrpd->numByTruck; ++i) {
            bestNextArray[i] = next[i];
            bestPreArray[i] = pre[i];
        }
    }
}
