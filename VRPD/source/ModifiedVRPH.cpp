//
//  ModifiedVRPH.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 4/28/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include <stdio.h>
#include "VRPH.h"

void VRP::init(int n){
    
    /// 
    /// Constructor for an n-node problem.
    ///

    int i,j;

    num_nodes=n;
    num_original_nodes=n;
    total_demand=0;
    num_days=0;
    
    next_array = new int[n+2];
    pred_array = new int[n+2];
    route_num = new int[n+2];
    route = new VRPRoute[n+2];
    routed = new bool[n+2];
    best_sol_buff = new int[n+2];
    current_sol_buff = new int[n+2];
    search_space = new int[n+2];
    nodes = new VRPNode[n+2];    
    // Add an extra spot for the VRPH_DEPOT and for the dummy node
    
    symmetric=true;
    // Set to false only when we encounter FULL_MATRIX file

    forbid_tiny_moves=true;    
    // Default is to allow these moves

    d=NULL;
    d = new double* [n + 2];
    d[0] = new double [(n + 2) * (n + 2)];
    for (int i = 1; i < n + 2; ++i) {
        d[i] = d[i - 1] + n + 2;
    }
    // The distance matrix is allocated when the problem is loaded
    fixed=new bool*[n+2];
    fixed[0]=new bool[(n+2)*(n+2)];
    for(i=1;i<n+2;i++)
        fixed[i]=fixed[i-1]+(n+2);
    for(i=0;i<n+2;i++)
    {
        routed[i]=false;
        for(j=0;j<n+2;j++)
            fixed[i][j]=false;
    }

    // Set these to default values--they may change once
    // we read the file.

    min_vehicles=-1;
    has_service_times=false;
    max_route_length=VRP_INFINITY;
    orig_max_route_length=VRP_INFINITY;
    total_route_length=0.0;
    best_known=VRP_INFINITY;
    depot_normalized=false; 
    // Will be set to true if we shift nodes so VRPH_DEPOT is at origin
    
    // These are for record-to-record travel
    record = 0.0;
    deviation = VRPH_DEFAULT_DEVIATION;        

    // For keeping track of the statistics

    for(i=0;i<NUM_HEURISTICS;i++)
    {
        num_evaluations[i]=0;
        num_moves[i]=0;

    }

    total_service_time = 0.0;

    // Create the solution warehouse
    this->solution_wh=new VRPSolutionWarehouse(NUM_ELITE_SOLUTIONS,n);

    this->tabu_list=new VRPTabuList(MAX_VRPH_TABU_LIST_SIZE);
    this->route_wh=NULL;

    // Set this to true only if we have valid coordinates 
    // This is valid only for plotting the solution
    can_display=false;
    
}
void VRP::create_distance_matrix(int type)
{    
    ///
    /// Creates the O(n^2) size distance matrix for the
    /// provided data using the distance function referenced by type.
    /// If the type is EXPLICIT, then the entire distance matrix should
    /// be provided in the actual TSPLIB file.
    ///
    
    for (int i = 0; i < num_nodes + 2; ++i) {
        for (int j = 0; j < num_nodes + 2; ++j) {
            d[i][j] = d[j][i] = vrpd->cost[nodes[i].id][nodes[j].id];
        }
    }
    
//    int i,j,k,n;
//
//
//    if(type==VRPH_EXPLICIT)
//        // We have presumably already loaded in the distance matrix!
//        return;
//
//    n= this->num_original_nodes;
//    k=0;
//
//    // Otherwise construct the matrix - we store the whole thing even though
//    // it is symmetric as we found that this was quite a bit faster...
//
//    for(i=0;i<=n+1;i++)
//    {
//        for(j=0;j<=n+1;j++)
//            this->d[i][j]=VRPDistance(type, this->nodes[i].x,this->nodes[i].y,this->nodes[j].x,
//                this->nodes[j].y) + this->nodes[j].service_time ;
//
//    }
//
    return;

}

VRP::VRP(Solver& solver): vrpd(solver.vrpd){
    
    init(solver.vrpd->getNumByTruck() - 1);
    strcpy(name, "VRP Helper");
    problem_type = VRPH_CVRP;
    best_known = INFINITY;
    num_nodes = solver.vrpd->getNumByTruck() - 1;
    matrix_size = num_nodes;
    dummy_index = num_nodes + 1;
    max_veh_capacity = solver.truckCap;
    orig_max_veh_capacity = max_veh_capacity;
    can_display = true;
    for (int i = 0; i <= num_nodes; ++i) {
        nodes[i].id = vrpd->localIndexToGlobal(i);
        nodes[i].x = solver.nodes[nodes[i].id].getX();
        nodes[i].y = solver.nodes[nodes[i].id].getY();
    }
    nodes[dummy_index].id = nodes[0].id;
    nodes[dummy_index].x = nodes[0].x;
    nodes[dummy_index].y = nodes[0].y;
    for (int i = 0; i < num_nodes + 1; ++i) {
        nodes[i].demand = 1;
        nodes[i].service_time = 0;
    }
    nodes[num_nodes + 1].demand = 0;
    nodes[num_nodes + 1].service_time = 0;
    create_neighbor_lists(num_nodes);
    create_distance_matrix(0);
    max_route_length = vrpd->INF;
}

int* VRP::getNextArray() const{
    return next_array;
}

int* VRP::getPreArray() const{
    return pred_array;
}

double VRP::getCurrentObject() const{
    return currentObject;
}

void VRP::updateCurrentObject(double object){
    currentObject = object;
}

bool VRP::hasDummy() const{
    return dummyIndicator;
}

void VRP::capture_best_solution()
{
    ///
    /// Determines if the current solution is the best found so far.
    ///

    if( (this->currentObject < this->bestObject) &&
        (VRPH_ABS(this->currentObject - this->bestObject) > VRPH_EPSILON) )
    {
        this->bestObject=this->currentObject;
        this->export_solution_buff(this->best_sol_buff);
        
    }

    
//    if(this->total_route_length < this->solution_wh->worst_obj || 
//        this->solution_wh->num_sols < this->solution_wh->max_size)
//    {
//        VRPSolution this_sol(this->num_nodes);
//
//        this_sol.obj=this->total_route_length;
//        this_sol.in_IP=false;
//
//        // Export buffer
//        this->export_canonical_solution_buff(this_sol.sol);
//        
//        this->solution_wh->add_sol(&this_sol, 0); 
//        // We don't know any information to help us know where to insert
//    }

    return;
    
}

void VRP::initHelper(){
    vrpd->generateInitialTruckRoute();
    import_solution_buff(vrpd->solution);
    vrpd->importTruckRoute(getNextArray(), getPreArray());
    currentObject = vrpd->deployAllDrones();
    capture_best_solution();
}
