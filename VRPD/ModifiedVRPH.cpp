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

VRP::VRP(Solver& solver): vrpd(solver.vrpd){
    
    init(solver.vrpd->getNumByTruck());
    
}