//
//  ModifiedSwapEnds.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 5/4/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include <stdio.h>
#include "VRPH.h"

bool SwapEnds::evaluate(class VRP *V, int a, int v, VRPMove *M)
{

    ///
    /// This function takes the routes containing nodes a and v
    /// and evaluates the swapping of the ends of the routes following a and v respectively,
    /// subject to the provided rules.
    /// Example:  VRPH_DEPOT-i-a-j-k-l-VRPH_DEPOT and VRPH_DEPOT-t-u-v-x-y-z-VRPH_DEPOT becomes
    /// VRPH_DEPOT-i-a-x-y-z-VRPH_DEPOT and VRPH_DEPOT-t-u-v-j-k-l-VRPH_DEPOT
    ///

    
    int load_after_a, load_after_v, new_a_load, new_v_load, added_to_a, added_to_v, n, b, w;
    double new_a_len, new_v_len ;
    double savings;
    VRPSegment Sa, Sv;


    if(a==VRPH_DEPOT || v==VRPH_DEPOT)
        report_error("%s: Swap ends called with depot; Move doesn't make sense\n",__FUNCTION__);

    n = V->num_nodes;
    

    if(V->route_num[a] == V->route_num[v])
    {
        fprintf(stderr,"a=%d; v=%d; %d==%d!!\n",a,v,V->route_num[a], V->route_num[v]);
        report_error("%s: swap ends called with a and v in same route!\n",__FUNCTION__);
    }
    
    w = VRPH_MAX(V->next_array[v],0);
    b = VRPH_MAX(V->next_array[a],0);
    
    savings = ((V->d[a][w] + V->d[v][b]) - (V->d[a][b] + V->d[v][w]));
    
    V->get_segment_info(VRPH_DEPOT, a, &Sa);
    added_to_v= V->route[V->route_num[a]].num_customers-Sa.num_custs;

    load_after_a = V->route[V->route_num[a]].load - Sa.load;

    V->get_segment_info(VRPH_DEPOT, v, &Sv);
    added_to_a=V->route[V->route_num[v]].num_customers-Sv.num_custs;

    load_after_v = V->route[V->route_num[v]].load - Sv.load;
    
    
    /// Example: ( a & v input): VRPH_DEPOT-i-a-b-j-k-l-VRPH_DEPOT and VRPH_DEPOT-t-u-v-w-x-y-z-VRPH_DEPOT becomes
    /// VRPH_DEPOT-i-a-w-x-y-z-VRPH_DEPOT and VRPH_DEPOT-t-u-v-b-j-k-l-VRPH_DEPOT

    new_a_len = Sa.len + V->route[V->route_num[v]].length - Sv.len + V->d[a][w]-V->d[v][w];
    new_a_load = Sa.load + load_after_v;
    new_v_len = Sv.len + V->route[V->route_num[a]].length - Sa.len  + V->d[v][b]-V->d[a][b];
    new_v_load = Sv.load + load_after_a;

    if(new_a_len > V->max_route_length || new_v_len > V->max_route_length 
        || new_a_load > V->max_veh_capacity || new_v_load > V->max_veh_capacity )
        // We violate some capacity constraint & the move is infeasible
        return false;


    // else the move is feasible and meets rules - record the move;

    M->num_affected_routes=2;
    M->route_nums[0]=V->route_num[a];
    M->route_nums[1]=V->route_num[v];
    M->savings=savings;
    M->route_lens[0]=new_a_len;
    M->route_lens[1]=new_v_len;
    M->route_loads[0]=new_a_load;
    M->route_loads[1]=new_v_load;
    M->route_custs[0]=V->route[V->route_num[a]].num_customers-added_to_v+added_to_a;
    M->route_custs[1]=V->route[V->route_num[v]].num_customers-added_to_a+added_to_v;
    M->new_total_route_length= V->total_route_length+M->savings;
    M->total_number_of_routes=V->total_number_of_routes;//none destroyed here
    M->move_type=SWAP_ENDS;
    M->num_arguments=2;
    M->move_arguments[0]=a;
    M->move_arguments[1]=v;
    
    // This is the modified calculation of M.savings
    V->remove_dummy();
    int *tmpSol = new int[V->num_original_nodes + 2];
    double oldObject = V->getCurrentObject();
    V->export_solution_buff(tmpSol);
    move(V, M->move_arguments[0], M->move_arguments[1]);
    double newObject = V->vrpd->getDroneDeploymentsolution(*V);
    M->savings = newObject - oldObject;
    V->import_solution_buff(tmpSol);
    delete [] tmpSol;
    
    return true;    
    
}