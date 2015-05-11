//
//  ModifiedFlip.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 5/3/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include <stdio.h>
#include "VRPH.h"

bool Flip::evaluate(class VRP *V, int start_point, int end_point, VRPMove *M)
{    
    ///
    /// Evaluates the move of reversing a portion of a route in between nodes
    /// start and end.  
    /// Example:  0-a-b-start-d-e-f-g-h-end-x-y-z-0 becomes
    ///           0-a-b-start-h-g-f-e-d-end-x-y-z-0.
    /// If the move is feasible, the information
    /// regarding the move is stored in the VRPMove data structure M.
    /// start_point must be before end_point in the current route orientation.  
    ///

    if (evaluateLock) {
        return true;
    }
    
    int post_start, pre_end, route_num;
    double old_cost, new_cost;
    double savings, route_len;

    //Check for VRPH_DEPOT nodes!
    if(start_point==VRPH_DEPOT || end_point==VRPH_DEPOT)
        report_error("%s: flip::called with VRPH_DEPOT node-ambiguous\n");

    route_num= V->route_num[start_point];

    if(V->route_num[end_point]!=route_num)
    {
        fprintf(stderr,"%d(route #=%d), %d(route #=%d)\n",start_point, route_num,
            end_point, V->route_num[end_point]);

        report_error("%s: flip attempted using different routes!\n");
    }

    if(V->next_array[start_point]==end_point)
        return false;

    post_start= VRPH_MAX(V->next_array[start_point],VRPH_DEPOT);
    pre_end= VRPH_MAX(V->pred_array[end_point],VRPH_DEPOT);

    if(post_start == pre_end || post_start==end_point || pre_end==start_point)
        return false;    // Nothing to reverse!


    // Need to compute the old costs and the new cost - 

    old_cost=(V->d[start_point][post_start]-1*V->nodes[post_start].service_time) + 
        (V->d[pre_end][end_point] -  1*V->nodes[end_point].service_time) ;
    new_cost=(V->d[start_point][pre_end]-1*V->nodes[pre_end].service_time) + 
        (V->d[post_start][end_point] - 1*V->nodes[end_point].service_time);

    savings=new_cost - old_cost;
    
    // The move satisfies the savings rules - now check feasibility...

    route_len= V->route[route_num].length;

    // Check feasibility
    route_len=route_len+savings;

    if(route_len>V->max_route_length)
        return false;    // infeasible

    // Otherwise it's feasible since no load change occurs
    M->num_affected_routes=1;
//    M->savings=savings;
    M->route_nums[0]=route_num;
    M->route_lens[0]=route_len;
    M->route_loads[0]=V->route[route_num].load;
    M->route_custs[0]= V->route[route_num].num_customers; // no change
    M->new_total_route_length= V->total_route_length+savings;
    M->total_number_of_routes = V->total_number_of_routes;
    M->move_type=FLIP;
    M->num_arguments=2;
    M->move_arguments[0]=start_point;
    M->move_arguments[1]=end_point;
    
    // This is the modified calculation of M.savings
    evaluateLock = true;
    V->remove_dummy();
    int *tmpSol = new int[V->num_original_nodes + 2];
    double oldObject = V->getCurrentObject();
    V->export_solution_buff(tmpSol);
    moveHelper(V, M);
    double newObject = V->vrpd->getDroneDeploymentsolution(*V);
    M->savings = newObject - oldObject;
    V->import_solution_buff(tmpSol);
    delete [] tmpSol;
    evaluateLock = false;
   
//    double threshold = 100;
//    if (M->savings > threshold || M->savings < -threshold) {
//        int tmp = 0;
//    }

    return true;

}

bool Flip::moveHelper(VRP *V, VRPMove *M) {
    
    ///
    /// Makes the actual solution modification implied by the Two-Opt
    /// move involving edges a-b and c-d.  Handles both intraroute and interroute
    /// moves
    ///
    
    int a,b,c,d;

    a=M->eval_arguments[0];b=M->eval_arguments[1];c=M->eval_arguments[2];d=M->eval_arguments[3];

    bool uses_dummy=false;

    // We have at most one VRPH_DEPOT node
    if(a==VRPH_DEPOT)
    {
        uses_dummy=true;
        V->presert_dummy(b);
    }

    if(b==VRPH_DEPOT)
    {
        uses_dummy=true;
        V->postsert_dummy(a);
    }

    if(c==VRPH_DEPOT)
    {
        uses_dummy=true;
        V->presert_dummy(d);
    }

    if(d==VRPH_DEPOT)
    {
        uses_dummy=true;
        V->postsert_dummy(c);
    }

    Flip flip;
    flip.evaluateLock = true;
    
    if(flip.move(V, M->move_arguments[0], M->move_arguments[1])==true)
    {
        
        if(uses_dummy)
            V->remove_dummy();
        
        return true;
    }
    else
    {
        fprintf(stderr,"flip.move(%d,%d) is false\n",M->move_arguments[0],M->move_arguments[1]);
        report_error("%s: Flip error 1\n",__FUNCTION__);
    }
    

   
    return false;
}

bool Flip::evaluateHelper(class VRP *V, int start_point, int end_point, VRPMove *M){
    
    ///
    /// Evaluates the move of reversing a portion of a route in between nodes
    /// start and end.  
    /// Example:  0-a-b-start-d-e-f-g-h-end-x-y-z-0 becomes
    ///           0-a-b-start-h-g-f-e-d-end-x-y-z-0.
    /// If the move is feasible, the information
    /// regarding the move is stored in the VRPMove data structure M.
    /// start_point must be before end_point in the current route orientation.  
    ///

    if (evaluateLock) {
        return true;
    }
    
    int post_start, pre_end, route_num;
    double old_cost, new_cost;
    double savings, route_len;

    //Check for VRPH_DEPOT nodes!
    if(start_point==VRPH_DEPOT || end_point==VRPH_DEPOT)
        report_error("%s: flip::called with VRPH_DEPOT node-ambiguous\n");

    route_num= V->route_num[start_point];

    if(V->route_num[end_point]!=route_num)
    {
        fprintf(stderr,"%d(route #=%d), %d(route #=%d)\n",start_point, route_num,
            end_point, V->route_num[end_point]);

        report_error("%s: flip attempted using different routes!\n");
    }

    if(V->next_array[start_point]==end_point)
        return false;

    post_start= VRPH_MAX(V->next_array[start_point],VRPH_DEPOT);
    pre_end= VRPH_MAX(V->pred_array[end_point],VRPH_DEPOT);

    if(post_start == pre_end || post_start==end_point || pre_end==start_point)
        return false;    // Nothing to reverse!


    // Need to compute the old costs and the new cost - 

    old_cost=(V->d[start_point][post_start]-1*V->nodes[post_start].service_time) + 
        (V->d[pre_end][end_point] -  1*V->nodes[end_point].service_time) ;
    new_cost=(V->d[start_point][pre_end]-1*V->nodes[pre_end].service_time) + 
        (V->d[post_start][end_point] - 1*V->nodes[end_point].service_time);

    savings=new_cost - old_cost;
    
    // The move satisfies the savings rules - now check feasibility...

    route_len= V->route[route_num].length;

    // Check feasibility
    route_len=route_len+savings;

    if(route_len>V->max_route_length)
        return false;    // infeasible

    // Otherwise it's feasible since no load change occurs
    M->num_affected_routes=1;
//    M->savings=savings;
    M->route_nums[0]=route_num;
    M->route_lens[0]=route_len;
    M->route_loads[0]=V->route[route_num].load;
    M->route_custs[0]= V->route[route_num].num_customers; // no change
    M->new_total_route_length= V->total_route_length+savings;
    M->total_number_of_routes = V->total_number_of_routes;
    M->move_type=FLIP;
    M->num_arguments=2;
    M->move_arguments[0]=start_point;
    M->move_arguments[1]=end_point;
    
    // This is the modified calculation of M.savings
    return true;
}