//
//  SA_Solver.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 5/1/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include <stdio.h>
#include "VRPH.h"

double VRP::SA_solve(int heuristics, double start_temp, double cool_ratio,
                     int iters_per_loop, int num_loops, int nlist_size, bool verbose)    
{
    ///
    /// Uses the given parameters to generate a VRP solution using Simulated Annealing.
    /// Assumes that data has already been imported into V and that we have
    /// some existing solution.  Returns the total route length of the best solution found.
    ///

    this->temperature = start_temp;
    this->cooling_ratio = cool_ratio;

    int ctr, n, j,  i,  R, rules, random, fixed, neighbor_list, objective;

    if(heuristics & VRPH_RANDOMIZED)
        random=VRPH_RANDOMIZED;
    else
        random=0;

    if(heuristics & VRPH_FIXED_EDGES)
        fixed=VRPH_FIXED_EDGES;
    else
        fixed=0;

    if(heuristics & VRPH_USE_NEIGHBOR_LIST)
        neighbor_list=VRPH_USE_NEIGHBOR_LIST;
    else
        neighbor_list=0;

    objective=VRPH_SAVINGS_ONLY;
    // default strategy

    if(heuristics & VRPH_MINIMIZE_NUM_ROUTES)
        objective=VRPH_MINIMIZE_NUM_ROUTES;
    
    n=num_nodes;

    // The perm[] array will contain all the nodes in the current solution
    int *perm;
    perm=new int[this->num_nodes];
    j=VRPH_ABS(this->next_array[VRPH_DEPOT]);
    for(i=0;i<this->num_nodes;i++)
    {
        perm[i]=j;
        if(!routed[j])
            report_error("%s: Unrouted node in solution!!\n");

        j=VRPH_ABS(this->next_array[j]);
    }
    if(j!=VRPH_DEPOT)
        report_error("%s: VRPH_DEPOT is not last node in solution!!\n");

    // Define the heuristics we may use

    OnePointMove OPM;
    TwoPointMove TPM;
    TwoOpt         TO;
    OrOpt         OR;
    ThreeOpt     ThreeO;
    CrossExchange    CE;
    ThreePointMove ThreePM;

    double start_val;

    this->export_solution_buff(this->best_sol_buff);
    // We are assuming we have an existing solution

    // Set the neighbor list size used in the improvement search
    this->neighbor_list_size=VRPH_MIN(nlist_size, num_nodes);

    best_total_route_length=this->total_route_length;
    normalize_route_numbers();

    ctr=0;

    // The idea is to perform num_loops loops of num_iters iterations each.
    // For each iteration, run through the given heuristic operation at random
    // and perform a Simulated Annealing search.

    rules=VRPH_USE_NEIGHBOR_LIST+VRPH_FIRST_ACCEPT+VRPH_SIMULATED_ANNEALING+VRPH_SAVINGS_ONLY;

    double worst_obj=0;
    for(ctr=0;ctr<num_loops;ctr++)
    {
        if(verbose)
        {
            printf("\nctr=%d of %d, temp=%f, obj=%f (overall best=%f; worst=%f)\n",ctr,num_loops,
                this->temperature, 
                this->total_route_length,this->best_total_route_length,worst_obj);
            fflush(stdout);
        }
        // Reset worst_obj;
        worst_obj=0;

        // Cool it...
        this->temperature = this->cooling_ratio * this->temperature;


        for(int k=0; k < iters_per_loop; k++)
        {
            start_val=total_route_length;
            if(heuristics & THREE_OPT)
            {
                rules=VRPH_SIMULATED_ANNEALING+VRPH_INTRA_ROUTE_ONLY+random+fixed+objective;
                normalize_route_numbers();
                R=total_number_of_routes;
                for(i=1; i<=R; i++)    
                {
                    ThreeO.route_search(this,i,rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }

            }


            if(heuristics & ONE_POINT_MOVE)
            {
                rules=VRPH_SIMULATED_ANNEALING+neighbor_list+random+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);

                for(i=1;i<=n;i++)    
                {

                    OPM.search(this,perm[i-1],rules);
                    if(getCurrentObject() > worst_obj)
                        worst_obj=this->total_route_length;
                }

            }



            if(heuristics & TWO_POINT_MOVE)
            {
                rules=VRPH_SIMULATED_ANNEALING+neighbor_list+random+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);

                for(i=1;i<=n;i++)    
                {
                    TPM.search(this,perm[i-1],rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }


            }



            if(heuristics & TWO_OPT)
            {

                rules=VRPH_SIMULATED_ANNEALING+neighbor_list+random+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);

                for(i=1;i<=n;i++)    
                {
                    TO.search(this,perm[i-1],rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }


            }        

            if(heuristics & THREE_POINT_MOVE)
            {
                rules=VRPH_SIMULATED_ANNEALING+VRPH_INTER_ROUTE_ONLY+neighbor_list+random+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);


                for(i=1;i<=n;i++)    
                {
                    ThreePM.search(this,perm[i-1],rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }
            }

            if(heuristics & OR_OPT)
            {
                rules=VRPH_SIMULATED_ANNEALING+neighbor_list+random+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);

                for(i=1;i<=n;i++)    
                {
                    OR.search(this,perm[i-1],3,rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }

                for(i=1;i<=n;i++)    
                {
                    OR.search(this,perm[i-1],2,rules);
                    if(this->total_route_length > worst_obj)
                        worst_obj=this->total_route_length;
                }


            }

            if(heuristics & CROSS_EXCHANGE)
            {
                normalize_route_numbers();
                this->find_neighboring_routes();
                R=total_number_of_routes;
                rules=VRPH_SIMULATED_ANNEALING+fixed+objective;
                if(random)
                    random_permutation(perm, this->num_nodes);


                for(i=1; i<=R-1; i++)    
                {
                    for(j=0;j<=1;j++)
                    {
                        CE.route_search(this,i, route[i].neighboring_routes[j],rules);
                        if(this->total_route_length > worst_obj)
                            worst_obj=this->total_route_length;
                    }
                }
            }            
        }
    }

    delete [] perm;

    // Restore the best sol
    this->import_solution_buff(this->best_sol_buff);
    this->currentObject = this->bestObject;
    // Now return the obj. function value

    if(has_service_times==false)
        return this->best_total_route_length;
    else
        return this->best_total_route_length-total_service_time;

}