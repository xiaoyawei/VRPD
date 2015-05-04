//
//  ModifiedSwap.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 5/3/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include <stdio.h>
#include "VRPH.h"

bool Swap::evaluate(class VRP *V, int u, int i, VRPMove *M)
{
    ///
    /// Evaluates the move of swapping the positions of nodes u and i in 
    /// the current   For example, 
    /// Current situation:    t-u-v and h-i-j 
    /// New situation:        t-i-v and h-u-k        
    ///

    if(V->routed[u]==false || V->routed[i]==false)
        return false;

    if(u==VRPH_DEPOT || i==VRPH_DEPOT)
        report_error("%s: called with VRPH_DEPOT node\n",__FUNCTION__);

    if(u==i)
    {
        fprintf(stderr,"SWAP::%d=%d??\n",u,i);
        V->show_route(V->route_num[u]);
        report_error("%s: called with u=i\n",__FUNCTION__);
    }

    int t,v,h,j,u_route,i_route,swap_type=0;
    double savings,u_change=0,i_change=0;

    t=VRPH_MAX(V->pred_array[u],0);
    v=VRPH_MAX(V->next_array[u],0);
    h=VRPH_MAX(V->pred_array[i],0);
    j=VRPH_MAX(V->next_array[i],0);

    // Don't bother with this move if we have u or i as a singleton route
    if( (h==VRPH_DEPOT && j==VRPH_DEPOT) || (t==VRPH_DEPOT && v==VRPH_DEPOT) )
        return false;

    savings=VRP_INFINITY;

    //savings=new-old 
    if(h==u)
    {
        // t-h/u-i/v-j --> t-i/v-h/u-j
        savings=(V->d[t][i]+V->d[i][h]+V->d[u][j])-
            (V->d[t][u]+V->d[u][v]+V->d[i][j]);
        swap_type=1;
    }
    
    if(h==v)
    {
        savings=(V->d[t][i]+V->d[i][h]+V->d[v][u]+V->d[u][j])-
            (V->d[t][u]+V->d[u][v]+V->d[h][i]+V->d[i][j]);
        swap_type=2;
    }

    if(j==t && t!=VRPH_DEPOT)//!!!
    {
        savings=(V->d[h][u]+V->d[u][t]+V->d[j][i]+V->d[i][v])-
            (V->d[h][i]+V->d[i][j]+V->d[t][u]+V->d[u][v]);
        swap_type=3;
    }

    if(j==u)
    {
        savings=(V->d[h][j]+V->d[j][i]+V->d[i][v])-
            (V->d[h][i]+V->d[i][j]+V->d[u][v]);
        swap_type=4;
    }

    if(swap_type==0)
    {
        // Normal case w/ no exceptions
        savings=(V->d[t][i]+V->d[i][v] + V->d[h][u]+V->d[u][j])-
            (V->d[t][u]+V->d[u][v] + V->d[h][i]+V->d[i][j]);

        i_change=(V->d[h][u]+V->d[u][j])-(V->d[h][i]+V->d[i][j]);
        
        // Route u was t-u-v and is now t-i-v
        u_change=(V->d[t][i]+V->d[i][v])-(V->d[t][u]+V->d[u][v]);

        swap_type=5;
    }

    // Now need to check feasibility

    u_route= V->route_num[u];
    i_route= V->route_num[i];

    if(u_route==i_route)
    {
        // Same route, so the length changes by savings
        if(V->route[u_route].length+savings>V->max_route_length)
            return false;    // infeasible

        // Otherwise record the move

        M->num_affected_routes=1;
        M->savings=savings;
        M->route_nums[0]=u_route;
        M->route_lens[0]=V->route[u_route].length+savings;
        M->route_loads[0]=V->route[u_route].load;
        M->route_custs[0]= V->route[u_route].num_customers; // no change
        M->new_total_route_length= V->total_route_length+savings;
        M->total_number_of_routes = V->total_number_of_routes;
        M->move_type=SWAP;
        M->num_arguments=2;
        M->move_arguments[0]=u;
        M->move_arguments[1]=i;
    }
    else
    {
        // Different routes--but in this case we can't have any of the 
        // overlap conditions unless the VRPH_DEPOT is involved!!

        
        M->num_affected_routes=2;
        M->route_nums[0] = u_route;
        M->route_nums[1] = i_route;
        M->move_type=SWAP;
        M->num_arguments=2;
        M->move_arguments[0]=u;
        M->move_arguments[1]=i;

        if(swap_type==1)
        {
            report_error("%s: should not be in different routes(1)\n",__FUNCTION__);
            
        }

        if(swap_type==2)
        {
            //h=v:  must have v=h=VRPH_DEPOT
            u_change=(V->d[t][i]+V->d[v][i])-(V->d[t][u]+V->d[u][v]);
            i_change=(V->d[h][u]+V->d[u][j])-(V->d[h][i]+V->d[i][j]);

        }

        if(swap_type==3)
        {
            //j=t=VRPH_DEPOT
            u_change=(V->d[t][i]+V->d[v][i])-(V->d[t][u]+V->d[u][v]);
            i_change=(V->d[h][u]+V->d[u][j])-(V->d[h][i]+V->d[i][j]);

        }

        if(swap_type==4)
        {
            
            //j==u- not possible
            report_error("%s: should not be in different routes(4)\n",__FUNCTION__);
            

        }

        if(swap_type==5)
        {
            // Route i was h-i-j and is now h-u-j
            i_change=(V->d[h][u]+V->d[u][j])-(V->d[h][i]+V->d[i][j]);
            // Route u was t-u-v and is now t-i-v
            u_change=(V->d[t][i]+V->d[i][v])-(V->d[t][u]+V->d[u][v]);

        }

        if( ( M->route_lens[1] = V->route[i_route].length+i_change ) >V->max_route_length  )
            return false;    // route that used to contain i is infeasible


        if( ( M->route_lens[0] = V->route[u_route].length+u_change ) >V->max_route_length  )
            return false;    // route that used to contain u is infeasible

        // Now check capacity constraints

        if( (M->route_loads[1] = V->route[i_route].load+V->nodes[u].demand-
            V->nodes[i].demand ) >  V->max_veh_capacity)
            return false;    // route that used to contain i is infeasible

        if( ( M->route_loads[0]= V->route[u_route].load+V->nodes[i].demand-
            V->nodes[u].demand ) >  V->max_veh_capacity)
            return false;    // route that used to contain u is infeasible

    }
    // The move is feasible - store the rest and check 
    M->savings=savings;
    M->new_total_route_length=V->total_route_length+savings;
    M->total_number_of_routes=V->total_number_of_routes;
    M->route_custs[0]=V->route[u_route].num_customers;
    M->route_custs[1]=V->route[i_route].num_customers;
    
    // This is the modified calculation of M.savings
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