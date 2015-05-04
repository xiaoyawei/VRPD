////////////////////////////////////////////////////////////
//                                                        //
// This file is part of the VRPH software package for     //
// generating solutions to vehicle routing problems.      //
// VRPH was developed by Chris Groer (cgroer@gmail.com).  //
//                                                        //
// (c) Copyright 2010 Chris Groer.                        //
// All Rights Reserved.  VRPH is licensed under the       //
// Common Public License.  See LICENSE file for details.  //
//                                                        //
////////////////////////////////////////////////////////////

#include "VRPH.h"



bool Postsert::move(VRP *V, int u, int i)
{

    ///
    /// This function inserts node number u AFTER node i in 
    /// whatever route node i is currently in and modifies all
    /// relevant solution information.
    /// 

    int pre_i, post_i, pre_u, post_u;
    int start_u,end_u, start_i, end_i;
    int n, i_route, u_route;
    int new_i_end, new_i_start, new_u_start, new_u_end;
    VRPMove M;

    // Special cases first
    if(u==VRPH_DEPOT)
    {
        report_error("%s: Not allowed to move the depot\n",__FUNCTION__);

    }
    if(i==VRPH_DEPOT)
    {
        // Inserting immediately after the VRPH_DEPOT--ambiguous!
        report_error("%s: Not allowed to use postsert after VRPH_DEPOT!  Use presert instead!\n",__FUNCTION__);

    }

    if(i<=0||u<=0)
    {
        fprintf(stderr,"i=%d; u=%d\n",i,u);
        report_error("%s: Not allowed to use postsert with non-positive indices.\n",__FUNCTION__);

    }

    // First, evaluate the move


    if(evaluate(V,u,i, &M)==false)
        return false;    // the move is not allowed


    // Otherwise, the move is feasible so make it.

    // Update solution information
    V->update(&M);

    // Now modify ordering

    if(V->next_array[i]==u)
    {
        // Nothing to do
        return true;
    }

    n= V->num_nodes;
    i_route= V->route_num[i];
    u_route= V->route_num[u];

    // Now change the location-related arrays

    start_u= V->route[u_route].start;
    start_i= V->route[i_route].start;
    end_u= V->route[u_route].end;
    end_i= V->route[i_route].end;

    // pre_i is what used to be before i
    pre_i= V->pred_array[i];
    // post_i is what used to be after i
    post_i= V->next_array[i];
    // pre_u is what used to be before u
    pre_u= V->pred_array[u];
    // post_u is what used to be after u
    post_u= V->next_array[u];

    if(start_i==u)
        new_i_start=post_u;
    else
        new_i_start=start_i;

    if(start_u==u)
    {
        // post_u is the new u start
        new_u_start=post_u;
    }
    else
        new_u_start=start_u;

    if(end_u == u)
    {
        // pre_u must be the new end of u's route
        new_u_end=pre_u;
    }
    else
        new_u_end=end_u;

    if(end_i==i)
    {
        // u is the new end
        new_i_end= u;
    }
    else
        new_i_end=end_i;    

    // Special case
    if(post_i==-u)
    {
        // must have i as the last node in a route in this case and u the first
        // node in the next route
        // i and u cannot be in the same original route in this case

        //new_i_end=u;
        V->next_array[i]=u;
        V->pred_array[u]=i;

        // VRPH_ADDED
        V->next_array[u]=-VRPH_ABS(post_u);  
        // post_u is now the beginning of u's old route
        V->pred_array[VRPH_ABS(post_u)]=-u;

        // Update i_route information
        V->route_num[u]=i_route;
        V->route[i_route].end=new_i_end;
        V->route[i_route].start=new_i_start;

        // Now for the updates to u's former route
        if(start_u==u && end_u ==u)
            return true;

        if(start_u==u)
            // New start to u's old route since u used to be first
            new_u_start=post_u;
        if(end_u==u)
            // New end to u's old route since u used to be last
            new_u_end=pre_u;
        else
            new_u_end=end_u;
        // Otherwise no changes to start and end

        V->route[u_route].end=new_u_end;
        V->route[u_route].start=new_u_start;

        return true;
    }

    V->next_array[i] = u;
    V->next_array[u] = post_i;
    V->pred_array[u]=i;

    // We now need to make u's old predecessor pre_u point to post_u since
    // u is no longer there

    if(pre_u>0&&post_u>0)
    {
        V->next_array[VRPH_ABS(pre_u)]=post_u;
        V->pred_array[VRPH_ABS(post_u)]=pre_u;
    }
    else
    {
        // u was the first or last node in its route
        V->next_array[VRPH_ABS(pre_u)]=-VRPH_ABS(post_u);
        V->pred_array[VRPH_ABS(post_u)]=-VRPH_ABS(pre_u);

    }    

    // The element who used to be after u is now preceded by the element
    // that was before u

    // The element who used to be after i is now preceded by u 
    //if(post_i>=0)// CHANGED 8/11/2008
    if(post_i>0)
        V->pred_array[post_i]=u;
    else
        V->pred_array[-post_i]=-u;

    // Update i_route information
    V->route_num[u]=i_route;
    V->route[i_route].end = new_i_end;
    V->route[i_route].start=new_i_start;

    // Now update u's former route

    if(start_u==u && end_u ==u)
        return true;        
    

    if(u_route==i_route)
    {
        new_u_end=new_i_end;
        new_u_start=new_i_start;
        if(end_u==u)
            new_u_end=pre_u;
    }

    V->route[u_route].end=new_u_end;
    V->route[u_route].start=new_u_start;

    return true;
}




