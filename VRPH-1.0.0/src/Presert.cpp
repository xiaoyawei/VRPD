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


bool Presert::move(VRP *V, int u, int i)
{

    ///
    /// This function inserts node number u BEFORE node i in 
    /// whatever route node i is currently in if the move
    /// is feasible.
    ///

    int pre_i, post_i, pre_u, post_u;
    int start_u,end_u, start_i, end_i;
    int i_route, u_route;
    int new_u_start, new_u_end, new_i_start, new_i_end;
    VRPMove M;

    if(V->next_array[u]==i)
    {
        // nothing to do
        return true;
    }

    if(evaluate(V,u,i, &M)==false)
        return false;    //infeasible

#if PRESERT_VERIFY
    V->verify_routes("Before presert move\n");
#endif


    // Otherwise, the move is feasible, so make it

    
    // Update the solution information
    V->update(&M);

    // Now modify the ordering

    i_route= V->route_num[i];
    u_route= V->route_num[u];

    // pre_i is what used to be before i
    pre_i= V->pred_array[i];
    // post_i is what used to be after i
    post_i= V->next_array[i];
    // pre_u is what used to be before u
    pre_u= V->pred_array[u];
    // post_u is what used to be after u
    post_u= V->next_array[u];

        
    // Get the start and end of the original route.
    start_i= V->route[i_route].start;
    end_i= V->route[i_route].end;

    start_u= V->route[u_route].start;
    end_u= V->route[u_route].end;

    
    if(start_i==i)
        // u is now before i
        new_i_start=u;
    else
    {
        if(start_i==u)
            new_i_start=post_u;
        else
            new_i_start=start_i;
    }

    if(end_i == u)
        // pre_u is the new end of this route since u is moving
        new_i_end=pre_u;
    else
        new_i_end=end_i;

    if(start_u==u)
        // post_ is the new start since u is moving
        new_u_start=post_u;
    else
        new_u_start=start_u;

    if(end_u==u)
        new_u_end=pre_u;
    else
        new_u_end=end_u;
    
    
    
    // Special case
    
    if(pre_i==-u)
    {
        // We have    1-...t-v-u-1
        //            1-i-a-...
        // and we form the routes
        //            1-...-t-v-1
        //            1-u-i-a-...

        V->next_array[u]=i;
        V->pred_array[i]=u;

        V->next_array[VRPH_ABS(pre_u)]=-u;
        V->pred_array[u]=-VRPH_ABS(pre_u);

        // Update i_route information
        V->route_num[u]=i_route;
        V->route[i_route].end=new_i_end;
        V->route[i_route].start=new_i_start;

        // Now update u's former route

        // Make sure we didn't have VRPH_DEPOT-u-VRPH_DEPOT route
        if(start_u==end_u)
            return true;//??
        

        // Update u_route information
        V->route[u_route].end=new_u_end;
        V->route[u_route].start=new_u_start;

        return true;
    }

    // Added Special case - u is the first node in the route following i's route
    if(V->next_array[end_i] == -u)
    {
        V->next_array[end_i] = -VRPH_ABS(post_u);//temp1;
        V->pred_array[VRPH_ABS(post_u)] = -end_i;
        V->next_array[u]=i;
        V->pred_array[i]=u;
        V->pred_array[u]=pre_i;

        if(pre_i>0)    // was >=!!
            V->next_array[pre_i]=u;
        else
            // post_i 
            V->next_array[VRPH_ABS(pre_i)]=-u;

        // Update i_route information
        V->route_num[u]=i_route;
        V->route[i_route].end=new_i_end;
        V->route[i_route].start=new_i_start;

        
        // Check for VRPH_DEPOT-u-VRPH_DEPOT route
        if(start_u==end_u)
            return true;
        

        // start_u is now next[u] since u used to be at the start
        start_u= V->next_array[u];

        // Update u_route information
        V->route[u_route].end=new_u_end;
        V->route[u_route].start=new_u_start;

        return true;

    }

    // u is now followed by i
    V->next_array[u]=i;
    V->pred_array[i]=u;
    V->pred_array[u]=pre_i;
    // We now need to make u's old predecessor pre_u point to post_u since
    // u is no longer there
    
    // If u was the first node in its route, then post_u is now the first node
    // in the route
    if(pre_u<=0 || post_u<=0)
    {
        // u is first or last in its route
        V->next_array[VRPH_ABS(pre_u)]=-VRPH_ABS(post_u);
        V->pred_array[VRPH_ABS(post_u)]=-VRPH_ABS(pre_u);
    }
    else
    {
        V->next_array[pre_u]=post_u;
        V->pred_array[VRPH_ABS(post_u)]=pre_u;
    }
    // The element who used to be after u is now preceded by the element
    // that was before u
    
    // The element who used to be after i is now preceded by u 
    if(pre_i>0)// was >=!!
        V->next_array[pre_i]=u;
    else
        // post_i 
        V->next_array[VRPH_ABS(pre_i)]=-u;

    // Update i_route information
    V->route_num[u]=i_route;
    V->route[i_route].end=new_i_end;
    V->route[i_route].start=new_i_start;

    // Check for VRPH_DEPOT-u-VRPH_DEPOT route
    if(start_u==end_u)
        return true;
    

    // Update u_route information
    if(u==start_u)
    {
        // u was first in its route-->post_u is new start
        start_u=VRPH_ABS(post_u);
    }
    if(u==end_u)
    {
        // u was last in its rotue --> pre_u is new end
        end_u=VRPH_ABS(pre_u);
    }
    if(u_route == i_route)
    {
        new_u_end=new_i_end;
        new_u_start=new_i_start;
    }
    
    V->route[u_route].end=new_u_end;
    V->route[u_route].start=new_u_start;
    
    return true;

}


