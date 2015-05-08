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


bool Flip::move(VRP *V, int start_point, int end_point)
{
    ///
    /// This reverses the portion of the route between start_point and end_point
    /// if the proposed move is feasible.  Returns true and makes all relevant
    /// solution modifications if the move is made and false otherwise.
    ///

    int current, old_next, cnt;
    int start, end, i, route_num;
    VRPMove M;

    if(start_point<=VRPH_DEPOT || end_point<=VRPH_DEPOT )
        report_error("%s: flip::reverse_partial_route called with VRPH_DEPOT or negative index\n");

    if(start_point==end_point)
        report_error("%s: flip::reverse_partial_route called with start==end\n");

    route_num= V->route_num[start_point];

    if(route_num!= V->route_num[end_point])
        report_error("%s: flip::not in the same route\n");

    // evaluate the move
    if(evaluateHelper(V,start_point,end_point, &M)==false)
        return false;

    V->update(&M);

    // Now update the arrays

    i=0;

    start = start_point;
    end = end_point;
    // Assume the orientation is correct to avoid checking again!!

    // Special case!  next(start) = end
    if(VRPH_MAX(VRPH_DEPOT,V->next_array[start])==end)
    {
        int pre_start= V->pred_array[start];

        if(pre_start<0)
            report_error("%s: flip::pre_start <0 in special case!!\n");

        old_next= V->next_array[end];

        V->next_array[end]=start;
        V->pred_array[start]=end;

        V->next_array[start]=old_next;
        V->pred_array[old_next]=start;

        V->next_array[pre_start]=end;
        V->pred_array[end]=pre_start;

#if FLIP_VERIFY
        V->verify_routes("flip 1\n");
#endif

        return true;

    }

    // Otherwise we have only a single case to handle
    current= V->next_array[start];        //n1
    old_next= V->next_array[current];    //n2


    V->next_array[current]=end;        //next[n1]=end
    V->pred_array[end]=current;        //pred[end]=n1;
    V->pred_array[current]=old_next;    //pred[n1]=n2;
    current=old_next;                            //current=n2
    old_next= V->next_array[current];    //old_next=n3

    cnt=0;

    while(old_next != end)
    {

        V->next_array[current] = V->pred_array[current];
        V->pred_array[current]=old_next;
        current = old_next;
        old_next = V->next_array[current];
        cnt++;
        if(cnt>V->num_nodes)
            report_error("%s: flip::Impossible loop encountered\n");

    }
    V->next_array[current]= V->pred_array[current];
    V->pred_array[current]=start;
    V->next_array[start]=current;

    return true;
}
