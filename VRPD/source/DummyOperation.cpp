//
//  DummyOperation.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 5/3/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include <stdio.h>
#include "VRPH.h"

bool VRP::postsert_dummy(int i)
{
    // This function inserts dummy node after i

    if (dummyIndicator) {
        return false;
    }
    dummyIndicator = true;
    int pre_i, post_i, dummy;
    int start, end, start_i, end_i;
    int n, i_route;
    //double tu, uv, tv, iu, ju, ij, u_loss, i_gain, i_change, i_length, u_length;

    // Special cases first

    if(i<=VRPH_DEPOT || i>matrix_size)
        report_error("%s: input doesn't make sense\n",__FUNCTION__);

    n= num_nodes;
    dummy= dummy_index;

    i_route= route_num[i];

    start_i= route[i_route].start;
    end_i= route[i_route].end;

    start=start_i;
    if(end_i==i)
        // i is last in the route
        end=dummy;
    else 
        end=end_i;

    // pre_i is what used to be before i
    pre_i= pred_array[i];
    // post_i is what used to be after i
    post_i= next_array[i];

    next_array[i] = dummy;
    next_array[dummy] = post_i;

    // Now need to update pred_array as well!

    // u's predecessor is now i
    pred_array[dummy]=i;

    // The element who used to be after i is now preceded by u 
    if(post_i>=0)
        pred_array[post_i]=dummy;
    else
        // post_i 
        pred_array[-post_i]=-dummy;

    //start_array[dummy]=start;
    //end_array[dummy]=end;

    // Update i_route information
    route_num[dummy]=i_route;
    route[i_route].end=end;
    route[i_route].start=start;

    return true;
}

bool VRP::presert_dummy(int i)
{
    // This function inserts a dummy node BEFORE node i in 
    // whatever route node i is currently in

    if (dummyIndicator) {
        return false;
    }
    dummyIndicator = true;
    int pre_i, post_i;
    int start, end, start_i, end_i, dummy;
    int n, i_route;


    n= num_nodes ;
    dummy = dummy_index; // dummy node has index (n+1)

    if(i<=VRPH_DEPOT)
        report_error("%s: bad index\n",__FUNCTION__);

    i_route = route_num[i];

    start_i= route[i_route].start;
    end_i= route[i_route].end;


    // Get the start and end of i's route
    start=start_i;
    end=end_i;

    if(start==i)
        // i is first in the route, so dummy is now the first node
        start=dummy;

    // pre_i is what used to be before i
    pre_i= pred_array[i];
    // post_i is what used to be after i
    post_i= next_array[i];


    // dummy is now followed by i
    next_array[dummy]=i;
    pred_array[i]=dummy;
    pred_array[dummy]=pre_i;    

    // The element who used to be after i is now preceded by dummy
    if(pre_i>0)// was >=!!
        next_array[pre_i]=dummy;
    else
        // post_i 
        next_array[VRPH_ABS(pre_i)]=-dummy;

    // Update i_route information
    route_num[dummy]=i_route;
    route[i_route].end=end;
    route[i_route].start=start;

    // Add in the relevant Data fields for the dummy node!!

    return true;

}
bool VRP::remove_dummy()
{
    if (!dummyIndicator) {
        return false;
    }
    dummyIndicator = false;
    int pre_d, post_d, d_route, dummy, d_start, d_end, n;

    n= num_nodes;
    dummy= dummy_index;

    post_d= next_array[dummy];
    pre_d= pred_array[dummy];

    if(post_d > dummy || post_d < -dummy || pre_d > dummy || pre_d < -dummy)
    {
        fprintf(stderr,"post_d= %d; pre_d=%d\n",post_d,pre_d);
        report_error("%s: invalid indices\n",__FUNCTION__);
    }

    d_route= route_num[dummy];
    d_start= route[d_route].start;
    d_end= route[d_route].end;

    if(d_start==dummy)
    {
        if(post_d<0)
        {
            report_error("%s: post_d error in removal\n",__FUNCTION__);
        }

        route[d_route].start=post_d;
    }

    if(d_end==dummy)
    {
        if(pre_d<0)
        {
            report_error("%s: pre_d error in removal\n",__FUNCTION__);
        }
        route[d_route].end=pre_d;
    }

    next_array[VRPH_ABS(pre_d)]=post_d;

    if(d_start==dummy)
        next_array[VRPH_ABS(pre_d)]=-post_d;


    pred_array[VRPH_ABS(post_d)]=pre_d;

    if(d_end==dummy)
        pred_array[VRPH_ABS(post_d)]=-pre_d;


    return true;
}