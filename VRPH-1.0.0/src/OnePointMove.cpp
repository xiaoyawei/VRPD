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

// SEARCH



bool OnePointMove::route_search(class VRP *V, int r1, int r2, int rules)
{
    ///
    /// Searches for a one point move where a node from route r1 is moved
    /// into route r2.
    ///

    // So search is now ROUTE_BASED - no neighbor_lists
    if( (rules & VRPH_USE_NEIGHBOR_LIST) > 0)
        report_error("%s: route_searches do not use neighbor list\n",__FUNCTION__);

    // Otherwise, we have a route search - look for all moves involving 
    // route r1 and route r2

    VRPMove M;
    VRPMove BestM;
    int k;
    double best_savings=VRP_INFINITY;
    int best_k=0;
    int accept_type;


    //default setting
    accept_type=VRPH_FIRST_ACCEPT;

    if( (rules & VRPH_FIRST_ACCEPT) > 0)
        accept_type=VRPH_FIRST_ACCEPT;
    if( (rules & VRPH_BEST_ACCEPT) > 0)
        accept_type=VRPH_BEST_ACCEPT;
    if( (rules & VRPH_LI_ACCEPT) > 0)
        accept_type=VRPH_LI_ACCEPT;
        

    int j;
    j= V->route[r1].start;
    while(j!=VRPH_DEPOT)
    {
        // Loop through r1 and r2
        k= V->route[r2].start;
        while(k!=VRPH_DEPOT)
        {
            if(evaluate(V,j,k,rules,&M)==true)
            {

                // Feasible move found
                if(accept_type==VRPH_FIRST_ACCEPT)
                {
                    // This is the first move found -- make it and return if VRPH_FIRST_ACCEPT
                    
                    if(move(V,&M)==true)
                        return true;
                    else
                        report_error("%s: move is false?\n",__FUNCTION__);
                                
                }

                if(accept_type==VRPH_BEST_ACCEPT)
                {
                    // VRPH_BEST_ACCEPT - store the move

                    if(M.savings<best_savings)
                    {
                        best_savings=M.savings;
                        best_k=k;
                        BestM=M;
                    }
                }

                if(accept_type==VRPH_LI_ACCEPT)
                {
                    // move if downhill, store otherwise.
                    
                    if(M.savings<-VRPH_EPSILON)
                    {
                        // it's downhill, so make it.
                        if(move(V,&M)==true)
                            return true;
                        else
                            report_error("%s: false 7\n",__FUNCTION__);
                    }

                    if(M.savings<best_savings)
                    {
                        best_savings=M.savings;
                        best_k=k;
                        BestM=M;
                    }
                }
            }
            k=VRPH_MAX(V->next_array[k],0);
        }
        j=VRPH_MAX(V->next_array[j],0);
    }

    if(accept_type==VRPH_FIRST_ACCEPT)
    {
        // No moves found
        return false;
    }

    // Otherwise we will make the best move found
    if(best_savings==VRP_INFINITY)
        return false;

    // else we found a move - make it
    if(move(V,&BestM)==true)
        return true;
    else
        report_error("%s: false 8\n",__FUNCTION__);
    
    return false;
    

}

// EVALUATE
bool OnePointMove::evaluate(class VRP *V, int j, int b, int rules, VRPMove *M)
{
    ///
    /// This function evaluates the move of inserting j either before or after node b
    /// and places the best savings found in the VRPMove struct M if the move is feasible
    /// and returns false if no feasible move is found, true otherwise.
    ///

    V->num_evaluations[ONE_POINT_MOVE_INDEX]++;

    int a,c,i,k;

    a = VRPH_MAX(V->pred_array[b],VRPH_DEPOT);
    c = VRPH_MAX(V->next_array[b],VRPH_DEPOT);
    k = VRPH_MAX(V->next_array[j],VRPH_DEPOT);
    i = VRPH_MAX(V->pred_array[j],VRPH_DEPOT);


    if(rules & VRPH_FIXED_EDGES)
    {        
        // Make sure we aren't disturbing fixed edges

        if( V->fixed[i][j]|| V->fixed[j][k] )
            return false;

        if(b!=VRPH_DEPOT &&  (V->fixed[b][c] && V->fixed[a][b]) )
            return false;
    }

    M->evaluated_savings=false;

    if(b==j || V->routed[j]==false || V->routed[b]==false || j==VRPH_DEPOT) 
        return false;

    if(b!=VRPH_DEPOT)
    {
        if( (rules & VRPH_INTER_ROUTE_ONLY) && (V->route_num[j] ==V->route_num[b]) )
            return false;

        if((rules & VRPH_INTRA_ROUTE_ONLY) && (V->route_num[j] !=V->route_num[b]) )
            return false;

        // Can quickly check veh capacity: j added to V->route_num[b]
        if(V->route_num[j] != V->route_num[b])
        {
            if( V->nodes[j].demand + V->route[V->route_num[b]].load > V->max_veh_capacity)
                return false;
        }


    }
    
    double savings1, savings2, best_savings;
    Postsert postsert;
    Presert presert;
    best_savings=VRP_INFINITY;
    savings1=VRP_INFINITY;
    savings2=VRP_INFINITY;

    // First consider the complicated case where b is the VRPH_DEPOT.
    // We have 2*r edges to consider where r is the # of routes
    // In this case we will find the best possible insertion
    if(b==VRPH_DEPOT)
    {
        int current_start, current_end, current_route;
        current_start=abs(V->next_array[VRPH_DEPOT]);
        bool allowed, found_move;
        VRPMove CurrentM;
        found_move=false;
        
        for(;;)
        {
            // Consider the edge VRPH_DEPOT-current_start
            int t=current_start;
            allowed=true;

            if(j!=t)
            {
                // Check for fixed edges
                if((rules & VRPH_FIXED_EDGES) && V->fixed[VRPH_DEPOT][t])
                    allowed=false;

                if( (presert.evaluate(V,j,t,&CurrentM)==true)&&(V->check_move(&CurrentM,rules)==true) && allowed )
                {
                    
                    if(CurrentM.is_better(V,M,rules))
                    {
                        found_move=true;
                        *M=CurrentM;
                    }
                }
            }

            // Now try the t-VRPH_DEPOT edge                
            current_route= V->route_num[current_start];
            current_end= V->route[current_route].end;
            t=current_end;
            allowed=true;
            if(j!=t)
            {
                // Check for fixed edges
                if((rules & VRPH_FIXED_EDGES) && V->fixed[t][VRPH_DEPOT])
                    allowed=false;

                if( (postsert.evaluate(V,j,t,&CurrentM)==true)&&(V->check_move(&CurrentM,rules)==true) && allowed )
                {
                    if(CurrentM.is_better(V,M,rules))
                    {
                        found_move=true;
                        *M=CurrentM;
                    }
                }
                
            }

            // Now advance to the next node adjacent to the VRPH_DEPOT
            current_start=abs(V->next_array[current_end]);
            if(current_start==VRPH_DEPOT)    // We're done
                break;
        }

        return found_move;

        
    }
    
    // Special Case
    if(c == j)
    {
        // Only option is to insert j between a and b (b is not VRPH_DEPOT)

        if(rules & VRPH_FIXED_EDGES)
        {
            // Make sure we aren't disturbing fixed edges

            if( V->fixed[a][b] )//|| V->fixed[b][c])
                return false;
        }
        
        if( (presert.evaluate(V,j,b,M)==true)&&(V->check_move(M,rules)==true) )
            return true;
        else
            return false;
        
    }


    if(a == j)
    {
        // Only option is to insert j between b and c.  b is not VRPH_DEPOT

        if(rules & VRPH_FIXED_EDGES)
        {
            // Make sure we aren't disturbing fixed edges

            if( V->fixed[b][c] )//|| V->fixed[a][b] ) 
                return false;
        }

        if( (postsert.evaluate(V,j,b,M)==true)&&(V->check_move(M,rules)==true) )
            return true;
        else
            return false;
        
    }
    
    // No conflicts o/w    - can insert j either before or after b 
    // We will choose the better move always!

    // savings = new-old
    savings1 = (V->d[a][j]+V->d[j][b]+V->d[i][k]) - (V->d[a][b]+V->d[i][j]+V->d[j][k])  ;
    savings2 = (V->d[i][k]+V->d[b][j]+V->d[j][c]) - (V->d[b][c]+V->d[i][j]+V->d[j][k])  ;

    best_savings = VRPH_MIN(savings1, savings2);

    
    if( savings1 <= savings2 &&  (presert.evaluate(V,j,b,M)==true)
        &&(V->check_move(M,rules)==true) )
    {

        if(rules & VRPH_FIXED_EDGES)
        {
            // Make sure we aren't disturbing fixed edges

            if( V->fixed[a][b] )//|| V->fixed[b][c] ) 
                return false;
        }
        return true;
    }
    
    // Now check savings2
    if( savings2<savings1 && postsert.evaluate(V,j,b,M)==true &&
        (V->check_move(M,rules)==true) )
    {

        if(rules & VRPH_FIXED_EDGES)
        {
            // Make sure we aren't disturbing fixed edges

            if( V->fixed[b][c] )//|| V->fixed[a][b] ) 
                return false;
        }

        return true;
    }

    // Need to check savings1 again in the event that savings2 was better, but the move
    // itself was infeasible--not sure if this ever happens...

    if( (presert.evaluate(V,j,b,M)==true)&&(V->check_move(M,rules)==true))
    {
        if(rules & VRPH_FIXED_EDGES)
        {
            // Make sure we aren't disturbing fixed edges

            if( V->fixed[a][b] )//|| V->fixed[b][c]) 
                return false;
        }

        return true;
    }
    

    
    // ELSE no feasible moves found
    return false;
}


