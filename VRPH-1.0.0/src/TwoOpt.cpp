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


bool TwoOpt::route_search(class VRP *V, int r1, int r2, int rules)
{

    ///
    /// Searches for all two opt moves involving an edge from r1 and
    /// an edge from r2.
    ///


    VRPMove M, BestM;
    int i,j,a,b;
    BestM.savings=VRP_INFINITY;
    int accept_type;

    if( (rules & VRPH_USE_NEIGHBOR_LIST) > 0)
        report_error("%s: route searches do not use neighbor_list\n",__FUNCTION__);

    accept_type=VRPH_FIRST_ACCEPT;    //default

    if( (rules & VRPH_FIRST_ACCEPT) > 0)
        accept_type=VRPH_FIRST_ACCEPT;
    if( (rules & VRPH_BEST_ACCEPT) > 0)
        accept_type=VRPH_BEST_ACCEPT;
    if( (rules & VRPH_LI_ACCEPT) > 0)
        accept_type=VRPH_LI_ACCEPT;

    double current_savings=VRP_INFINITY;


    a=VRPH_DEPOT;
    b= V->route[r1].start;

    for(;;)
    {
        // Get the edge a-b from route r1

        // Now get edges from route r2;

        j= V->route[r2].start;
        i=VRPH_DEPOT;
        for(;;)
        {

            // We now have the edges: a-b, b-c in route r1
            //                        i-j, j-k in route r2
            // Find the best of these 4 2 opt moves
            current_savings=VRP_INFINITY;


            // a-b, i-j
            if(evaluate(V,a,b,i,j,rules, &M)==true)
            {    
                if(accept_type==VRPH_FIRST_ACCEPT || (accept_type==VRPH_LI_ACCEPT && M.savings<-VRPH_EPSILON) )
                {
                    if(move(V,&M)==false)
                        report_error("%s: first accept move is false!\n",__FUNCTION__);
                    else
                        return true;

                }                

                if(accept_type == VRPH_LI_ACCEPT || accept_type==VRPH_BEST_ACCEPT)
                {
                    // See if it's the best so far

                    if(M.is_better(V, &BestM, rules))
                        BestM=M;
                }
            }

            // Advance route r2 node
            i=j;
            if(i==VRPH_DEPOT)
                break;
            j=VRPH_MAX(V->next_array[j],0);
        }
        // Advance route r1 node
        a=b;
        if(a==VRPH_DEPOT)
            break;
        b=VRPH_MAX(V->next_array[b],0);
    }

    if(accept_type == VRPH_FIRST_ACCEPT)
        return false;

    if(BestM.savings==VRP_INFINITY)
            return false;    // No move found

    if( (accept_type == VRPH_LI_ACCEPT) || (accept_type==VRPH_BEST_ACCEPT))
    {
        // Make the best move

        if(move(V, &BestM)==false)
            report_error("%s: best move is false\n",__FUNCTION__);            
        else
            return true;
    }

    return false;
}

bool TwoOpt::evaluate(class VRP *V, int a, int b, int c, int d, int rules, VRPMove *M)
{
    ///
    /// Considers the Two-Opt move involving the edges a-b and c-d and the
    /// provided rules.  If the move meets the rules, then
    /// the relevant changes to the solution are stored in the VRPMove M and
    /// the function returns true.  Returns false otherwise.
    /// 
    ///

    V->num_evaluations[TWO_OPT_INDEX]++;

    // Look for overlaps
    if(  (b==c) || (a==c) || (a==d) || (b==d) || V->routed[a]==false ||
        V->routed[b]==false || V->routed[c]==false || V->routed[d]==false )
    {
        // Two Opt move doesn't make sense here...
        return false;
    }

    // Check for fixed edges
    if(rules & VRPH_FIXED_EDGES)
    {
        if(V->fixed[a][b] || V->fixed[c][d])
            return false;
    }

    // Look for two edges involving the VRPH_DEPOT
    int num_depot_edges=0;
    if(a==VRPH_DEPOT) num_depot_edges++;
    if(b==VRPH_DEPOT) num_depot_edges++;
    if(c==VRPH_DEPOT) num_depot_edges++;
    if(d==VRPH_DEPOT) num_depot_edges++;
    if(num_depot_edges>1)
        return false;    

    M->eval_arguments[0]=a;M->eval_arguments[1]=b;M->eval_arguments[2]=c;M->eval_arguments[3]=d;
    M->evaluated_savings=false;

    int a_route, c_route;

    if(a!=VRPH_DEPOT)
        a_route= V->route_num[a];
    else
        a_route= V->route_num[b];

    if(c!=VRPH_DEPOT)
        c_route= V->route_num[c];
    else
        c_route= V->route_num[d];

    // Check for INTER/INTRA restrictions
    
    if( ( rules & VRPH_INTER_ROUTE_ONLY) && (a_route==c_route) )
        return false;

    if( ( rules & VRPH_INTRA_ROUTE_ONLY) && (a_route!=c_route) )
        return false;

#if 1
    // First, we can see if any of the possible moves passes the savings rules
    // before going into more expensive evaluations.

    if(a_route==c_route)
    {
        M->num_affected_routes=1;
        M->savings = ( V->d[a][c]+V->d[b][d] -V->nodes[c].service_time) - 
            ( V->d[a][b]+V->d[c][d]-V->nodes[b].service_time );

        // Can check feasibility
        if(M->savings+V->route[a_route].length > V->max_route_length)
            return false;

    }
    else
    {
        M->num_affected_routes=2;
        M->savings = (V->d[a][d]+V->d[c][b]) - (V->d[a][b]+V->d[c][d]) ;

        // Observation:  if savings>0, then one route must increase its length
        // by at least savings/2
        // So if both increase their lengths by savings/2 and are both infeasible
        // then we can say for certain that the move is infeasible.

        if(M->savings/2+V->route[a_route].length> V->max_route_length &&
            M->savings/2+V->route[c_route].length> V->max_route_length)
            return false;    
    }

    M->evaluated_savings=false;
    if( (V->check_savings(M, rules))==false)
        return false;    


#endif

    // Otherwise, the move is reasonable to consider further...

    
    if(a_route==c_route)
    {

        // same route-- the 2opt move here corresponds to reversing the portion of
        // this route that is between a and c
        Flip flip;    

        // First the easy case - VRPH_DEPOT not involved

        if(a!=VRPH_DEPOT && b!=VRPH_DEPOT && c!=VRPH_DEPOT && d!=VRPH_DEPOT)
        {

            if(V->before(a,c)==true)
            {
                
                    
                if( (flip.evaluate(V,a,d,M)==true)&&(V->check_move(M, rules)==true))
                {
                    
                    return true;
                }
                else
                {
                    
                    return false;
                }
            }
            else
            {
                // a is after c
                if( (flip.evaluate(V,c,b,M)==true) &&(V->check_move(M, rules)==true))
                {
                    
                    return true;
                }
                else
                {
                
                    return false;
                }
                
            }
        }
        else // VRPH_DEPOT involved
        {    
            // Otherwise, we have a VRPH_DEPOT node 
            // We'll need to use the dummy node 
            int dummy= V->dummy_index;

            if(a==VRPH_DEPOT)
            {
                // Put a dummy node between a and b and evaluate
                V->presert_dummy(b);
                if( (flip.evaluate(V,dummy,d,M)==true)&&(V->check_move(M, rules)==true))
                {
                    V->remove_dummy();
                    return true;
                }
                else
                {
                    V->remove_dummy();
                    return false;
                }
            }

            if(b==VRPH_DEPOT)
            {
                V->postsert_dummy(a);
                if( (flip.evaluate(V,c,dummy,M)==true)&&(V->check_move(M, rules)==true))
                {
                    V->remove_dummy();
                    return true;
                }
                else
                {
                    V->remove_dummy();
                    return false;
                }
            }

            if(c==VRPH_DEPOT)
            {

                V->presert_dummy(d);
                if( (flip.evaluate(V,dummy,b,M)==true)&&(V->check_move(M, rules)==true))
                {
                    V->remove_dummy();
                    return true;
                }
                else
                {
                    V->remove_dummy();
                    return false;
                }
            }

            if(d==VRPH_DEPOT)
            {

                V->postsert_dummy(c);

                if( (flip.evaluate(V,a,dummy,M)==true)&&(V->check_move(M, rules)==true))
                {
                    V->remove_dummy();
                    return true;
                }
                else
                {
                    V->remove_dummy();
                    return false;
                }
            }
        }
    }

    // else the edges are in different routes

    SwapEnds swap_ends;

    // First the easy case - VRPH_DEPOT not involved

    if(a!=VRPH_DEPOT && b!=VRPH_DEPOT && c!=VRPH_DEPOT && d!=VRPH_DEPOT)
    {
        if( (swap_ends.evaluate(V,a,c,M)==true)&&(V->check_move(M, rules)==true))
            return true;
        else
            return false;
    }
    // else we have some VRPH_DEPOT nodes to deal with

    int dummy= V->dummy_index;

    // Case 2:  a is the VRPH_DEPOT
    if(a==VRPH_DEPOT && b!=VRPH_DEPOT && c!=VRPH_DEPOT && d!=VRPH_DEPOT)
    {
        V->presert_dummy(b);
        if( (swap_ends.evaluate(V,dummy,c,M)==true)&&(V->check_move(M, rules)==true))
        {
            V->remove_dummy();
            return true;
        }
        else
        {
            V->remove_dummy();
            return false;
        }



    }

    // Case 3:  d is the VRPH_DEPOT
    if(d==VRPH_DEPOT && b!=VRPH_DEPOT && c!=VRPH_DEPOT && a!=VRPH_DEPOT)
    {
        V->postsert_dummy(c);
        if( (swap_ends.evaluate(V,a,c,M)==true)&&(V->check_move(M, rules)==true))
        {
            V->remove_dummy();
            return true;
        }
        else
        {
            V->remove_dummy();
            return false;
        }


    }

    // Case 4:  c is the VRPH_DEPOT
    if(a!=VRPH_DEPOT && b!=VRPH_DEPOT && c==VRPH_DEPOT && d!=VRPH_DEPOT)
    {

        V->presert_dummy(d);
        if( (swap_ends.evaluate(V,dummy,a,M)==true)&&(V->check_move(M, rules)==true))
        {
            V->remove_dummy();
            return true;
        }
        else
        {
            V->remove_dummy();
            return false;
        }
    }

    // Case 5:  b is the VRPH_DEPOT
    if(a!=VRPH_DEPOT && b==VRPH_DEPOT && c!=VRPH_DEPOT && d!=VRPH_DEPOT)
    {

        V->postsert_dummy(a);
        if( (swap_ends.evaluate(V,c,a,M)==true)&&(V->check_move(M, rules)==true))
        {            
            V->remove_dummy();
            return true;
        }
        else
        {
            V->remove_dummy();
            return false;
        }

    }

    // Should never get here!

    report_error("%s: should never get here!\n",__FUNCTION__);
    return false;
    
}

bool TwoOpt::move(class VRP *V, VRPMove *M)
{
    ///
    /// Makes the actual solution modification implied by the Two-Opt
    /// move involving edges a-b and c-d.  Handles both intraroute and interroute
    /// moves
    ///
    int a,b,c,d;

    a=M->eval_arguments[0];b=M->eval_arguments[1];c=M->eval_arguments[2];d=M->eval_arguments[3];

    V->num_moves[TWO_OPT_INDEX]++;
    
    if(M->move_type==FLIP && M->move_type==SWAP_ENDS)
        report_error("%s: unknown move type\n",__FUNCTION__);

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

    if(M->move_type==FLIP)
    {
        Flip flip;

        if(flip.move(V, M->move_arguments[0], M->move_arguments[1])==true)
        {
            
            if(uses_dummy)
                V->remove_dummy();

            V->capture_best_solution();
            return true;
        }
        else
        {
            fprintf(stderr,"flip.move(%d,%d) is false\n",M->move_arguments[0],M->move_arguments[1]);
            report_error("%s: Flip error 1\n",__FUNCTION__);
        }
    }

    // SWAP_ENDS otherwise
    SwapEnds swap_ends;

    if(swap_ends.move(V,M->move_arguments[0],M->move_arguments[1])==true)
    {
        if(uses_dummy)
            V->remove_dummy();
        
        V->capture_best_solution();

        return true;
    }
    else
        report_error("%s: SwapEnds error 1\n",__FUNCTION__);


    return false;

}

