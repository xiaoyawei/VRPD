//
//  ModifiedOnePointMove.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 5/3/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include <stdio.h>
#include "VRPH.h"

bool OnePointMove::search(class VRP *V, int j, int rules)
{
    ///
    /// Attempts to find an appropriate one point move involving node j using the specified
    /// rules.     If acceptable move is found, the move is made and function returns
    /// true.  Returns false if no move is found.
    ///
    
    VRPMove M;
    VRPMove BestM;
    M.savings=M.new_total_route_length=BestM.savings=BestM.new_total_route_length=VRP_INFINITY;

    int i,k;
    int best_k=0;
    int accept_type;

    i=VRPH_MAX(V->pred_array[j],VRPH_DEPOT);
    k=VRPH_MAX(V->next_array[j],VRPH_DEPOT);

    // Return false if there are two or fewer nodes in i's route
    if(V->route[V->route_num[j]].num_customers<=3)
        return false;

    if( (rules & VRPH_FIXED_EDGES)  )
    {
        // Make sure we aren't disturbing fixed edges
        if( (V->fixed[i][j]) || (V->fixed[j][k]) )
            return false;

    }    

    // Determine acceptance type
    //default setting
    accept_type=VRPH_FIRST_ACCEPT;

    if( rules & VRPH_FIRST_ACCEPT )
        accept_type=VRPH_FIRST_ACCEPT;
    if( rules & VRPH_BEST_ACCEPT )
        accept_type=VRPH_BEST_ACCEPT;
    if( rules & VRPH_LI_ACCEPT )
        accept_type=VRPH_LI_ACCEPT;

    // Create the search_space
    V->create_search_neighborhood(j, rules);    

    int *old_sol=NULL;
    if(rules & VRPH_TABU)
    {
        // Remember the original solution 
        old_sol=new int[V->num_original_nodes+2];
        V->export_solution_buff(old_sol);
    }
        
    for(i=0;i<V->search_size;i++)
    {            
        k=V->search_space[i];
        if(evaluate(V,j,k,rules,&M)==true)
        {
            // Feasible move found
            if(accept_type==VRPH_FIRST_ACCEPT || (accept_type==VRPH_LI_ACCEPT && M.savings<-VRPH_EPSILON) )
            {
                if(move(V, &M)==false)
                    report_error("%s: move error 1\n",__FUNCTION__);
                else
                {
                    if(!(rules & VRPH_TABU))
                        return true;
                    else
                    {
                        // Check VRPH_TABU status of move - return true if its ok
                        // or revert to old_sol if not and continue to search.
                        if(V->check_tabu_status(&M, old_sol))
                        {
                            delete [] old_sol;
                            return true; // The move was ok
                        }
                        // else we reverted back - continue the search for a move
                    }
                }
            }

            if(accept_type==VRPH_BEST_ACCEPT || accept_type==VRPH_LI_ACCEPT)
            {
                // store the move

                if(M.is_better(V, &BestM, rules))
                {
                    best_k=k;
                    BestM=M;
                }
            }                
        }
    }        


    // We've considered all the possibilities now...
    if(accept_type==VRPH_FIRST_ACCEPT || BestM.savings==VRP_INFINITY)
    {
        // No moves found
        if(old_sol)
            delete [] old_sol;
        return false;
    }

    // else we found a move - try to make it


    if(move(V,&BestM)==true)
    {
        // Update the object
//        V->updateCurrentObject(V->getCurrentObject() + BestM.savings);
        if(!(rules & VRPH_TABU))
            return true;                    
    }
    
    if(rules & VRPH_TABU)
    {    
        // Check VRPH_TABU status of move - return true if its ok
        // or revert to old_sol if not and return
        if(V->check_tabu_status(&BestM, old_sol))// was &M??
        {
            delete [] old_sol;
            return true; // The move was ok
        }
        else
        {
            delete [] old_sol;
            return false;
        }
    }
        
    // Should have already returned
    report_error("%s: move error 4\n",__FUNCTION__);    

    return false;
}

bool OnePointMove::move(class VRP *V, VRPMove *M)
{
    ///
    /// Makes the one point move determined by the VRPMove M. 
    ///        
    
    V->updateCurrentObject(V->getCurrentObject() + M->savings);
    
    if(M->move_type==PRESERT)
    {
        Presert presert;
        if(presert.move(V,M->move_arguments[0],M->move_arguments[1]))
        {
            V->num_moves[ONE_POINT_MOVE_INDEX]++;
            V->capture_best_solution();

            return true;
        }
        else
            report_error("%s: presert move is false\n",__FUNCTION__);


    }
    else
    {
        if(M->move_type==POSTSERT)
        {
            Postsert postsert;
            if(postsert.move(V,M->move_arguments[0],M->move_arguments[1]))
            {
                V->num_moves[ONE_POINT_MOVE_INDEX]++;
                V->capture_best_solution();
                return true;
            }
            else
                report_error("%s: postsert move is false\n",__FUNCTION__);
        }
    }
    
    return false;
    
}