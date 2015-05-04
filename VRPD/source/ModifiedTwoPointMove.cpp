//
//  ModifiedTwoPointMove.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 5/3/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include <stdio.h>
#include "VRPH.h"

bool TwoPointMove::search(class VRP *V, int j, int rules)
{
    ///
    /// Attempts to find the best Two-Point move involving node j using the specified
    /// search space, and rules.
    /// If an acceptable move is found, then the move is made and all relevant solution
    /// modifications are made.
    ///

    VRPMove M;
    VRPMove BestM;
    BestM.savings=VRP_INFINITY;
    int i,k;
    int accept_type;

    if(j==VRPH_DEPOT)        
        return false;

    if(rules & VRPH_FIXED_EDGES)
    {
        i=VRPH_MAX(V->pred_array[j],VRPH_DEPOT);
        k=VRPH_MAX(V->next_array[j],VRPH_DEPOT);

        // Make sure we aren't disturbing fixed edges

        if( V->fixed[i][j] || V->fixed[j][k] ) 
            return false;
    }

    accept_type=VRPH_FIRST_ACCEPT;//default

    if( (rules & VRPH_FIRST_ACCEPT) > 0)
        accept_type=VRPH_FIRST_ACCEPT;
    if( (rules & VRPH_BEST_ACCEPT) > 0)
        accept_type=VRPH_BEST_ACCEPT;
    if( (rules & VRPH_LI_ACCEPT) > 0)
        accept_type=VRPH_LI_ACCEPT;

    int *old_sol=NULL;
    if(rules & VRPH_TABU)
    {
        // Remember the original solution 
        old_sol=new int[V->num_original_nodes+2];
        V->export_solution_buff(old_sol);
    }

    // Create the search_space
    V->create_search_neighborhood(j, rules);

    for(i=0;i<V->search_size;i++)
    {
        k=V->search_space[i];

        if(k!=VRPH_DEPOT && k!=j)
        {
            // VRPH_DEPOT not allowed in TwoPointMove

            if(evaluate(V,j,k,rules,&M)==true)
            {
                // Feasible move found
                if(accept_type==VRPH_FIRST_ACCEPT || (accept_type==VRPH_LI_ACCEPT && M.savings<-VRPH_EPSILON) )
                {
                    // make the move

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

                        }
                    }
                }



                if(accept_type==VRPH_BEST_ACCEPT || accept_type==VRPH_LI_ACCEPT )
                {
                    // compare to best move so far
                    if(M.is_better(V, &BestM, rules))
                        BestM=M;

                }

            }
        }
    }



    if(accept_type==VRPH_FIRST_ACCEPT)
    {
        if(old_sol)
            delete [] old_sol;
        return false;
    }

    if(BestM.savings==VRP_INFINITY)
    {
        if(old_sol)
            delete [] old_sol;
        return false;
    }
    // else we found a move - make it

    if(move(V,&BestM)==true)
    {
        V->updateCurrentObject(V->getCurrentObject() + BestM.savings);
        if(!(rules & VRPH_TABU))
            return true;                    
    }

    if(rules & VRPH_TABU)
    {    
        // Check VRPH_TABU status of move - return true if its ok
        // or revert to old_sol if not and return
        if(V->check_tabu_status(&M, old_sol))
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


    report_error("%s: best move false!\n",__FUNCTION__);


    return false;
}