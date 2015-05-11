//
//  ModifiedTwoOpt.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 5/4/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include <stdio.h>
#include "VRPH.h"

bool TwoOpt::search(class VRP *V, int b, int rules)
{
    ///
    /// Attempts to find the best Two-Opt move involving node b using the specified
    /// rules.
    /// If acceptable move is found, the move is made and all relevant solution
    /// modifications are made.
    ///

    VRPMove M;
    VRPMove BestM;
    int i,ii,j,k,a,c;
    int best_edges[4];
    int accept_type;

    memset(best_edges,-1,4*sizeof(int));
    accept_type=VRPH_FIRST_ACCEPT;    //default

    if( (rules & VRPH_FIRST_ACCEPT) == VRPH_FIRST_ACCEPT)
        accept_type=VRPH_FIRST_ACCEPT;
    if( (rules & VRPH_BEST_ACCEPT) == VRPH_BEST_ACCEPT )
        accept_type=VRPH_BEST_ACCEPT;
    if( (rules & VRPH_LI_ACCEPT) == VRPH_LI_ACCEPT )
        accept_type=VRPH_LI_ACCEPT;

    int *old_sol=NULL;
    if(rules & VRPH_TABU)
    {
        // Remember the original solution 
        old_sol=new int[V->num_original_nodes+2];
        V->export_solution_buff(old_sol);
    }

    // Create the search_space
    V->create_search_neighborhood(b, rules);

    BestM.savings=VRP_INFINITY;

    // Get the existing edges a-b and b-c
    a=VRPH_MAX(V->pred_array[b],0);
    c=VRPH_MAX(V->next_array[b],0);

    if(rules & VRPH_FIXED_EDGES)
    {
        // If both edges a-b and b-c are fixed, then no 2 opt moves are possible
        if(V->fixed[a][b] && V->fixed[b][c])
            return false;
    }

    for(ii=0; ii<V->search_size; ii++)
    {
        j=V->search_space[ii];
        if(V->routed[j]==false)
        {
            fprintf(stderr,"Unrouted node %d found in TO.search\n",j);
            report_error("%s: Error in routed array\n",__FUNCTION__);
        }

        if(j!=b && j!=VRPH_DEPOT)
        {
            i=VRPH_MAX(V->pred_array[j],0);
            k=VRPH_MAX(V->next_array[j],0);
            
            // Four edges:  a-b, b-c, i-j, j-k
            // Now evaluate the 4 moves

            M.savings=VRP_INFINITY ;

            if(evaluate(V,a,b,i,j,rules, &M)==true)
            {        
                
                if( ( (accept_type == VRPH_LI_ACCEPT) && ( M.savings<-VRPH_EPSILON )) ||
                    accept_type==VRPH_FIRST_ACCEPT )
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


                if( accept_type == VRPH_LI_ACCEPT || accept_type == VRPH_BEST_ACCEPT )
                {

                    if(M.is_better(V, &BestM, rules))
                    {
                        BestM=M;
                        best_edges[0]=a;best_edges[1]=b;best_edges[2]=i;best_edges[3]=j;
                    }
                }

            }
            

            if(evaluate(V,a,b,j,k,rules, &M)==true)
            {    
                if( ( (accept_type == VRPH_LI_ACCEPT) && ( M.savings<-VRPH_EPSILON )) ||
                    accept_type==VRPH_FIRST_ACCEPT )
                {

                    if(move(V, &M)==false)
                        report_error("%s: move error 2\n",__FUNCTION__);
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

                if( accept_type == VRPH_LI_ACCEPT || accept_type == VRPH_BEST_ACCEPT )
                {
                    if(M.is_better(V, &BestM, rules))
                    {
                        BestM=M;
                        best_edges[0]=a;best_edges[1]=b;best_edges[2]=j;best_edges[3]=k;
                    }
                }

            }
            

            if(evaluate(V,b,c,i,j,rules, &M)==true)
            {
                if( ( (accept_type == VRPH_LI_ACCEPT) && ( M.savings<-VRPH_EPSILON )) ||
                    accept_type==VRPH_FIRST_ACCEPT )
                {

                    if(move(V, &M)==false)
                        report_error("%s: move error 3\n",__FUNCTION__);
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

                if( accept_type == VRPH_LI_ACCEPT || accept_type == VRPH_BEST_ACCEPT )
                {
                    if(M.is_better(V, &BestM, rules))
                    {
                        BestM=M;
                        best_edges[0]=b;best_edges[1]=c;best_edges[2]=i;best_edges[3]=j;
                    }
                }

            }
            

            if(evaluate(V,b,c,j,k,rules, &M)==true)
            {

                if( ( (accept_type == VRPH_LI_ACCEPT) && ( M.savings<-VRPH_EPSILON )) ||
                    accept_type==VRPH_FIRST_ACCEPT )
                {
                    if(move(V, &M)==false)
                        report_error("%s: move error 4\n",__FUNCTION__);
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

                if( accept_type == VRPH_LI_ACCEPT || accept_type == VRPH_BEST_ACCEPT )
                {
                    if(M.is_better(V, &BestM, rules))
                    {
                        BestM=M;
                        best_edges[0]=b;best_edges[1]=c;best_edges[2]=j;best_edges[3]=k;
                    }
                }
            }

        }
        // end if j!=VRPH_DEPOT && j!=b


        if(j==VRPH_DEPOT && j!=b)
        {
            // In this case we have many edges to consider 
            // We will consider all edges of the form VRPH_DEPOT-t 
            // and t-VRPH_DEPOT            

            int current_start, current_end, current_route;
            current_start=abs(V->next_array[VRPH_DEPOT]);


            for(;;)
            {
                // Consider the edge VRPH_DEPOT-current_start
                int t=current_start;

                if(evaluate(V,a,b,VRPH_DEPOT,t,rules, &M)==true)
                {
                    if( ( (accept_type == VRPH_LI_ACCEPT) && ( M.savings<-VRPH_EPSILON )) ||
                        accept_type==VRPH_FIRST_ACCEPT )
                    {
                        if(move(V, &M)==false)
                            report_error("%s: VRPH_DEPOT move error 1\n",__FUNCTION__);
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

                    if( accept_type == VRPH_LI_ACCEPT || accept_type == VRPH_BEST_ACCEPT )
                    {

                        if(M.is_better(V, &BestM, rules))
                        {
                            BestM=M;
                            best_edges[0]=a;best_edges[1]=b;best_edges[2]=VRPH_DEPOT;best_edges[3]=t;
                        }

                    }    
                }

                if(evaluate(V,b,c,VRPH_DEPOT,t,rules, &M)==true)
                {
                    if( ( (accept_type == VRPH_LI_ACCEPT) && ( M.savings<-VRPH_EPSILON )) ||
                        accept_type==VRPH_FIRST_ACCEPT )
                    {
                        if(move(V, &M)==false)
                            report_error("%s: VRPH_DEPOT move error 2\n",__FUNCTION__);
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

                    if( accept_type == VRPH_LI_ACCEPT || accept_type == VRPH_BEST_ACCEPT )
                    {

                        if(M.is_better(V, &BestM, rules))
                        {
                            BestM=M;
                            best_edges[0]=b;best_edges[1]=c;best_edges[2]=VRPH_DEPOT;best_edges[3]=t;
                        }

                    }    
                }

                // Now try the t-VRPH_DEPOT edge                
                current_route= V->route_num[current_start];
                current_end= V->route[current_route].end;
                t=current_end;

                if(evaluate(V,a,b,t,VRPH_DEPOT,rules, &M)==true)
                {
                    if( ( (accept_type == VRPH_LI_ACCEPT) && ( M.savings<-VRPH_EPSILON )) ||
                        accept_type==VRPH_FIRST_ACCEPT )
                    {
                        if(move(V, &M)==false)
                            report_error("%s: VRPH_DEPOT move error 3\n",__FUNCTION__);
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

                    if( accept_type == VRPH_LI_ACCEPT || accept_type == VRPH_BEST_ACCEPT )
                    {

                        if(M.is_better(V, &BestM, rules))
                        {
                            BestM=M;
                            best_edges[0]=a;best_edges[1]=b;best_edges[2]=t;best_edges[3]=VRPH_DEPOT;
                        }

                    }
                }

                if(evaluate(V,b,c,t,VRPH_DEPOT,rules, &M)==true)
                {
                    if( ( (accept_type == VRPH_LI_ACCEPT) && ( M.savings<-VRPH_EPSILON )) ||
                        accept_type==VRPH_FIRST_ACCEPT )
                    {
                        if(move(V, &M)==false)
                            report_error("%s: VRPH_DEPOT move error 4\n",__FUNCTION__);
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

                    if( accept_type == VRPH_LI_ACCEPT || accept_type == VRPH_BEST_ACCEPT )
                    {

                        if(M.is_better(V, &BestM, rules))
                        {
                            BestM=M;
                            best_edges[0]=b;best_edges[1]=c;best_edges[2]=t;best_edges[3]=VRPH_DEPOT;
                        }

                    }
                }

                // Now advance to the next node adjacent to the VRPH_DEPOT
                current_start=abs(V->next_array[current_end]);
                if(current_start==VRPH_DEPOT)    // We're done
                    break;
            }
            
            // end VRPH_DEPOT loop
            
        }
        // end j loop
    }
    // end ii loop

    
    if(accept_type==VRPH_FIRST_ACCEPT || BestM.savings==VRP_INFINITY)
    {
        if(rules&VRPH_TABU)
            delete [] old_sol;
        return false;        // No moves found
    }

    

    if(accept_type==VRPH_BEST_ACCEPT || accept_type==VRPH_LI_ACCEPT)
    {
        
        if(move(V,&BestM)==false)
        {
            fprintf(stderr,"%f:  (%d[%d]-%d[%d], %d[%d]-%d[%d])\n",BestM.savings,
                best_edges[0],
                V->route_num[best_edges[0]],
                best_edges[1],V->route_num[best_edges[1]],
                best_edges[2],V->route_num[best_edges[2]],
                best_edges[3],V->route_num[best_edges[3]]);
            V->show_routes();
            report_error("%s: best move evaluates to false\n",__FUNCTION__);
        }
        else
        {
            if(!(rules & VRPH_TABU)){
//                V->updateCurrentObject(V->currentObject + BestM.savings);
                return true;
            }
            else
            {
                // Check VRPH_TABU status of move - return true if its ok
                // or revert to old_sol if not and continue to search.
                if(V->check_tabu_status(&BestM, old_sol))
                {
                    delete [] old_sol;
                    return true; // The move was ok
                }
                // else we reverted back - search over
                delete [] old_sol;
                return false;

            }
        }
    }
    
    report_error("%s: search shouldn't get here!\n",__FUNCTION__);

    return false;

}

bool TwoOpt::move(class VRP *V, VRPMove *M)
{
    ///
    /// Makes the actual solution modification implied by the Two-Opt
    /// move involving edges a-b and c-d.  Handles both intraroute and interroute
    /// moves
    ///
    
//    double oldObject = V->getCurrentObject();
    
//    if (oldObject < 40) {
//        int tmp = 0;
//    }
//    
    V->updateCurrentObject(V->getCurrentObject() + M->savings);
//    
//    if (V->getCurrentObject() < 50) {
//        int tmp = 0;
//    }
    
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