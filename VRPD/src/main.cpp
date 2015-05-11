//
//  main.cpp
//  VRPD
//
//  Created by Xiaoya Wei on 4/22/15.
//  Copyright (c) 2015 Xiaoya Wei. All rights reserved.
//

#include <iostream>
#include "VRPD.h"
#include "Solver.h"

int main(int argc, char * argv[]) {
    
    using std::string;
    Solver solver;
    cmdline::parser parser;
    parser.add<string>("filename", 'f', "The name of the input file", true, "");
    parser.add<string>("outfile", 'o', "The name of the output file", false, "output.txt");
    parser.add<int>("verboseLevel", 'v', "Display debug info at runtime: 0 - No display, 1 - Low, 2 - Medium, 3 - High", false, DefaultValue::verboseLevel, cmdline::range(0, 3));
    parser.add<double>("stop", 's', "Set the threshold to terminate when a solution better than stop is found", false, DefaultValue::stopObject);
    parser.add<double>("startTemp", 't', "The starting temperature for the simulated annealing algorithm", false, DefaultValue::startingTemp);
    parser.add<double>("coolRatio", 'c', "The cooling ratio for the simulated annealing algirhtm", false, DefaultValue::coolingRatio);
    parser.add<int>("numLoop", 'l', "The number of loops for simulated annealing to run", false, DefaultValue::numLoop);
    parser.add<int>("numIter", 'i', "The number of iterations for simulated annealing to run in every loop", false, DefaultValue::numIter);
    parser.add<int>("listSize", 'b', "The number of potential transforms simulated annleaing attempts at each iteration", false, DefaultValue::numListSize);
    parser.add<int>("heuristic", 'h', "The heuristics employed (sum): 4 - Two Opt, 2 - Two Point Move, 1 - One Point Move", false, DefaultValue::heuristic, cmdline::range(1, 7));
    parser.parse_check(argc, argv);
    
    solver.setParameters(parser);
    const char *filename = parser.get<string>("filename").c_str();
    const char *outfile = parser.get<string>("outfile").c_str();
    solver.main(filename, outfile);
    return 0;
}
