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

int main(int argc, const char * argv[]) {
    Solver solver;
    const char *filename;
    if (argc == 2) {
        filename = argv[1];
    } else if (argc != 3) {
        std::cout << "Command NOT FOUND!" << std::endl;
        exit(0);
    } else {
        if (!strcmp(argv[1], "-v")) {
            filename = argv[2];
            solver.setVerbose();
        } else if (!strcmp(argv[2], "-v")) {
            filename = argv[1];
            solver.setVerbose();
        } else {
            std::cout << "COMMMAND NOT FOUND" << std::endl;
        }
    }
//    const char *filename = "/Users/xiaoya/Documents/Google Drive/Spring 2015/BMGT831/VRPD/NewInstance/instance1.txt";
    solver.main(filename);
    return 0;
}
