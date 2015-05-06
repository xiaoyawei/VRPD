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
    const char *filename = "/Users/xiaoya/Documents/Google Drive/Spring 2015/BMGT831/VRPD/Instance/instance1.txt";
    Solver solver;
    solver.main(filename);
    return 0;
}
