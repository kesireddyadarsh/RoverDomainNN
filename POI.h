//
//  POI.h
//  roverDomainNN
//
//  Created by ak on 9/26/16.
//  Copyright © 2016 ak. All rights reserved.
//

#ifndef POI_h
#define POI_h
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cassert>
#include <time.h>
#include <stdlib.h>
#include "environment.h"
#include "POI.h"
#include "evolutionAlgorithm.h"
#include "rover.h"
#include "neuralNetwork.h"


using namespace std;


class POI{
public:
    double x_position_poi,y_position_poi,value_poi;
    //Environment test;
    //vector<Rover> individualRover;
};

#endif /* POI_h */
