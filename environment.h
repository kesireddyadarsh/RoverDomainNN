//
//  environment.h
//  roverDomainNN
//
//  Created by ak on 9/26/16.
//  Copyright © 2016 ak. All rights reserved.
//

#ifndef environment_h
#define environment_h

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


class Environment{
public:
    vector<POI> individualPOI;
};

#endif /* environment_h */
