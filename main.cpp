//
//  main.cpp
//  evv_NN_V4
//
//  Created by adarsh kesireddy on 4/11/16.
//  Copyright © 2016 adarsh kesireddy. All rights reserved.
//

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


//This is main function
int main(int argc, const char * argv[]) {
    // insert code here...
    //Try to move Neural network in to header
    srand(time(NULL));
    
    //Create numNN of neural network
    int numNN=100;
    //int numCases = 4;
    vector<unsigned> topology;
    topology.clear();
    topology.push_back(8);
    topology.push_back(10);
    topology.push_back(2);
    Population mypop(numNN,topology);
    
    //Create values for Rover;
    int x_position_otherRover=NULL;
    int y_position_otherRover=NULL;
    
    Environment world;
    //Set values of poi's
    POI individualPOI;
    individualPOI.x_position_poi=30;
    individualPOI.y_position_poi=50;
    individualPOI.value_poi=100;
    
    
    //vectors of rovers
    vector<Rover> teamRover;
    
    //import weight into rovers
    Rover individualRover;
    //run the rover sensors all 8
    individualRover.x_position=1;
    individualRover.y_position=1;
    individualRover.theta = 90 ; //radians
    individualRover.get_all_sensorvalues(individualPOI.x_position_poi, individualPOI.y_position_poi, x_position_otherRover, y_position_otherRover);
    
    for (int i=0; i<individualRover.sensors.size(); i++) {
        cout<<"This is sensor value:::"<<individualRover.sensors.at(i)<<endl;
    }
    
    return 0;
}
