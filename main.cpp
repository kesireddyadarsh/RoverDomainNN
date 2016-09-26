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
    //cout<<"This is size of neural networks::"<<mypop.popVector.size()<<endl;
    //cout<<"This is size of layer ::"<<mypop.popVector.at(10).z_layer.at(0).size()<<endl;
    //for (int i=0; i<mypop.popVector.size(); i++) {
        //cout<<"This is weight"<<mypop.popVector.at(10).z_layer.at(0)<<endl;
    //}
    //cout<<"This is weight of connection:::"<<mypop.popVector.at(10).z_layer.at(0).size()<<endl;
    
    //Create values for Rover;
    int x_position_otherRover=0;
    int y_position_otherRover=0;
    
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
    //choose your teta
    int first_net =0;
    for (int total_loop =0; total_loop<numNN; total_loop++) {
    for (int single_net=0; single_net<100; single_net++) {
        
        individualRover.get_all_sensorvalues(individualPOI.x_position_poi, individualPOI.y_position_poi, x_position_otherRover, y_position_otherRover);
        for (int i=0; i<individualRover.sensors.size(); i++) {
            cout<<"This is sensor value:::"<<individualRover.sensors.at(i)<<endl;
        }
        //run the nn
        mypop.runNetwork(individualRover.sensors);
        //dx and dy
        //Change the dx and dy
        individualRover.delta_x = mypop.popVector.at(first_net).outputvaluesNN.at(0);
        individualRover.delta_y = mypop.popVector.at(first_net).outputvaluesNN.at(1);
        //move rover
        individualRover.x_position=individualRover.x_position+individualRover.delta_x;
        individualRover.y_position=individualRover.y_position+individualRover.delta_y;
        //Create teta and keep chaning
        cout<<"This is x position::"<<individualRover.x_position<<endl;
        cout<<"This is y position::"<<individualRover.y_position<<endl;
        individualRover.sensors.clear();
        
    }
    first_net++;
    }
    
    //find teta value
    
        //cout<<"This is delta_y"<<individualRover.delta_y<<endl;
    //move rover
    //evaluate to POI
    //loop to run rover sensors all 8
    //loop to import wt's
    
    //sort and evolution
    
    //repeat above all time some number of times
    
    return 0;
}
