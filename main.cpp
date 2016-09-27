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
#include <math.h>
#include "environment.h"
#include "POI.h"
#include "evolutionAlgorithm.h"
#include "rover.h"
#include "neuralNetwork.h"

#define PI 3.14159265

using namespace std;

double resolve(double angle){
    while(angle > 360){
        angle -=360;
    }
    while(angle < 0){
        angle += 360;
    }
    return angle;
}

double determineQuadrant(int distance_in_x_phi, int distance_in_y_phi){
    //double temp = ((double)distance_in_y_phi/distance_in_x_phi);
    double phi = atan2(distance_in_y_phi,distance_in_x_phi) * (180 / PI);
    return phi;
}

double calculate_theta(double dx,double dy,double theta){
    theta = theta + ((atan2(dx,dy)) * (180 / PI));
    return theta;
}

void movement_rover(double phi,double theta){
    double angle = phi-theta;
}

bool full_sensor_test(){
    bool passfail = false;
    
    bool passrover=false;
    bool passsensor=false;
    
    if(passrover && passsensor){
        passfail = true;
    }
    
    return passfail;
}

bool POI_sensor_test(){
    bool VERBOSE = true;
    
    bool passfail = false;
    
    bool pass1 = false;
    bool pass2 = false;
    bool pass3 = false;
    bool pass4 = false;
    
    POI P;
    Rover R;
    
    /// Stationary Rover
    R.x_position = 0;
    R.y_position = 0;
    R.theta = 0; /// north
    
    P.value_poi = 10;
    
    /// POI directly north, sensor 0 should read; no others.
    P.x_position_poi = 0.001;
    P.y_position_poi = 1;
    
    // sense.
    R.reset_sensors();
    R.sense_poi(P.x_position_poi, P.y_position_poi, P.value_poi);
    
    if(R.sensors.at(0) != 0 && R.sensors.at(1) == 0 && R.sensors.at(2) ==0 && R.sensors.at(3) == 0){
        pass1 = true;
    }
    
    assert(pass1 == true);
    
    if(VERBOSE){
    cout << "Direct north case: " << endl;
    for(int sen = 0; sen < R.sensors.size(); sen++){
        cout << R.sensors.at(sen) << "\t";
    }
    cout << endl;
    }
    
    /// POI directly south, sensor 2 should read; no others.
    P.x_position_poi = 0;
    P.y_position_poi = -1;
    
    // sense.
    R.reset_sensors();
    R.sense_poi(P.x_position_poi, P.y_position_poi, P.value_poi);
    
    if(R.sensors.at(0) == 0 && R.sensors.at(1) == 0 && R.sensors.at(2) !=0 && R.sensors.at(3) == 0){
        pass2 = true;
    }
    
    assert(pass2 == true);
    
    if(VERBOSE){
        cout << "Direct south case: " << endl;
        for(int sen = 0; sen < R.sensors.size(); sen++){
            cout << R.sensors.at(sen) << "\t";
        }
        cout << endl;
    }
    
    /// POI directly east, sensor 1 should read; no others.
    P.x_position_poi = 1;
    P.y_position_poi = 0;
    
    // sense.
    R.reset_sensors();
    R.sense_poi(P.x_position_poi, P.y_position_poi, P.value_poi);
    
    if(R.sensors.at(0) == 0 && R.sensors.at(1) != 0 && R.sensors.at(2) ==0 && R.sensors.at(3) == 0){
        pass3 = true;
    }
    
    assert(pass3 == true);
    
    if(VERBOSE){
        cout << "Direct east case: " << endl;
        for(int sen = 0; sen < R.sensors.size(); sen++){
            cout << R.sensors.at(sen) << "\t";
        }
        cout << endl;
    }

    
    /// POI directly west, sensor 3 should read; no others.
    P.x_position_poi = -1;
    P.y_position_poi = 0;
    
    // sense.
    R.reset_sensors();
    R.sense_poi(P.x_position_poi, P.y_position_poi, P.value_poi);
    
    if(R.sensors.at(0) == 0 && R.sensors.at(1) == 0 && R.sensors.at(2) ==0 && R.sensors.at(3) != 0){
        pass4 = true;
    }
    
    assert(pass4 == true);
    
    if(VERBOSE){
        cout << "Direct west case: " << endl;
        for(int sen = 0; sen < R.sensors.size(); sen++){
            cout << R.sensors.at(sen) << "\t";
        }
        cout << endl;
    }

    
    
    if(pass1 && pass2 && pass3 && pass4){
        passfail = true;
    }
    assert(passfail == true);
    return passfail;
}

bool rover_sensor_test(){
    bool passfail = false;
    
    bool pass5 = false;
    bool pass6 = false;
    bool pass7 = false;
    bool pass8 = false;
    
    Rover R1;
    Rover R2;
    R1.x_position = 0;
    R1.y_position = 0;
    R1.theta = 0; // north
    R2.theta = 0;
    
    // case 1, Rover 2 to the north
    R2.x_position = 0;
    R2.y_position = 1;
    R1.reset_sensors();
    R1.sense_rover(R2.x_position,R2.y_position);
    /// sensor 4 should fire, none other.
    if(R1.sensors.at(4) != 0 && R1.sensors.at(5) == 0 && R1.sensors.at(6) == 0 && R1.sensors.at(7) == 0){
        pass5 = true;
    }
    assert(pass5 == true);
    
    // case 2, Rover 2 to the east
    R2.x_position = 1;
    R2.y_position = 0;
    R1.reset_sensors();
    R1.sense_rover(R2.x_position,R2.y_position);
    /// sensor 5 should fire, none other.
    if(R1.sensors.at(4) == 0 && R1.sensors.at(5) != 0 && R1.sensors.at(6) == 0 && R1.sensors.at(7) == 0){
        pass6 = true;
    }
    assert(pass6 == true);
    
    // case 3, Rover 2 to the south
    R2.x_position = 0;
    R2.y_position = -1;
    R1.reset_sensors();
    R1.sense_rover(R2.x_position,R2.y_position);
    /// sensor 6 should fire, none other.
    if(R1.sensors.at(4) == 0 && R1.sensors.at(5) == 0 && R1.sensors.at(6) != 0 && R1.sensors.at(7) == 0){
        pass7 = true;
    }
    assert(pass7 == true);
    
    // case 4, Rover 2 to the west
    R2.x_position = -1;
    R2.y_position = 0;
    R1.reset_sensors();
    R1.sense_rover(R2.x_position,R2.y_position);
    /// sensor 7 should fire, none other.
    if(R1.sensors.at(4) == 0 && R1.sensors.at(5) == 0 && R1.sensors.at(6) == 0 && R1.sensors.at(7) != 0){
        pass8 = true;
    }
    assert(pass8 == true);
    
    if(pass5 && pass6 && pass7 && pass8){
        passfail = true;
    }
    assert(passfail == true);
    return passfail;
}

void custom_test(){
    Rover R;
    POI P;
    R.x_position = 0;
    R.y_position = 0;
    R.theta = 90;
    
    P.x_position_poi = 0.56;
    P.y_position_poi = -1.91;
    P.value_poi = 100;
    
    R.reset_sensors();
    R.sense_poi(P.x_position_poi,P.y_position_poi,P.value_poi);
    
    
}


//This is main function
int main(int argc, const char * argv[]) {
    // insert code here...
    //Try to move Neural network in to header
    srand((unsigned)time(NULL));
    
    //POI_sensor_test();
    //rover_sensor_test();
    custom_test();
    
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
    individualPOI.x_position_poi=30.0;
    individualPOI.y_position_poi=50.0;
    individualPOI.value_poi=100;
    
    
    //vectors of rovers
    vector<Rover> teamRover;
    
    //import weight into rovers
    Rover individualRover;
    //run the rover sensors all 8
    individualRover.x_position=1.0;
    individualRover.y_position=1.0;
    individualRover.theta = 0.0 ; //radians
    double distance_in_x_phi= individualPOI.x_position_poi-individualRover.x_position;
    double distance_in_y_phi= individualPOI.y_position_poi-individualRover.y_position;
    cout<<"This is distance in x::"<<distance_in_x_phi<<endl;
    cout<<"This is distance in y::"<<distance_in_y_phi<<endl;
    individualRover.phi = determineQuadrant(distance_in_x_phi,distance_in_y_phi);
    cout<<"this is phi value in main::::"<<individualRover.phi<<endl;
    individualRover.get_all_sensorvalues(individualPOI.x_position_poi, individualPOI.y_position_poi, x_position_otherRover, y_position_otherRover,individualRover.phi);
    
    for (int i=0; i<individualRover.sensors.size(); i++) {
        cout<<"This is sensor value:::"<<individualRover.sensors.at(i)<<endl;
    }
    
    for (int i=0; i<100; i++) {
        mypop.runNetwork(individualRover.sensors);
        double dx = mypop.popVector.at(0).outputvaluesNN.at(0);
        double dy = mypop.popVector.at(0).outputvaluesNN.at(1);
        individualRover.theta = calculate_theta(dx,dy,individualRover.theta);
        double delta_x = (dy *sin(individualRover.theta))+(dx *sin(individualRover.theta));
        double delta_y = (dy *cos(individualRover.theta))+(dx *cos(individualRover.theta));
        individualRover.x_position += delta_x;
        individualRover.y_position += delta_y;
        double distance_in_x_phi= individualPOI.x_position_poi-individualRover.x_position;
        double distance_in_y_phi= individualPOI.y_position_poi-individualRover.y_position;
        cout<<"This is distance in x::"<<distance_in_x_phi<<endl;
        cout<<"This is distance in y::"<<distance_in_y_phi<<endl;
        individualRover.phi = determineQuadrant(distance_in_x_phi,distance_in_y_phi);
        movement_rover(individualRover.phi,individualRover.theta);
        individualRover.sensors.clear();
        individualRover.get_all_sensorvalues(individualPOI.x_position_poi, individualPOI.y_position_poi, x_position_otherRover, y_position_otherRover,individualRover.phi);
        
    }
    
    return 0;
}
