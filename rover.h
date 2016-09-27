//
//  rover.h
//  roverDomainNN
//
//  Created by ak on 9/26/16.
//  Copyright © 2016 ak. All rights reserved.
//

#ifndef rover_h
#define rover_h
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


#define PI 3.14159265

using namespace std;



class Rover{
    //Environment environment_object;
public:
    double x_position,y_position,x_initial,y_initial;
    vector<double> sensors;
    vector<Net> singleneuralNetwork;
    double sense_poi(double x_position_poi,double y_position_poi);
    double sense_rover(double x_position_otherrover, double y_position_otherrover);
    double initial_sense_poi(double x_position_poi,double y_position_poi);
    double initial_sense_rover(double x_position_otherrover, double y_position_otherrover);
    vector<double> controls;
    void get_all_sensorvalues(double x_position_poi,double y_position_poi,double x_position_otherrover, double y_position_otherrover, double phi);
    void get_all_sensorvalues_initial(double x_position_poi,double y_position_poi,double x_position_otherrover, double y_position_otherrover);
    double delta_x,delta_y;
    double theta;
    double phi;
    void find_poi_quadrant(int x_position,int y_position);
};

double Rover::initial_sense_poi(double x_position_poi,double y_position_poi){
    double delta=0;
    double distance = sqrt(pow(x_initial-x_position_poi, 2)+pow(y_initial-y_position_poi, 2));
    double minimum_observation_distance =0.0;
    delta=(distance>minimum_observation_distance)?distance:minimum_observation_distance ;
    return delta;
}

double Rover::initial_sense_rover(double x_position_otherrover, double y_position_otherrover){
    double delta_sense_rover=0.0;
    if (x_position_otherrover == NULL || y_position_otherrover == NULL) {
        return delta_sense_rover;
    }
    double distance = sqrt(pow(x_initial-x_position_otherrover, 2)+pow(y_initial-y_position_otherrover, 2));
    delta_sense_rover=(1/distance);
    return delta_sense_rover;
}

//Function returns: sum of values of POIs divided by their distance
double Rover::sense_poi(double x_position_poi,double y_position_poi ){
    double delta_sense_poi=0;
    double distance = sqrt(pow(x_position-x_position_poi, 2)+pow(y_position-y_position_poi, 2));
    double minimum_observation_distance =0.0;
    delta_sense_poi=(distance>minimum_observation_distance)?distance:minimum_observation_distance ;
    return delta_sense_poi;
}

//Function returns: sum of sqaure distance from a rover to all the other rovers in the quadrant
double Rover::sense_rover(double x_position_otherrover, double y_position_otherrover){
    double delta_sense_rover=0.0;
    if (x_position_otherrover == NULL || y_position_otherrover == NULL) {
        return delta_sense_rover;
    }
    double distance = sqrt(pow(x_position-x_position_otherrover, 2)+pow(y_position-y_position_otherrover, 2));
    delta_sense_rover=(1/distance);
    
    return delta_sense_rover;
}

void Rover::get_all_sensorvalues(double x_position_poi,double y_position_poi,double x_position_otherrover, double y_position_otherrover, double phi){
    cout<<"THis is phi value:::"<<phi<<endl;
    if (phi>360) {
        phi = phi-360;
    }
    int case_number = 0;
    if ((0<=phi && 45>= phi)||(315<phi && 360>= phi)) {
        //do something in Q1
        case_number = 1;
    }else if ((45<phi && 135>= phi)) {
        // do something in Q2
        case_number = 2;
    }else if((135<phi && 225>= phi)){
        //do something in Q3
        case_number = 3;
    }else if((225<phi && 315>= phi)){
        //do something in Q4
        case_number = 4;
    }
    for (int i=1; i<5; i++) {
        if (case_number == i) {
            sensors.push_back(sense_poi(x_position_poi,y_position_poi));
        }else{
            sensors.push_back(0);
        }
    }
    for (int i=0; i<4; i++) {
        sensors.push_back(sense_rover(x_position_otherrover,y_position_otherrover));
    }
}

void Rover::get_all_sensorvalues_initial(double x_position_poi, double y_position_poi, double x_position_otherrover, double y_position_otherrover){
    for (int i=0; i<4; i++) {
        sensors.push_back(initial_sense_poi(x_position_poi, y_position_poi));
    }
    for (int i=0; i<4; i++) {
        sensors.push_back(initial_sense_poi(x_position_otherrover,y_position_otherrover));
    }
}




#endif /* rover_h */
