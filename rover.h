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
double resolve(double angle);


class Rover{
    //Environment environment_object;
public:
    double x_position,y_position;
    vector<double> sensors;
    vector<Net> singleneuralNetwork;
    void sense_poi(double x, double y, double val);
    void sense_rover(double x, double y);
    double sense_poi_delta(double x_position_poi,double y_position_poi);
    double sense_rover_delta(double x_position_otherrover, double y_position_otherrover);
    vector<double> controls;
    //void get_all_sensorvalues(double x_position_poi,double y_position_poi,double x_position_otherrover, double y_position_otherrover, double phi);
    double delta_x,delta_y;
    double theta;
    double phi;
    void reset_sensors();
    int find_quad(double x, double y);
    double find_phi(double x, double y);
    double find_theta(double x_sensed, double y_sensed);
    void move_rover(double dx, double dy);
};

//Function returns: sum of values of POIs divided by their distance
double Rover::sense_poi_delta(double x_position_poi,double y_position_poi ){
    double delta_sense_poi=0;
    double distance = sqrt(pow(x_position-x_position_poi, 2)+pow(y_position-y_position_poi, 2));
    double minimum_observation_distance =0.0;
    delta_sense_poi=(distance>minimum_observation_distance)?distance:minimum_observation_distance ;
    return delta_sense_poi;
}

//Function returns: sum of sqaure distance from a rover to all the other rovers in the quadrant
double Rover::sense_rover_delta(double x_position_otherrover, double y_position_otherrover){
    double delta_sense_rover=0.0;
    if (x_position_otherrover == NULL || y_position_otherrover == NULL) {
        return delta_sense_rover;
    }
    double distance = sqrt(pow(x_position-x_position_otherrover, 2)+pow(y_position-y_position_otherrover, 2));
    delta_sense_rover=(1/distance);
    
    return delta_sense_rover;
}

void Rover::sense_poi(double poix, double poiy, double val){
    double delta = sense_poi_delta(poix, poiy);
    int quad = find_quad(poix,poiy);
    sensors.at(quad) += val/delta;
}

void Rover::sense_rover(double otherx, double othery){
    double delta = sense_poi_delta(otherx,othery);
    int quad = find_quad(otherx,othery);
    sensors.at(quad+4) += 1/delta;
}

void Rover::reset_sensors(){
    sensors.clear();
    for(int i=0; i<8; i++){
        sensors.push_back(0.0);
    }
}

double Rover::find_phi(double x_sensed, double y_sensed){
    double distance_in_x_phi =  x_sensed - x_position;
    double distance_in_y_phi =  y_sensed - y_position;
    double phi = atan2(distance_in_y_phi,distance_in_x_phi) * (180 / PI);
    return phi;
}

double Rover::find_theta(double x_sensed, double y_sensed){
    double distance_in_x_theta =  x_sensed - x_position;
    double distance_in_y_theta =  y_sensed - y_position;
    theta += atan2(distance_in_x_theta,distance_in_y_theta) * (180 / PI);
    return phi;
}

int Rover::find_quad(double x_sensed, double y_sensed){
    int quadrant;
    
    double phi = find_phi(x_sensed, y_sensed);
    double quadrant_angle = phi - theta;
    quadrant_angle = resolve(quadrant_angle);
    cout << "IN QUAD: FIND PHI: " << phi << endl;
    
    phi = resolve(phi);
    
    cout << "IN QUAD: FIND PHI2: " << phi << endl;
    
    int case_number;
    if ((0<=quadrant_angle && 45>= quadrant_angle)||(315<quadrant_angle && 360>= quadrant_angle)) {
        //do something in Q1
        case_number = 0;
    }else if ((45<quadrant_angle && 135>= quadrant_angle)) {
        // do something in Q2
        case_number = 1;
    }else if((135<quadrant_angle && 225>= quadrant_angle)){
        //do something in Q3
        case_number = 2;
    }else if((225<quadrant_angle && 315>= quadrant_angle)){
        //do something in Q4
        case_number = 3;
    }
    quadrant = case_number;
    
    cout << "QUADANGLE =  " << quadrant_angle << endl;
    cout << "QUADRANT = " << quadrant << endl;
    
    return quadrant;
}

void Rover::move_rover(double dx, double dy){
    
    double aom = atan2(dy,dx)*180/PI; /// angle of movement
//    cout << "AOM: " << aom << endl;
//    
//    cout << "Dx: " << dx << endl;
//    cout << "Dy: " << dy << endl;
    
    double rad2deg = PI/180;
    x_position = x_position + sin(theta*rad2deg) * dy + cos((theta)*rad2deg) * dx;
    y_position = y_position + sin((theta)*rad2deg) * dx + cos(theta*rad2deg) * dy;
    theta = theta + aom;
    theta = resolve(theta);
    
    //x_position =(x_position)+  (dy* cos(theta*(PI/180)))-(dx *sin(theta*(PI/180)));
    //y_position =(y_position)+ (dy* sin(theta*(PI/180)))+(dx *cos(theta*(PI/180)));
    //theta = theta+ (atan2(dx,dy) * (180 / PI));
    //theta = resolve(theta);
}



/*
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
            //sensors.push_back(sense_poi(x_position_poi,y_position_poi));
        }else{
            sensors.push_back(0);
        }
    }
    for (int i=0; i<4; i++) {
        //sensors.push_back(sense_rover(x_position_otherrover,y_position_otherrover));
    }
}*/




#endif /* rover_h */
