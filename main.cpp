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
#include <fstream>
#include "environment.h"
#include "POI.h"
#include "evolutionAlgorithm.h"
#include "rover.h"
#include "neuralNetwork.h"

#define PI 3.14159265

bool test_rover = true;
bool runNeuralNetwork = true;
bool development_tool =false;


using namespace std;

// Will resolve angle between 0 to 360
double resolve(double angle){
    while(angle >= 360){
        angle -=360;
    }
    while(angle < 0){
        angle += 360;
    }
    while (angle == 360) {
        angle = 0;
    }
    return angle;
}

// Contains boolean values returns pass or fail
bool full_sensor_test(){
    bool passfail = false;
    
    bool passrover=false;
    bool passsensor=false;
    
    if(passrover && passsensor){
        passfail = true;
    }
    
    return passfail;
}


// Tests Stationary POI and Stationary Rover in all directions
bool POI_sensor_test(){
    bool VERBOSE = development_tool;
    
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
    
    if(VERBOSE){
        cout << "Direct west case: " << endl;
        for(int sen = 0; sen < R.sensors.size(); sen++){
            cout << R.sensors.at(sen) << "\t";
        }
        cout << endl;
    }
    assert(pass4 == true);
    
    
    if(pass1 && pass2 && pass3 && pass4){
        passfail = true;
    }
    assert(passfail == true);
    return passfail;
}


//Test for stationary rovers test in all directions
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

//x and y position of poi
vector<vector<double>> poi_positions;
vector<double> poi_positions_loc;

void stationary_rover_test(double x_start,double y_start){//Pass x_position,y_position
    Rover R_obj; //Rover object
    POI P_obj;
    
    R_obj.reset_sensors();
    
    //x and y position of poi
    vector<vector<double>> poi_positions;
    vector<double> poi_positions_loc;
    
    R_obj.x_position =x_start;
    R_obj.y_position=y_start;
    R_obj.theta=0.0;
    int radius = 2;
    
    double angle=0;
    
    P_obj.value_poi=100;
    
    int quad_0=0,quad_1=0,quad_2=0,quad_3=0,quad_0_1=0;
    while (angle<360) {
        if ((0<=angle && 45>= angle)) {
            quad_0++;
        }else if ((45<angle && 135>= angle)) {
            // do something in Q2
            quad_1++;
        }else if((135<angle && 225>= angle)){
            //do something in Q3
            quad_2++;
        }else if((225<angle && 315>= angle)){
            //do something in Q4
            quad_3++;
        }else if ((315<angle && 360> angle)){
            quad_0_1++;
        }
        poi_positions_loc.push_back(R_obj.x_position+(radius*cos(angle * (PI /180))));
        poi_positions_loc.push_back(R_obj.y_position+(radius*sin(angle * (PI /180))));
        poi_positions.push_back(poi_positions_loc);
        poi_positions_loc.clear();
        angle+=7;
    }
    
    vector<bool> checkPass_quad_1,checkPass_quad_2,checkPass_quad_3,checkPass_quad_0;
    
    for (int i=0; i<poi_positions.size(); i++) {
        for (int j=0; j<poi_positions.at(i).size(); j++) {
            P_obj.x_position_poi = poi_positions.at(i).at(j);
            P_obj.y_position_poi = poi_positions.at(i).at(++j);
            R_obj.sense_poi(P_obj.x_position_poi, P_obj.y_position_poi, P_obj.value_poi);
            if (R_obj.sensors.at(0) != 0 && R_obj.sensors.at(1) == 0 && R_obj.sensors.at(2) ==0 && R_obj.sensors.at(3) == 0) {
                checkPass_quad_0.push_back(true);
            }else if (R_obj.sensors.at(0) == 0 && R_obj.sensors.at(1) != 0 && R_obj.sensors.at(2) ==0 && R_obj.sensors.at(3) == 0){
                checkPass_quad_1.push_back(true);
            }else if (R_obj.sensors.at(0) == 0 && R_obj.sensors.at(1) == 0 && R_obj.sensors.at(2) !=0 && R_obj.sensors.at(3) == 0){
                checkPass_quad_2.push_back(true);
            }else if (R_obj.sensors.at(0) == 0 && R_obj.sensors.at(1) == 0 && R_obj.sensors.at(2) ==0 && R_obj.sensors.at(3) != 0){
                checkPass_quad_3.push_back(true);
            }
            R_obj.reset_sensors();
        }
    }
    if (checkPass_quad_0.size() != (quad_0_1+quad_0)) {
        cout<<"Something wrong with quad_0"<<endl;;
    }else if (checkPass_quad_1.size() != (quad_1)){
        cout<<"Something wrong with quad_1"<<endl;
    }else if (checkPass_quad_2.size() != quad_2){
        cout<<"Something wrong with quad_2"<<endl;
    }else if (checkPass_quad_3.size() != quad_3){
        cout<<"Something wrong with quad_3"<<endl;
    }
}

void find_x_y_stationary_rover_test_1(double angle, double radius, double x_position, double y_position){
    poi_positions_loc.push_back(x_position+(radius*cos(angle * (PI /180))));
    poi_positions_loc.push_back(y_position+(radius*sin(angle * (PI /180))));
}

void stationary_rover_test_1(double x_start,double y_start){
    bool VERBOSE = development_tool;
    Rover R_obj; //Rover object
    POI P_obj;
    
    R_obj.reset_sensors();
    
    R_obj.x_position =x_start;
    R_obj.y_position=y_start;
    R_obj.theta=0.0;
    int radius = 2;
    
    bool check_pass = false;
    
    double angle=0;
    
    P_obj.value_poi=100;
    
    while (angle<360) {
        find_x_y_stationary_rover_test_1(angle, radius, R_obj.x_position, R_obj.y_position);
        P_obj.x_position_poi = poi_positions_loc.at(0);
        P_obj.y_position_poi = poi_positions_loc.at(1);
        R_obj.sense_poi(P_obj.x_position_poi, P_obj.y_position_poi, P_obj.value_poi);
        if (R_obj.sensors.at(0) != 0 && R_obj.sensors.at(1) == 0 && R_obj.sensors.at(2) ==0 && R_obj.sensors.at(3) == 0) {
            if (VERBOSE) {
                cout<<"Pass Quad 0"<<endl;
            }
            check_pass = true;
        }else  if (R_obj.sensors.at(0) == 0 && R_obj.sensors.at(1) != 0 && R_obj.sensors.at(2) ==0 && R_obj.sensors.at(3) == 0) {
            if (VERBOSE) {
                cout<<"Pass Quad 1"<<endl;

            }
                check_pass = true;
            }else if (R_obj.sensors.at(0) == 0 && R_obj.sensors.at(1) == 0 && R_obj.sensors.at(2) !=0 && R_obj.sensors.at(3) == 0) {
                if (VERBOSE) {
                    cout<<"Pass Quad 2"<<endl;
                }
                check_pass = true;
            }else if (R_obj.sensors.at(0) == 0 && R_obj.sensors.at(1) == 0 && R_obj.sensors.at(2) ==0 && R_obj.sensors.at(3) != 0) {
                if (VERBOSE) {
                    cout<<"Pass Quad 3"<<endl;
                }
                check_pass = true;
            }else{
                cout<<"Issue at an angle ::"<<angle<<" with x_position and y_position"<<R_obj.x_position<<R_obj.y_position<<endl;
                exit(10);
            }
        assert(check_pass==true);
        poi_positions_loc.clear();
        R_obj.reset_sensors();
        angle+=7;
        check_pass=false;
    }
}

void stationary_poi_test(double x_start,double y_start){
    bool VERBOSE = development_tool;
    Rover R_obj; //Rover object
    POI P_obj; // POI object
    vector<double> rover_position_loc;
    
    R_obj.reset_sensors();
    
    P_obj.x_position_poi=x_start;
    P_obj.y_position_poi=y_start;
    P_obj.value_poi=100;
    R_obj.theta=0.0;
    
    R_obj.x_position =0.0;
    R_obj.y_position =0.0;
    
    bool check_pass = false;
    
    for (int i=0; i<=R_obj.theta; ) {
        if (R_obj.theta > 360) {
            break;
        }
        R_obj.sense_poi(P_obj.x_position_poi, P_obj.y_position_poi, P_obj.value_poi);
        if (VERBOSE) {
            cout<<endl;
            for (int j=0; j<R_obj.sensors.size(); j++) {
                cout<<R_obj.sensors.at(j)<<"\t";
            }
            cout<<endl;
        }
        if (R_obj.sensors.at(0) != 0 && R_obj.sensors.at(1) == 0 && R_obj.sensors.at(2) ==0 && R_obj.sensors.at(3) == 0) {
            if (VERBOSE) {
                cout<<"Pass Quad 0"<<endl;
            }
            check_pass = true;
        }else  if (R_obj.sensors.at(0) == 0 && R_obj.sensors.at(1) != 0 && R_obj.sensors.at(2) ==0 && R_obj.sensors.at(3) == 0) {
            if (VERBOSE) {
                cout<<"Pass Quad 1";
            }
            check_pass = true;
        }else if (R_obj.sensors.at(0) == 0 && R_obj.sensors.at(1) == 0 && R_obj.sensors.at(2) !=0 && R_obj.sensors.at(3) == 0) {
            if (VERBOSE) {
                cout<<"Pass Quad 2";
            }
            check_pass = true;
        }else if (R_obj.sensors.at(0) == 0 && R_obj.sensors.at(1) == 0 && R_obj.sensors.at(2) ==0 && R_obj.sensors.at(3) != 0) {
            if (VERBOSE) {
                cout<<"Pass Quad 3";
            }
            check_pass = true;
        }else{
            cout<<"Issue at an angle ::"<<R_obj.theta<<" with x_position and y_position"<<P_obj.x_position_poi<<P_obj.y_position_poi<<endl;
            exit(10);
        }
        assert(check_pass==true);
        i+=7;
        R_obj.theta+=7;
        R_obj.reset_sensors();
    }
}

void two_rovers_test(double x_start, double y_start){
    bool VERBOSE = development_tool;
    Rover R_obj; //Rover object
    POI P_obj; // POI object
    vector<double> rover_position_loc;
    
    R_obj.reset_sensors();
    
    double otherRover_x = x_start;
    double otherRover_y = y_start;
    P_obj.value_poi=100;
    R_obj.theta=0.0;
    
    R_obj.x_position =0.0;
    R_obj.y_position =0.0;
    
    bool check_pass = false;
    
    for (int i=0; i<=R_obj.theta; ) {
        if (R_obj.theta > 360) {
            break;
        }
        R_obj.sense_rover(otherRover_x, otherRover_y);
        if (VERBOSE) {
            cout<<endl;
            for (int j=0; j<R_obj.sensors.size(); j++) {
                cout<<R_obj.sensors.at(j)<<"\t";
            }
            cout<<endl;
        }
        if (R_obj.sensors.at(4) != 0 && R_obj.sensors.at(5) == 0 && R_obj.sensors.at(6) ==0 && R_obj.sensors.at(7) == 0) {
            if ((0<=R_obj.theta && 45>= R_obj.theta)||(315<R_obj.theta && 360>= R_obj.theta)) {
                if (VERBOSE) {
                    cout<<"Pass Quad 0"<<endl;
                }
                check_pass = true;
            }
            
        }else  if (R_obj.sensors.at(4) == 0 && R_obj.sensors.at(5) != 0 && R_obj.sensors.at(6) ==0 && R_obj.sensors.at(7) == 0) {
            if((45<R_obj.theta && 135>= R_obj.theta)){
                if (VERBOSE) {
                    cout<<"Pass Quad 1";
                }
                check_pass = true;
            }
        }else if (R_obj.sensors.at(4) == 0 && R_obj.sensors.at(5) == 0 && R_obj.sensors.at(6) !=0 && R_obj.sensors.at(7) == 0) {
            if((135<R_obj.theta && 225>= R_obj.theta)){
                if (VERBOSE) {
                    cout<<"Pass Quad 2";
                }
                check_pass = true;
            }
        }else if (R_obj.sensors.at(4) == 0 && R_obj.sensors.at(5) == 0 && R_obj.sensors.at(6) ==0 && R_obj.sensors.at(7) != 0) {
            if((225<R_obj.theta && 315>= R_obj.theta)){
                if (VERBOSE) {
                    cout<<"Pass Quad 3";
                }
                check_pass = true;
            }
        }else{
            cout<<"Issue at an angle ::"<<R_obj.theta<<" with x_position and y_position"<<P_obj.x_position_poi<<P_obj.y_position_poi<<endl;
            exit(10);
        }
        assert(check_pass==true);
        i+=7;
        R_obj.theta+=7;
        R_obj.reset_sensors();
    }

}

vector<double> row_values;
vector<vector<double>> assert_check_values;

void fill_assert_check_values(){
    //First set of x , y thetha values
    for(int i=0;i<3;i++)
        row_values.push_back(0);
    assert_check_values.push_back(row_values);
    row_values.clear();
    
    //second set of x,y,thetha values
    row_values.push_back(0);
    row_values.push_back(1);
    row_values.push_back(0);
    assert_check_values.push_back(row_values);
    row_values.clear();
    
    //third set of x,y,thetha values
    row_values.push_back(1);
    row_values.push_back(2);
    row_values.push_back(45);
    assert_check_values.push_back(row_values);
    row_values.clear();
    
    //fourth set of x,y,thetha values
    row_values.push_back(1);
    row_values.push_back(3);
    row_values.push_back(0);
    assert_check_values.push_back(row_values);
    row_values.clear();
    
    //fifth set of x,y,thetha values
    row_values.push_back(0);
    row_values.push_back(4);
    row_values.push_back(315);
    assert_check_values.push_back(row_values);
    row_values.clear();
    
    //sixth set of x,y,thetha values
    row_values.push_back(0);
    row_values.push_back(5);
    row_values.push_back(0);
    assert_check_values.push_back(row_values);
    row_values.clear();
    
}

bool tolerance(double delta_maniplate,double check_value){
    double delta = 0.0000001;
    if (((delta+ delta_maniplate)>check_value)|| ((delta- delta_maniplate)<check_value) || (( delta_maniplate)==check_value)) {
        return true;
    }else{
        return false;
    }
}


void test_path(double x_start, double y_start){
    bool VERBOSE = development_tool;
    Rover R_obj;
    POI P_obj;
    
    //given
    R_obj.x_position=x_start;
    R_obj.y_position=y_start;
    R_obj.theta=0.0;
    
    P_obj.x_position_poi=1.0;
    P_obj.y_position_poi=1.0;
    P_obj.value_poi=100;
    
    
    
    fill_assert_check_values();
    
    int step_number = 0;
    bool check_assert = false;
    
    if (VERBOSE) {
        cout<<R_obj.x_position<<"\t"<<R_obj.y_position<<"\t"<<R_obj.theta<<endl;
    }
    if (step_number==0) {
        if(tolerance(R_obj.x_position, assert_check_values.at(step_number).at(0))){
            if(tolerance(R_obj.y_position, assert_check_values.at(step_number).at(1))){
                if(tolerance(R_obj.theta, assert_check_values.at(step_number).at(2))){
                    check_assert=true;
                    step_number++;
                }
            }
        }
    }
    assert(check_assert);
    check_assert=false;
    
    double dx=0.0,dy=1.0;
    R_obj.move_rover(dx, dy);
    if (VERBOSE) {
        cout<<R_obj.x_position<<"\t"<<R_obj.y_position<<"\t"<<R_obj.theta<<endl;
    }
    if (step_number==1) {
        if(tolerance(R_obj.x_position, assert_check_values.at(step_number).at(0))){
            if(tolerance(R_obj.y_position, assert_check_values.at(step_number).at(1))){
                if(tolerance(R_obj.theta, assert_check_values.at(step_number).at(2))){
                    check_assert=true;
                    step_number++;
                }
            }
        }
    }
    assert(check_assert);
    check_assert=false;
    
    
    dx=1.0;
    dy=1.0;
    R_obj.move_rover(dx, dy);
    if (VERBOSE) {
        cout<<R_obj.x_position<<"\t"<<R_obj.y_position<<"\t"<<R_obj.theta<<endl;
    }
    if (step_number==2) {
        if(tolerance(R_obj.x_position, assert_check_values.at(step_number).at(0))){
            if(tolerance(R_obj.y_position, assert_check_values.at(step_number).at(1))){
                if(tolerance(R_obj.theta, assert_check_values.at(step_number).at(2))){
                    check_assert=true;
                    step_number++;
                }
            }
        }
    }
    assert(check_assert);
    check_assert=false;
    
    dx=-1/sqrt(2.0);
    dy=1/sqrt(2.0);
    R_obj.move_rover(dx, dy);
    R_obj.reset_sensors();
    if (VERBOSE) {
        cout<<R_obj.x_position<<"\t"<<R_obj.y_position<<"\t"<<R_obj.theta<<endl;
    }
    if (step_number==3) {
        if(tolerance(R_obj.x_position, assert_check_values.at(step_number).at(0))){
            if(tolerance(R_obj.y_position, assert_check_values.at(step_number).at(1))){
                if(tolerance(R_obj.theta, assert_check_values.at(step_number).at(2))){
                    check_assert=true;
                    step_number++;
                }
            }
        }
    }
    assert(check_assert);
    check_assert=false;
    
    dx=-1.0;
    dy=1.0;
    R_obj.move_rover(dx, dy);
    R_obj.reset_sensors();
    if (VERBOSE) {
        cout<<R_obj.x_position<<"\t"<<R_obj.y_position<<"\t"<<R_obj.theta<<endl;
    }
    if (step_number==4) {
        if(tolerance(R_obj.x_position, assert_check_values.at(step_number).at(0))){
            if(tolerance(R_obj.y_position, assert_check_values.at(step_number).at(1))){
                if(tolerance(R_obj.theta, assert_check_values.at(step_number).at(2))){
                    check_assert=true;
                    step_number++;
                }
            }
        }
    }
    assert(check_assert);
    check_assert=false;
    
    dx=1/sqrt(2.0);
    dy=1/sqrt(2.0);
    R_obj.move_rover(dx, dy);
    R_obj.reset_sensors();
    if (VERBOSE) {
        cout<<R_obj.x_position<<"\t"<<R_obj.y_position<<"\t"<<R_obj.theta<<endl;
    }
    if (step_number==5) {
        if(tolerance(R_obj.x_position, assert_check_values.at(step_number).at(0))){
            if(tolerance(R_obj.y_position, assert_check_values.at(step_number).at(1))){
                if(tolerance(R_obj.theta, assert_check_values.at(step_number).at(2))){
                    check_assert=true;
                    step_number++;
                }
            }
        }
    }
    assert(check_assert);
    check_assert=false;

}

vector<vector<double>> point_x_y_circle;
vector<double> temp;

void find_x_y_test_circle_path(double start_x_position,double start_y_position,double angle){
    double radius = 1.0;
    temp.push_back(start_x_position+(radius*cos(angle * (PI /180))));
    temp.push_back(start_y_position+(radius*sin(angle * (PI/180))));
}

void test_circle_path(double x_start,double y_start){
    bool VERBOSE = development_tool;
    Rover R_obj;
    POI P_obj;
    
    P_obj.x_position_poi=0.0;
    P_obj.y_position_poi=0.0;
    P_obj.value_poi=100.0;
    
    if (VERBOSE) {
        cout<<R_obj.x_position<<"\t"<<R_obj.y_position<<"\t"<<R_obj.theta<<endl;
    }
    
    double dx=0.0,dy=1.0;
    double angle=0.0;
    
    for(;angle<=360;){
        R_obj.x_position=x_start;
        R_obj.y_position=y_start;
        R_obj.theta=0.0;
        find_x_y_test_circle_path(x_start, y_start,angle);
        dx=temp.at(0);
        dy=temp.at(1);
        R_obj.move_rover(dx, dy);
        assert(tolerance(R_obj.x_position, dx));
        assert(tolerance(R_obj.y_position, dy));
        assert(tolerance(R_obj.theta, angle));
        temp.clear();
        angle+=15.0;
    }
    
}

void test_all_sensors(){
    
    POI_sensor_test();
    rover_sensor_test();
    custom_test();
    double x_start = 0.0, y_start = 0.0;
    stationary_rover_test(x_start,y_start);
    stationary_rover_test_1(x_start, y_start);
    stationary_poi_test(x_start,y_start);
    two_rovers_test(x_start,y_start);
    test_path(x_start,y_start);
    x_start = 0.0, y_start = 0.0;
    test_circle_path(x_start,y_start);

}

double find_scaling_number(){
    double number =0.0;
    double temp_number =0.0;
    Rover R_obj; //Rover object
    POI P_obj; // POI object
    
    P_obj.x_position_poi=50.0;
    P_obj.y_position_poi=100.0;
    P_obj.value_poi =100;
    
    int temp_rand = rand()%100;
    vector<vector<double>> group_sensors;
    
    for (int temp=0; temp<temp_rand; temp++) {
        R_obj.x_position=rand()%100;
        R_obj.y_position=rand()%100;
        
        R_obj.reset_sensors();
        R_obj.sense_poi(P_obj.x_position_poi, P_obj.y_position_poi, P_obj.value_poi);
        group_sensors.push_back(R_obj.sensors);
    }
    
    for (int i=0; i<group_sensors.size(); i++) {
        temp_number=*max_element(group_sensors.at(i).begin(), group_sensors.at(i).end());
        if (temp_number>number) {
            number=temp_number;
        }
    }
    
    R_obj.reset_sensors();
    
    return number;
}


//This is main function
int main(int argc, const char * argv[]) {
    bool VERBOSE = development_tool;
    srand((unsigned)time(NULL));
    
    
    if (test_rover == true) {
         test_all_sensors();
        if (VERBOSE) {
            cout<<"\n\n\n$$$This is end of test all sensors function$$$\n\n"<<endl;
        }
    }

    if (runNeuralNetwork == true) {
        cout<<"\n\n Neural network"<<endl;
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
        
        //Rover object
        Rover Robj;
        
        //import weight into rovers
        Rover individualRover;
        
        //Find scaling number
        double scaling_number = find_scaling_number();
        int number = scaling_number;
        
        
        //object for file
        ofstream myfile_x_y;
        ofstream myfile_sensor;
        myfile_x_y.open("x_y coordinates");
        myfile_sensor.open("sensor_values");
        ofstream myfile_generation_fitness;
        myfile_generation_fitness.open("fitness_generation");
        /////////////start of generation loop
        for (int generation = 0; generation<50; generation++) {
            for (int indiviualNN=0; indiviualNN<numNN; indiviualNN++) {
                individualRover.x_position=10.0; //x_position of rover
                individualRover.y_position=10.0; //y_position of rover
                individualRover.theta = 0.0 ; //radians
                
                for (int indiviualNN_iteration=0; indiviualNN_iteration<100; indiviualNN_iteration++) {
                    
                    individualRover.reset_sensors();// reset sensor
                    individualRover.sense_poi(individualPOI.x_position_poi, individualPOI.y_position_poi, individualPOI.value_poi);//sense all around
                    
                    for (int change_input=0; change_input<individualRover.sensors.size(); change_input++) {
                        individualRover.sensors.at(change_input) /= number;
                    }// scaling og sensor values
                    
                    for (int temp=0; temp<individualRover.sensors.size(); temp++) {
                        myfile_sensor<<individualRover.sensors.at(temp)<<"\t";
                    }
                    myfile_sensor<<endl;
                    
                    mypop.runNetwork(individualRover.sensors,indiviualNN);
                    double dx = mypop.popVector.at(indiviualNN).outputvaluesNN.at(0);
                    double dy = mypop.popVector.at(indiviualNN).outputvaluesNN.at(1);
                    mypop.popVector.at(indiviualNN).outputvaluesNN.clear();
                    individualRover.move_rover(dx,dy);
                    
                    if(VERBOSE)
                        cout<<individualRover.x_position<<"\t"<<individualRover.y_position<<endl;
                    
                    myfile_x_y<<individualRover.x_position<<"\t"<<individualRover.y_position<<endl;
                    
                    // calculate reward value
                    double x_distance_value = ((individualRover.x_position*individualRover.x_position) -(individualPOI.x_position_poi*individualPOI.x_position_poi));
                    double y_distance_value =((individualRover.y_position*individualRover.y_position) -(individualPOI.y_position_poi*individualPOI.y_position_poi));
                    if (x_distance_value<0) {
                        x_distance_value=(-1)*x_distance_value;
                    }
                    if (y_distance_value<0) {
                        y_distance_value=(-1)*y_distance_value;
                    }
                    double distance = sqrt((x_distance_value)+(y_distance_value));
                    double minimum_value =((1>distance)?1:distance);
                    individualRover.reward = (individualPOI.value_poi/minimum_value);
                    individualRover.indi_reward.push_back(individualRover.reward);
                }
                
                double temp_reward = *max_element(individualRover.indi_reward.begin(), individualRover.indi_reward.end());
                individualRover.all_rewards.push_back(temp_reward);
                individualRover.indi_reward.clear();
                myfile_sensor<<endl;
                myfile_x_y<<endl;
            }
            myfile_sensor.close();
            myfile_x_y.close();
            
            //max_fitness to a text file
            //text file genration number and fitness
            //for (int temp_file_generation=0; temp_file_generation<individualRover.all_rewards.size(); temp_file_generation++) {
                int temp_highest_fitness = *max_element(individualRover.all_rewards.begin(), individualRover.all_rewards.end());
                myfile_generation_fitness<<generation<<"\t"<<temp_highest_fitness<<endl;
            //}
            
            //evolution
            double temp_selection_number= mypop.popVector.size()/2;
            for (int selectNN=0; selectNN<(temp_selection_number); ++selectNN) {
                double temp_random_1 = rand()%mypop.popVector.size();
                double temp_random_2 = rand()%mypop.popVector.size();
                while(temp_random_1==temp_random_2) {
                    temp_random_2 = rand()%mypop.popVector.size();
                }
                if (individualRover.all_rewards.at(temp_random_1)>individualRover.all_rewards.at(temp_random_2)) {
                    //delete neural network temp_random_2
                    mypop.popVector.erase(mypop.popVector.begin()+temp_random_2);
                    individualRover.all_rewards.erase(individualRover.all_rewards.begin()+temp_random_2);
                }else{
                    //delete neural network temp_random_1
                    mypop.popVector.erase(mypop.popVector.begin()+temp_random_1);
                    individualRover.all_rewards.erase(individualRover.all_rewards.begin()+temp_random_1);
                }
            }
            individualRover.all_rewards.clear();
            
            //repop
            vector<unsigned> a;
            for (int temp =0 ; temp<numNN/2; temp++) {
                int R = rand()% mypop.popVector.size();
                Net N(a);
                N=mypop.popVector.at(R);
                N.mutate();
                mypop.popVector.push_back(N);
            }
        }
        myfile_generation_fitness.close();
        
    }
    
    return 0;
}
