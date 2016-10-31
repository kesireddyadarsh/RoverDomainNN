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
#include "roverTestFunctions.h"
#include "rewardFunctionTest.h"


//This contains vectors and multiple POI's
#define PI 3.14159265

bool test_rover = true;
bool runNeuralNetwork = true;
bool test_reward = true;
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


void remove_lower_fitness_network(Population * p_Pop,vector<Rover>* p_rover){
    
    bool VERBOSE = false;
    
    //evolution
    double temp_selection_number= p_Pop->popVector.size()/2; //select half the size
    for (int selectNN=0; selectNN<(temp_selection_number); ++selectNN) {
        double temp_random_1 = rand()%p_Pop->popVector.size();
        double temp_random_2 = rand()%p_Pop->popVector.size();
        while(temp_random_1==temp_random_2) {
            temp_random_2 = rand()%p_Pop->popVector.size();
        }
        double random_rover_number = rand()%p_rover->size();
        
        if (p_rover->at(random_rover_number).max_reward.at(temp_random_1)>p_rover->at(random_rover_number).max_reward.at(temp_random_2)) {
            //delete neural network temp_random_2
            p_Pop->popVector.erase(p_Pop->popVector.begin()+temp_random_2);
            p_rover->at(random_rover_number).max_reward.erase(p_rover->at(random_rover_number).max_reward.begin()+temp_random_2);
        }else{
            //delete neural network temp_random_1
            p_Pop->popVector.erase(p_Pop->popVector.begin()+temp_random_1);
            p_rover->at(random_rover_number).max_reward.erase(p_rover->at(random_rover_number).max_reward.begin()+temp_random_1);
        }
        
        
    }
    
    //clear maximum values
    for (int clear_max_vec =0 ; clear_max_vec<p_rover->size(); clear_max_vec++) {
        p_rover->at(clear_max_vec).max_reward.clear();
    }
    
    if(VERBOSE){
        cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$\n\n"<<endl;
        for (int temp_print =0 ; temp_print<p_rover->size(); temp_print++) {
            cout<<"This is size of max reward::"<<p_rover->at(temp_print).max_reward.size()<<endl;
        }
    }
}

void repopulate_neural_networks(int numNN,Population* p_Pop){
    vector<unsigned> a;
    for (int temp =0 ; temp<numNN/2; temp++) {
        int R = rand()% p_Pop->popVector.size();
        Net N(a);
        N=p_Pop->popVector.at(R);
        N.mutate();
        p_Pop->popVector.push_back(N);
    }
}


//This is main function
int main(int argc, const char * argv[]) {\
    bool VERBOSE = false;
    bool seal_team = true;
    srand((unsigned)time(NULL));
    
    if(VERBOSE)
        cout<<"This is function ::"<<__FUNCTION__<<endl;
    
    
    if (test_rover) {
        testRoverSensors test_rover_sensor;
        test_rover_sensor.test_all_sensors();
    }
    
    if (test_reward) {
        rewardFunctionTest reward_function_test;
        reward_function_test.test_reward_function();
    }
    
    if (runNeuralNetwork) {
        if(VERBOSE)
            cout<<"\n\n Neural network"<<endl;
        
        
        //Create numNN of neural network with pointer
        int numNN=3;
        //int numCases = 4;
        vector<unsigned> topology;
        topology.clear();
        topology.push_back(8);
        topology.push_back(10);
        topology.push_back(2);
        Population mypop(numNN,topology);
        Population* mypop_evolution = &mypop;
        
        
        int number_of_rovers = 2;
        int number_of_poi = 4;
        
        //object for environment
        Environment world;
        
        //Set values of poi's
        POI individualPOI;
        individualPOI.x_position_poi_vec.push_back(50.0);
        individualPOI.y_position_poi_vec.push_back(100.0);
        individualPOI.x_position_poi_vec.push_back(100.0);
        individualPOI.y_position_poi_vec.push_back(150.0);
        individualPOI.x_position_poi_vec.push_back(50.0);
        individualPOI.y_position_poi_vec.push_back(150.0);
        individualPOI.x_position_poi_vec.push_back(25.0);
        individualPOI.y_position_poi_vec.push_back(50.0);
        individualPOI.value_poi_vec.push_back(10.0);
        individualPOI.value_poi_vec.push_back(10.0);
        individualPOI.value_poi_vec.push_back(10.0);
        individualPOI.value_poi_vec.push_back(10.0);
        
        double total_value_poi = 0.0;
        for (int t = 0 ; t<individualPOI.value_poi_vec.size(); t++) {
            total_value_poi += individualPOI.value_poi_vec.at(t);
        }
        
        //vectors of rovers
        vector<Rover> teamRover;
        Rover a;
        for (int i=0; i<number_of_rovers; i++) {
            teamRover.push_back(a);
        }
        vector<Rover>* p_rover = &teamRover;
        for (int i=0 ; i<number_of_rovers; i++) {
            teamRover.at(i).x_position_vec.push_back(0+(0.5*i));
            teamRover.at(i).y_position_vec.push_back(0);
        }
        
        //closest distance will be large value and replace with closest value as it rover explores
        for (int k=0; k<number_of_rovers; k++) {
            for (int j=0; j<number_of_poi; j++) {
                teamRover.at(k).closest_dist_to_poi.push_back(99999999.99);
            }
        }
        
        //check if environment along with rovers are set properly
        assert(individualPOI.x_position_poi_vec.size() == individualPOI.y_position_poi_vec.size());
        assert(individualPOI.value_poi_vec.size() == individualPOI.y_position_poi_vec.size());
        assert(individualPOI.value_poi_vec.size() == number_of_poi);
        assert(teamRover.size() == number_of_rovers);
        
        //multiple POI test sensor
        vector<int> index_highest_fitness_iteration;
        vector<double> distance_rover_poi;
        
        //Find scaling number
        double scaling_number = find_scaling_number();
        
        //object for file
        ofstream myfile_generation_fitness;
        myfile_generation_fitness.open("fitness_generation");
        
        //simulation number
        for (int simulation = 0; simulation<2 ; simulation++) {
            //For single neural network
            for (int neural_network_number = 0; neural_network_number<numNN; neural_network_number++) {
                //For single rover
                for (int rover_number =0 ; rover_number<number_of_rovers; rover_number++) {
                    teamRover.at(rover_number).x_position = teamRover.at(rover_number).x_position_vec.at(0);
                    teamRover.at(rover_number).y_position = teamRover.at(rover_number).y_position_vec.at(0);
                    teamRover.at(rover_number).theta = 0.0;
                    //time step
                    for (int time_step =0 ; time_step<200; time_step++) {
                        teamRover.at(rover_number).reset_sensors(); // Reset all sensors
                        teamRover.at(rover_number).sense_all_values(individualPOI.x_position_poi_vec, individualPOI.y_position_poi_vec, individualPOI.value_poi_vec); // sense all values
                        
                        //Change of input values
                        for (int change_sensor_values = 0 ; change_sensor_values <teamRover.at(rover_number).sensors.size(); change_sensor_values++) {
                            teamRover.at(0).sensors.at(change_sensor_values) /= scaling_number;
                        }
                        
                        //Pass sensor values to neural network
                        mypop.runNetwork(teamRover.at(rover_number).sensors, neural_network_number);
                        double dx = mypop.popVector.at(neural_network_number).outputvaluesNN.at(0);
                        double dy = mypop.popVector.at(neural_network_number).outputvaluesNN.at(1);
                        mypop.popVector.at(neural_network_number).outputvaluesNN.clear();
                        
                        //Move Rover
                        teamRover.at(rover_number).move_rover(dx,dy);
                        
                        //Calculate Rover Reward Value
                        //distance_rover_poi contains distance to each poi if its less than 1 then 1 will be considered
                        for (int cal_dis =0; cal_dis<individualPOI.value_poi_vec.size(); cal_dis++) {
                            double x_distance_cal =((teamRover.at(rover_number).x_position) -(individualPOI.x_position_poi_vec.at(cal_dis)));
                            double y_distance_cal = ((teamRover.at(rover_number).y_position) -(individualPOI.y_position_poi_vec.at(cal_dis)));
                            double distance = sqrt((x_distance_cal*x_distance_cal)+(y_distance_cal*y_distance_cal));
                            double cal_value =((1>distance)?1:distance);
                            distance_rover_poi.push_back(cal_value);
                        }
                        
                        //Check to make sure Reward is not exceeding total global award
                        assert(distance_rover_poi.size() == individualPOI.value_poi_vec.size());
                        
                        
                        vector<double> reward_per_step_time;
                        for (int temp_distance_cal = 0 ; temp_distance_cal<distance_rover_poi.size(); temp_distance_cal++) {
                            double per_poi_reward = (individualPOI.value_poi_vec.at(temp_distance_cal)/distance_rover_poi.at(temp_distance_cal));
                            reward_per_step_time.push_back(per_poi_reward);
                        }
                        double temp_local_reward = 0;
                        for (int i =0; i<reward_per_step_time.size(); i++) {
                            temp_local_reward += reward_per_step_time.at(i);
                        }
                        teamRover.at(rover_number).reward = (temp_local_reward);
                        teamRover.at(rover_number).local_time_step_reward.push_back(temp_local_reward);
                        
                        if(VERBOSE)
                            cout<<"This is reward:: "<<temp_local_reward<<" for rover ::"<<rover_number<<endl;
                        
                        reward_per_step_time.clear();
                        
                        //Check Closest Distance
                        assert(teamRover.at(rover_number).closest_dist_to_poi.size() == distance_rover_poi.size());
                        for (int c = 0; c<distance_rover_poi.size(); c++) {
                            if (teamRover.at(rover_number).closest_dist_to_poi.at(c) >= distance_rover_poi.at(c)) {
                                teamRover.at(rover_number).closest_dist_to_poi.at(c) = distance_rover_poi.at(c);
                            }
                        }
                        
                        assert(teamRover.at(rover_number).reward <= total_value_poi);
                        
                        distance_rover_poi.clear();
                        
                    }
                    //max value of reward for time step.
                    double temp_reward = *max_element(teamRover.at(rover_number).local_time_step_reward.begin(), teamRover.at(rover_number).local_time_step_reward.end());
                    teamRover.at(rover_number).max_reward.push_back(temp_reward);
                }
                
                
            }
            
            //clear low fitness
            remove_lower_fitness_network(mypop_evolution, p_rover);
            
            //mutate again
            repopulate_neural_networks(numNN, mypop_evolution);
        }
        
        
    }
    
    
    return 0;
}
