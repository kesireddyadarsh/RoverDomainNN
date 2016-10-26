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


//This is main function
int main(int argc, const char * argv[]) {\
    cout<<"This is function ::"<<__FUNCTION__<<endl;
    bool VERBOSE = development_tool;
    srand((unsigned)time(NULL));
    
    
    if (test_rover == true) {
        testRoverSensors test_rover_sensor;
        test_rover_sensor.test_all_sensors();
        if (VERBOSE) {
            cout<<"\n\n\n$$$This is end of test all sensors function$$$\n\n"<<endl;
        }
    }
    
    if (test_reward == true) {
        rewardFunctionTest reward_function_test;
        reward_function_test.test_reward_function();
    }
    
    if (runNeuralNetwork == false) {
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
        
        int number_of_rovers = 1;
        int number_of_poi = 2;
        
        Environment world;
        //Set values of poi's
        POI individualPOI;
        individualPOI.x_position_poi_vec.push_back(50.0);
        individualPOI.y_position_poi_vec.push_back(100.0);
        individualPOI.x_position_poi_vec.push_back(100.0);
        individualPOI.y_position_poi_vec.push_back(150.0);
        individualPOI.value_poi_vec.push_back(100.0);
        individualPOI.value_poi_vec.push_back(100.0);
        
        double total_value_poi = 0.0;
        
        for (int t = 0 ; t<individualPOI.value_poi_vec.size(); t++) {
            total_value_poi += individualPOI.value_poi_vec.at(t);
        }
        
        assert(individualPOI.x_position_poi_vec.size() == individualPOI.y_position_poi_vec.size());
        assert(individualPOI.value_poi_vec.size() == individualPOI.y_position_poi_vec.size());
        
        //vectors of rovers
        vector<Rover> teamRover;
        Rover a;
        teamRover.push_back(a);
        
        for (int k=0; k<number_of_rovers; k++) {
            for (int j=0; j<number_of_poi; j++) {
                teamRover.at(k).closest_dist_to_poi.push_back(99999999.99);
            }
        }
        
        //multiple POI test sensor
        vector<int> index_highest_fitness_iteration;
        vector<double> distance_rover_poi;
        
        //Find scaling number
        double scaling_number = find_scaling_number();
        int number = scaling_number;
        
        //object for file
        //file* check out
        //fopen
        ofstream myfile_generation_fitness;
        myfile_generation_fitness.open("fitness_generation");
        for(int generation =0; generation<200;generation++){
            cout<<"Generation::"<<generation<<endl;
            for (int indiviualNN=0; indiviualNN<numNN; indiviualNN++) {
                teamRover.at(0).x_position= 0.0; //x_position of rover
                teamRover.at(0).y_position= 0.0; //y_position of rover
                teamRover.at(0).theta = 0.0 ; //radians
                for (int indiviualNN_iteration=0; indiviualNN_iteration<1000; indiviualNN_iteration++) {
                    teamRover.at(0).reset_sensors();
                    teamRover.at(0).sense_all_values(individualPOI.x_position_poi_vec,individualPOI.y_position_poi_vec,individualPOI.value_poi_vec);
                    
                    for (int change_input=0; change_input<teamRover.at(0).sensors.size(); change_input++) {
                        teamRover.at(0).sensors.at(change_input) /= number;
                    }// scaling og sensor values
                    
                    mypop.runNetwork(teamRover.at(0).sensors, indiviualNN);
                    double dx = mypop.popVector.at(indiviualNN).outputvaluesNN.at(0);
                    double dy = mypop.popVector.at(indiviualNN).outputvaluesNN.at(1);
                    mypop.popVector.at(indiviualNN).outputvaluesNN.clear();
                    
                    teamRover.at(0).move_rover(dx,dy);
                    
                    teamRover.at(0).x_position_rover_iteration_vec.push_back(teamRover.at(0).x_position);
                    teamRover.at(0).y_position_rover_iteration_vec.push_back(teamRover.at(0).y_position);
                    teamRover.at(0).sensor_rover_iteration_vec.push_back(teamRover.at(0).sensors);
                    
                    // calculate reward value
                    for (int cal_dis =0; cal_dis<individualPOI.value_poi_vec.size(); cal_dis++) {
                        double x_distance_cal =((teamRover.at(0).x_position) -(individualPOI.x_position_poi_vec.at(cal_dis)));
                        double y_distance_cal = ((teamRover.at(0).y_position) -(individualPOI.y_position_poi_vec.at(cal_dis)));
                        double distance = sqrt((x_distance_cal*x_distance_cal)+(y_distance_cal*y_distance_cal));
                        double cal_value =((1>distance)?1:distance);
                        distance_rover_poi.push_back(cal_value);
                    }
                    
                    assert(distance_rover_poi.size() == individualPOI.value_poi_vec.size());
                    //distance_rover_poi contains distance to each poi if its less than 1 then 1 will be considered
                    //double local_reward ;
                    vector<double> local_reward_vec;
                    for (int temp_distance_cal = 0 ; temp_distance_cal<distance_rover_poi.size(); temp_distance_cal++) {
                        double per_poi_reward = (individualPOI.value_poi_vec.at(temp_distance_cal)/distance_rover_poi.at(temp_distance_cal));
                        local_reward_vec.push_back(per_poi_reward);
                    }
                    double temp_local_reward = 0;
                    for (int i =0; i<local_reward_vec.size(); i++) {
                        temp_local_reward += local_reward_vec.at(i);
                    }
                    teamRover.at(0).reward = (temp_local_reward);
                    local_reward_vec.clear();
                    
                    //Check Closest Distance
                    assert(teamRover.at(0).closest_dist_to_poi.size() == distance_rover_poi.size());
                    for (int c = 0; c<distance_rover_poi.size(); c++) {
                        if (teamRover.at(0).closest_dist_to_poi.at(c) >= distance_rover_poi.at(c)) {
                            teamRover.at(0).closest_dist_to_poi.at(c) = distance_rover_poi.at(c);
                        }
                    }
                    
                    assert(teamRover.at(0).reward <= total_value_poi);
                    
                    teamRover.at(0).indi_reward.push_back(teamRover.at(0).reward);
                    distance_rover_poi.clear();
                }
                
                double temp_reward = *max_element(teamRover.at(0).indi_reward.begin(), teamRover.at(0).indi_reward.end());
                teamRover.at(0).max_reward.push_back(temp_reward);
                
                assert(temp_reward <= total_value_poi);
                
                
                teamRover.at(0).x_position_rover_nn_vec.push_back(teamRover.at(0).x_position_rover_iteration_vec);
                teamRover.at(0).y_position_rover_nn_vec.push_back(teamRover.at(0).y_position_rover_iteration_vec);
                teamRover.at(0).sensor_rover_nn_vec.push_back(teamRover.at(0).sensor_rover_iteration_vec);
                teamRover.at(0).temp_rewards_nn.push_back(teamRover.at(0).indi_reward);
                teamRover.at(0).x_position_rover_iteration_vec.clear();
                teamRover.at(0).y_position_rover_iteration_vec.clear();
                teamRover.at(0).sensor_rover_iteration_vec.clear();
                teamRover.at(0).indi_reward.clear();
                
                
            }
            
            double temp_highest_fitness = *max_element(teamRover.at(0).max_reward.begin(), teamRover.at(0).max_reward.end());
            myfile_generation_fitness<<generation<<"\t"<<temp_highest_fitness<<endl;
            
            if (generation%5 == 0) {
                //print x y values
                //print sensor values
                
                ofstream myfile_x_y;
                ofstream myfile_sensor;
                int index_nn = 0;
                int index_iteration = 0;
                for (int te = 0; te < teamRover.at(0).temp_rewards_nn.size(); te++) {
                    for (int te_1 =0; te_1<teamRover.at(0).temp_rewards_nn.at(te).size(); te_1++) {// neural network
                        if (temp_highest_fitness == teamRover.at(0).temp_rewards_nn.at(te).at(te_1)) {//iteration
                            index_nn = te;
                            index_iteration = te_1;
                        }
                    }
                }
                
                cout<<"This is network::"<<index_nn<<endl;
                
                //actual print
                myfile_x_y.open("x_y_position_"+to_string(generation)+"_"+to_string(index_nn));
                myfile_sensor.open("sensor_"+to_string(generation)+"_"+to_string(index_nn));
                assert(teamRover.at(0).x_position_rover_nn_vec.size() == teamRover.at(0).y_position_rover_nn_vec.size());
                //assert(teamRover.at(0).x_position_rover_nn_vec.at(index_nn).size() == 100);
                
                for (int temp_print_local=0; temp_print_local<teamRover.at(0).x_position_rover_nn_vec.at(index_nn).size();temp_print_local++) {
                    myfile_x_y<<teamRover.at(0).x_position_rover_nn_vec.at(index_nn).at(temp_print_local)<<"\t"<<teamRover.at(0).y_position_rover_nn_vec.at(index_nn).at(temp_print_local)<<endl;
                }
                for (int temp_print_local=0; temp_print_local<teamRover.at(0).x_position_rover_nn_vec.at(index_nn).size();temp_print_local++) {
                    for (int temp_print =0; temp_print<teamRover.at(0).sensor_rover_nn_vec.at(index_nn).at(temp_print_local).size(); temp_print++) {
                        myfile_sensor<<teamRover.at(0).sensor_rover_nn_vec.at(index_nn).at(temp_print_local).at(temp_print)<<"\t";
                    }
                    myfile_sensor<<endl;
                }
                
                myfile_x_y<<endl;
                myfile_x_y.close();
            }
            
            //evolution
            double temp_selection_number= mypop.popVector.size()/2;
            for (int selectNN=0; selectNN<(temp_selection_number); ++selectNN) {
                double temp_random_1 = rand()%mypop.popVector.size();
                double temp_random_2 = rand()%mypop.popVector.size();
                while(temp_random_1==temp_random_2) {
                    temp_random_2 = rand()%mypop.popVector.size();
                }
                if (teamRover.at(0).max_reward.at(temp_random_1)>teamRover.at(0).max_reward.at(temp_random_2)) {
                    //delete neural network temp_random_2
                    mypop.popVector.erase(mypop.popVector.begin()+temp_random_2);
                    teamRover.at(0).max_reward.erase(teamRover.at(0).max_reward.begin()+temp_random_2);
                }else{
                    //delete neural network temp_random_1
                    mypop.popVector.erase(mypop.popVector.begin()+temp_random_1);
                    teamRover.at(0).max_reward.erase(teamRover.at(0).max_reward.begin()+temp_random_1);
                }
            }
            teamRover.at(0).max_reward.clear();
            
            //repop
            vector<unsigned> a;
            for (int temp =0 ; temp<numNN/2; temp++) {
                int R = rand()% mypop.popVector.size();
                Net N(a);
                N=mypop.popVector.at(R);
                N.mutate();
                mypop.popVector.push_back(N);
            }
            
            teamRover.at(0).temp_sensor_nn.clear();
            teamRover.at(0).temp_rewards_nn.clear();
            teamRover.at(0).x_position_rover_nn_vec.clear();
            teamRover.at(0).y_position_rover_nn_vec.clear();
            teamRover.at(0).sensor_rover_nn_vec.clear();
            teamRover.at(0).indi_reward.clear();
            teamRover.at(0).max_reward.clear();
            
            
        }
    }

    
    return 0;
}
