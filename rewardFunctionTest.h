//
//  rewardFunctionTest.h
//  RoverDomainWithVectors
//
//  Created by ak on 10/26/16.
//  Copyright © 2016 ak. All rights reserved.
//

#ifndef rewardFunctionTest_h
#define rewardFunctionTest_h

//This contains vectors and multiple POI's
#define PI 3.14159265

class rewardFunctionTest{
public:
    void single_rover_multi_poi_reward_test();
    void test_reward_function();
};

void rewardFunctionTest::single_rover_multi_poi_reward_test(){
    bool VERBOSE = false;
    //POI object
    POI P_obj_reward_function;
    Rover R_obj_reward_function;
    
    R_obj_reward_function.x_position = 0.0;
    R_obj_reward_function.y_position = 0.0;
    
    P_obj_reward_function.y_position_poi_vec.push_back(0.5);
    P_obj_reward_function.x_position_poi_vec.push_back(0);
    P_obj_reward_function.value_poi_vec.push_back(100);
    
    for (int j = 1; j<6;j++) {
        P_obj_reward_function.y_position_poi_vec.push_back(j);
        P_obj_reward_function.x_position_poi_vec.push_back(0);
        P_obj_reward_function.value_poi_vec.push_back(100);
        
    }
    
    assert(P_obj_reward_function.x_position_poi_vec.size() == P_obj_reward_function.y_position_poi_vec.size());
    assert(P_obj_reward_function.x_position_poi_vec.size() == P_obj_reward_function.value_poi_vec.size());
    
    vector<double> distance_to_poi;
    
    for (int i=0; i<P_obj_reward_function.x_position_poi_vec.size(); i++) {
        double temp_x = pow(P_obj_reward_function.x_position_poi_vec.at(i)- R_obj_reward_function.x_position, 2);
        double temp_y = pow(P_obj_reward_function.y_position_poi_vec.at(i)- R_obj_reward_function.y_position, 2);
        double temp_distance = sqrt(temp_x+temp_y);
        double temp_push_value = (1>temp_distance)?1:temp_distance;
        distance_to_poi.push_back(temp_push_value);
    }
    
    assert(distance_to_poi.size() == P_obj_reward_function.value_poi_vec.size());
    vector<double> rewards;
    for (int j=0 ; j<distance_to_poi.size(); j++) {
        double temp_value = P_obj_reward_function.value_poi_vec.at(j)/distance_to_poi.at(j);
        rewards.push_back(temp_value);
    }
    if(VERBOSE){
        for (int k=0; k<rewards.size(); k++) {
            cout<<rewards.at(k)<<endl;
        }
    }
    
    
    distance_to_poi.clear();
    rewards.clear();
}

void rewardFunctionTest::test_reward_function(){
    single_rover_multi_poi_reward_test();
}



#endif /* rewardFunctionTest_h */
