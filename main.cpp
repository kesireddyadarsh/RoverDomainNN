//
//  main.cpp
//  RoverDomain_CCEA
//
//  Created by ak on 11/16/16.
//  Copyright © 2016 ak. All rights reserved.
//

#include <iostream>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <cassert>

using namespace std;

bool run_simulation = true;
bool test_simulation = true;

#define PI 3.14159265

/*************************
    Neural Network
 ************************/

struct connect{
    double weight;
};

static double random_global(double a) { return a* (rand() / double(RAND_MAX)); }

// This is for each Neuron
class Neuron;
typedef vector<Neuron> Layer;

class Neuron{
public:
    Neuron(unsigned numOutputs, unsigned myIndex);
    vector<connect> z_outputWeights;
    static double randomWeight(void) { return rand() / double(RAND_MAX); }
    unsigned z_myIndex;
    double z_outputVal;
    void setOutputVal(double val) { z_outputVal = val; }
    double getOutputVal(void) const { return z_outputVal; }
    void feedForward(const Layer prevLayer);
    double transferFunction(double x);
};

//This creates connection with neurons.
Neuron::Neuron(unsigned numOutputs, unsigned myIndex){
    for (unsigned c = 0; c < numOutputs; ++c) {
        z_outputWeights.push_back(connect());
        z_outputWeights.back().weight = randomWeight() - 0.5;
    }
    z_myIndex = myIndex;
}

double Neuron::transferFunction(double x){
    return tanh(x);
}

void Neuron::feedForward(const Layer prevLayer){
    double sum = 0.0;
    bool debug_sum_flag = false;
    
    for (unsigned n = 0; n < prevLayer.size(); ++n) {
        if(debug_sum_flag == true){
            cout<<prevLayer[n].getOutputVal()<<endl;
            cout<<&prevLayer[n].z_outputWeights[z_myIndex];
            cout<<prevLayer[n].z_outputWeights[z_myIndex].weight;
        }
        sum += prevLayer[n].getOutputVal() * prevLayer[n].z_outputWeights[z_myIndex].weight;
        //cout<<"This is sum value"<<sum<<endl;
    }
    z_outputVal = Neuron::transferFunction(sum);
}

//This is single neural network
class Net{
public:
    Net(vector<unsigned> topology);
    void feedForward(vector<double> inputVals);
    vector<Layer> z_layer;
    vector<double> outputvaluesNN;
    double backProp();
    double z_error;
    double z_error_temp;
    vector<double> z_error_vector;
    void mutate();
    vector<double> temp_inputs;
    vector<double> temp_targets;
    
    //CCEA
    double fitness;
    vector<double> closest_dist_to_poi;
    
    //For team
    int my_id_wrt_rover;
    int my_team_number;
    int my_rover_number;
    
    //For team
    double local_reward_wrt_team;
    double global_reward_wrt_team;
    double difference_reward_wrt_team;
    
    //Hall of fame
    bool hall_of_fame = false;
    
};

Net::Net(vector<unsigned> topology){
    
    for(int  numLayers = 0; numLayers<topology.size(); numLayers++){
        //unsigned numOutputs = numLayers == topology.size() - 1 ? 0 : topology[numLayers + 1];
        
        unsigned numOutputs;
        if (numLayers == topology.size()-1) {
            numOutputs=0;
        }else{
            numOutputs= topology[numLayers+1];
        }
        
        if(numOutputs>15){
            cout<<"Stop it number outputs coming out"<<numOutputs<<endl;
            exit(10);
        }
        
        z_layer.push_back(Layer());
        
        for(int numNeurons = 0; numNeurons <= topology[numLayers]; numNeurons++){
            //cout<<"This is neuron number:"<<numNeurons<<endl;
            z_layer.back().push_back(Neuron(numOutputs, numNeurons));
        }
    }
}

void Net::mutate(){
    /*
     //popVector[temp].z_layer[temp][temp].z_outputWeights[temp].weight
     */
    for (int l =0 ; l < z_layer.size(); l++) {
        for (int n =0 ; n< z_layer[l].size(); n++) {
            for (int z=0 ; z< z_layer[l][n].z_outputWeights.size(); z++) {
                z_layer[l][n].z_outputWeights[z].weight += random_global(.1)-random_global(.1);
            }
        }
    }
}

void Net::feedForward(vector<double> inputVals){
    
    assert(inputVals.size() == z_layer[0].size()-1);
    for (unsigned i=0; i<inputVals.size(); ++i) {
        z_layer[0][i].setOutputVal(inputVals[i]);
    }
    for (unsigned layerNum = 1; layerNum < z_layer.size(); ++layerNum) {
        Layer &prevLayer = z_layer[layerNum - 1];
        for (unsigned n = 0; n < z_layer[layerNum].size() - 1; ++n) {
            z_layer[layerNum][n].feedForward(prevLayer);
        }
    }
    temp_inputs.clear();
    
    
    Layer &outputLayer = z_layer.back();
    z_error_temp = 0.0;
    
    for (unsigned n = 0; n < outputLayer.size() - 1; ++n) {
        //cout<<"This is value from outputlayer.getourputvalue:::::"<<outputLayer[n].getOutputVal()<<endl;
        //double delta = temp_targets[n] - outputLayer[n].getOutputVal();
        //cout<<"This is delta value::"<<delta;
        //z_error_temp += delta * delta;
        outputvaluesNN.push_back(outputLayer[n].getOutputVal());
    }
    
}

double Net::backProp(){
    z_error = 0.0;
    for (int temp = 0; temp< z_error_vector.size(); temp++) {
        //cout<<"This is z_error_vector"<<temp<<" value::"<< z_error_vector[temp]<<endl;
        z_error += z_error_vector[temp];
    }
    //    cout<<"This is z_error::"<<z_error<<endl;
    return z_error;
}

/***********************
    POI
 **********************/
class POI{
public:
    double x_position_poi,y_position_poi,value_poi;
    //Environment test;
    //vector<Rover> individualRover;
    vector<double> x_position_poi_vec;
    vector<double> y_position_poi_vec;
    vector<double> value_poi_vec;
};

/************************
    Environment
 ***********************/

class Environment{
public:
    vector<POI> individualPOI;
};

/************************
    Rover
 ***********************/

double resolve(double angle);


class Rover{
    //Environment environment_object;
public:
    double x_position,y_position;
    vector<double> x_position_vec,y_position_vec;
    vector<double> sensors;
    vector<Net> singleneuralNetwork;
    void sense_poi(double x, double y, double val);
    void sense_rover(double x, double y);
    double sense_poi_delta(double x_position_poi,double y_position_poi);
    double sense_rover_delta(double x_position_otherrover, double y_position_otherrover);
    vector<double> controls;
    double delta_x,delta_y;
    double theta;
    double phi;
    void reset_sensors();
    int find_quad(double x, double y);
    double find_phi(double x, double y);
    double find_theta(double x_sensed, double y_sensed);
    void move_rover(double dx, double dy);
    double reward =0.0;
    void sense_all_values(vector<double> x_position_poi_vec_rover,vector<double> y_position_poi_vec_rover,vector<double> value_poi_vec_rover);
    
    //stored values
    vector<double> max_reward;
    vector<double> policy;
    //vector<double> best_closest_distance;
    
    //Neural network
    vector<Net> network_for_agent;
    void create_neural_network_population(int numNN,vector<unsigned> topology);
    
    //random numbers for neural networks
    vector<int> random_numbers;
    
};

// variables used: indiNet -- object to Net
void Rover::create_neural_network_population(int numNN,vector<unsigned> topology){
    
    for (int populationNum = 0 ; populationNum<numNN; populationNum++) {
        //cout<<"This is neural network:"<<populationNum<<endl;
        Net singleNetwork(topology);
        network_for_agent.push_back(singleNetwork);
    }
    
}

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
    double deg2rad = 180/PI;
    double phi = (atan2(distance_in_x_phi,distance_in_y_phi) *(deg2rad));
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
    //    cout << "IN QUAD: FIND PHI: " << phi << endl;
    
    phi = resolve(phi);
    
    //    cout << "IN QUAD: FIND PHI2: " << phi << endl;
    
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
    
    //    cout << "QUADANGLE =  " << quadrant_angle << endl;
    //    cout << "QUADRANT = " << quadrant << endl;
    
    return quadrant;
}

void Rover::move_rover(double dx, double dy){
    
    double aom = atan2(dy,dx)*180/PI; /// angle of movement
    double rad2deg = PI/180;
    x_position = x_position + sin(theta*rad2deg) * dy + cos(theta*rad2deg) * dx;
    y_position = y_position + sin(theta*rad2deg) * dx + cos(theta*rad2deg) * dy;
    theta = theta + aom;
    theta = resolve(theta);
    
    //x_position =(x_position)+  (dy* cos(theta*(PI/180)))-(dx *sin(theta*(PI/180)));
    //y_position =(y_position)+ (dy* sin(theta*(PI/180)))+(dx *cos(theta*(PI/180)));
    //theta = theta+ (atan2(dx,dy) * (180 / PI));
    //theta = resolve(theta);
}


//Takes all poi values and update sensor values
void Rover::sense_all_values(vector<double> x_position_poi_vec_rover,vector<double> y_position_poi_vec_rover,vector<double> value_poi_vec_rover){
    bool VERBOSE = false;
    reset_sensors();
    
    double temp_delta_value = 0.0;
    vector<double> temp_delta_vec;
    int temp_quad_value =0;
    vector<double> temp_quad_vec;
    
    assert(x_position_poi_vec_rover.size() == y_position_poi_vec_rover.size());
    assert(value_poi_vec_rover.size() == y_position_poi_vec_rover.size());
    
    for (int value_calculating_delta = 0 ; value_calculating_delta < x_position_poi_vec_rover.size(); value_calculating_delta++) {
        temp_delta_value = sense_poi_delta(x_position_poi_vec_rover.at(value_calculating_delta), y_position_poi_vec_rover.at(value_calculating_delta));
        temp_delta_vec.push_back(temp_delta_value);
    }
    
    for (int value_calculating_quad = 0 ; value_calculating_quad < x_position_poi_vec_rover.size(); value_calculating_quad++) {
        temp_quad_value = find_quad(x_position_poi_vec_rover.at(value_calculating_quad), y_position_poi_vec_rover.at(value_calculating_quad));
        temp_quad_vec.push_back(temp_quad_value);
    }
    
    assert(temp_delta_vec.size()== temp_quad_vec.size());
    
    for (int update_sensor = 0 ; update_sensor<temp_quad_vec.size(); update_sensor++) {
        sensors.at(temp_quad_vec.at(update_sensor)) += value_poi_vec_rover.at(update_sensor)/temp_delta_vec.at(update_sensor);
    }
    
}

/*************************
    Population
 ************************/
//This is for population of neural network
class Population{
public:
    void create_Population(int numNN,vector<unsigned> topology);
    vector<Net> popVector;
    void runNetwork(vector<double> inputVal,int number_neural);
    void sortError();
    void mutation(int numNN);
    void newerrorvector();
    void findindex();
    int returnIndex(int numNN);
    void repop(int numNN);
    
};

// variables used: indiNet -- object to Net
void Population::create_Population(int numNN,vector<unsigned> topology){
    
    for (int populationNum = 0 ; populationNum<numNN; populationNum++) {
        //cout<<"This is neural network:"<<populationNum<<endl;
        Net singleNetwork(topology);
        popVector.push_back(singleNetwork);
    }
    
}

//Return index of higher
int Population::returnIndex(int numNN){
    int temp = numNN;
    int number_1 = (rand() % temp);
    int number_2 = (rand() % temp);
    while (number_1 == number_2) {
        number_2 = (rand() % temp);
    }
    
    if (popVector[number_1].z_error<popVector[number_2].z_error) {
        return number_2;
    }else if (popVector[number_1].z_error>popVector[number_2].z_error){
        return number_1;
    }else{
        return NULL;
    }
}

void Population::repop(int numNN){
    for (int temp =0 ; temp<numNN/2; temp++) {
        int R = rand()% popVector.size();
        popVector.push_back(popVector.at(R));
        popVector.back().mutate();
    }
}

void Population::runNetwork(vector<double> inputVals,int num_neural){
    popVector.at(num_neural).feedForward(inputVals);
    popVector.at(num_neural).backProp();
}

/**************************
Simulation Functions
**************************/
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

/**************************************************
 Below function make sure policies are not same per simulation
 1. generate_random_numbers : generates random numbers until numNN, make sures rovers doesn't have repeated policy numbers (checks columns).
 2. check_policies : checks in each simulation  if two rovers have same policies (checks rows)
 3. generate_random_numbers_policies : if same policies numbers comes in difference rover, this function will change entire random_number for second rover policy
 ************************************************/

void generate_random_numbers(vector<Rover>* teamRover,int number_of_neural_networks){
    //create random numbers
    for (int rover_number = 0; rover_number < teamRover->size(); rover_number++) {
        for (int neural_network = 0; neural_network < number_of_neural_networks ; neural_network++) {
            int random_number = rand()%number_of_neural_networks;
            for (int temp_network =0 ; temp_network < teamRover->at(rover_number).random_numbers.size(); temp_network++) {
                if (random_number == teamRover->at(rover_number).random_numbers.at(temp_network)) {
                    random_number = rand()%number_of_neural_networks;
                    temp_network = -1;
                }
            }
            teamRover->at(rover_number).random_numbers.push_back(random_number);
        }
    }
}

int check_policies(vector<Rover>* teamRover,int number_of_neural_networks){
    for (int neural_network = 0; neural_network<number_of_neural_networks; neural_network++) {
        for (int rover_number =0 ; rover_number < teamRover->size(); rover_number++) {
            for (int other_rover =0 ; other_rover < teamRover->size(); other_rover++) {
                if (other_rover == rover_number) {
                    continue;
                }
                if (teamRover->at(other_rover).random_numbers.at(neural_network) == teamRover->at(rover_number).random_numbers.at(neural_network)) {
                    return other_rover;
                }
            }
        }
    }
    return 1234567;
}

void generate_random_numbers_policies(vector<Rover>* teamRover, int number_of_neural_networks,int policy_number){
    teamRover->at(policy_number).random_numbers.clear();
    for (int neural_network = 0; neural_network < number_of_neural_networks; neural_network++) {
        int random_number = rand()%number_of_neural_networks;
        for (int temp = 0; temp < teamRover->at(policy_number).random_numbers.size() ; temp++) {
            if (random_number == teamRover->at(policy_number).random_numbers.at(temp)) {
                random_number = rand()%number_of_neural_networks;
                temp = -1;
            }
        }
        teamRover->at(policy_number).random_numbers.push_back(random_number);
    }
}

/****************************************************************************
 Same old EA
 **************************************************************************/


void survival_of_fittest(vector<Rover>* teamRover,int number_of_neural_network){
    for (int rover_number =0; rover_number < teamRover->size(); rover_number++) {
        for (int neural_network =0; neural_network < (number_of_neural_network/2); neural_network++) {
            // Generate two random values
            int random_number_1 = rand()%teamRover->at(rover_number).network_for_agent.size();
            int random_number_2 = rand()%teamRover->at(rover_number).network_for_agent.size();
            while (random_number_1 == random_number_2) {
                random_number_1 = rand()%teamRover->at(rover_number).network_for_agent.size();
            }
            
            //check which difference is less
            if (teamRover->at(rover_number).network_for_agent.at(random_number_1).difference_reward_wrt_team < teamRover->at(rover_number).network_for_agent.at(random_number_2).difference_reward_wrt_team) {
                //Kill random_number_1
                teamRover->at(rover_number).network_for_agent.erase(teamRover->at(rover_number).network_for_agent.begin()+random_number_1);
            }else{
                //Kill random_number_2
                teamRover->at(rover_number).network_for_agent.erase(teamRover->at(rover_number).network_for_agent.begin()+random_number_2);
            }
        }
        assert(teamRover->at(rover_number).network_for_agent.size() == (number_of_neural_network/2) );
    }
}

void repopulate(vector<Rover>* teamRover,int number_of_neural_network){
    for (int rover_number =0; rover_number < teamRover->size(); rover_number++) {
        vector<unsigned> a;
        for (int neural_network =0; neural_network < (number_of_neural_network/2); neural_network++) {
            int R = rand()%teamRover->at(rover_number).network_for_agent.size();
            Net N(a);
            N=teamRover->at(rover_number).network_for_agent.at(R);
            N.mutate();
            teamRover->at(rover_number).network_for_agent.push_back(N);
        }
        
        assert(teamRover->at(rover_number).network_for_agent.size() == number_of_neural_network);
    }
}


/********************************************************************************
 assigns_team_number: each policy is giving its team number
 ********************************************************************************/

void assign_team_number(vector<Rover>* teamRover, int number_of_neural_networks){
    for (int rover_number = 0; rover_number < teamRover->size(); rover_number++) {
        for (int neural_network = 0; neural_network < number_of_neural_networks; neural_network++) {
            int temp = teamRover->at(rover_number).random_numbers.at(neural_network);
            teamRover->at(rover_number).network_for_agent.at(neural_network).my_team_number = temp;
        }
    }
}

/********************************************************************************
 simulation: Following things happen here and flow is same
 1. policy numbers are saved into a vector
 2. setting rovers to initial position
 3. moving rovers in each time step and updating its distance to POI
 4. for each policy in team calculate local reward
 5. for each policy in team calculate global reward ( This will be same for each team)
 6. for each policu in team calculate difference reward
 ********************************************************************************/

void simulation( int team_number, vector<Rover>* teamRover, POI* individualPOI,double scaling_number){
    bool VERBOSE = false;
    
    //policy_numbers are pushed into a vector
    vector<int> policy_numbers;
    for (int rover_number = 0 ; rover_number < teamRover->size(); rover_number++) {
        for (int neural_network = 0; neural_network < teamRover->at(rover_number).random_numbers.size(); neural_network++) {
            if (team_number == teamRover->at(rover_number).network_for_agent.at(neural_network).my_team_number) {
                policy_numbers.push_back(neural_network);
                if(VERBOSE)
                    cout<<"This is in policy ::"<<neural_network<<endl;
            }
        }
    }
    if(VERBOSE)
        cout<<"$$$$$$$$$$$$$$$$$$$"<<endl;
    
    //setting all rovers to inital state
    for (int rover_number =0 ; rover_number<teamRover->size(); rover_number++) {
        teamRover->at(rover_number).x_position = teamRover->at(rover_number).x_position_vec.at(0);
        teamRover->at(rover_number).y_position = teamRover->at(rover_number).y_position_vec.at(0);
        teamRover->at(rover_number).theta = 0.0;
    }
    assert(policy_numbers.size() == teamRover->size());
    
    
    /*****************************************************************************
    policy_numbers has policy number to use
    rover_number will be rover number
    First for loop runs time_step second for loop runs size of policy_numbers
    So for each time step each policy from team will execute
    ******************************************************************************/
    for (int time_step = 0 ; time_step < 20; time_step++) {
        for (int rover_number = 0; rover_number < policy_numbers.size(); rover_number++) {
            if(VERBOSE)
            cout<<"This is rover_number::"<<rover_number<<endl;
            if(VERBOSE)
                cout<<"This is time_step ::"<<time_step<<endl;
            //reset_sense_new(rover_number, p_rover, p_poi); // reset and sense new values
            teamRover->at(rover_number).reset_sensors(); // Reset all sensors
            teamRover->at(rover_number).sense_all_values(individualPOI->x_position_poi_vec, individualPOI->y_position_poi_vec, individualPOI->value_poi_vec); // sense all values
            
            //Change of input values
            for (int change_sensor_values = 0 ; change_sensor_values <teamRover->at(rover_number).sensors.size(); change_sensor_values++) {
                teamRover->at(rover_number).sensors.at(change_sensor_values) /= scaling_number;
            }
            
            int temp_variable_nn = policy_numbers.at(rover_number);
            
            if (VERBOSE)
                cout<<"This is temp_variable_nn:::"<<temp_variable_nn<<endl;
            
            teamRover->at(rover_number).network_for_agent.at(temp_variable_nn).feedForward(teamRover->at(rover_number).sensors); // scaled input into neural network
            
            double dx = teamRover->at(rover_number).network_for_agent.at(temp_variable_nn).outputvaluesNN.at(0);
            double dy = teamRover->at(rover_number).network_for_agent.at(temp_variable_nn).outputvaluesNN.at(1);
            teamRover->at(rover_number).network_for_agent.at(temp_variable_nn).outputvaluesNN.clear();
            
            teamRover->at(rover_number).move_rover(dx, dy);
            
            // calculate rover distance for each POI
            for (int cal_dis =0; cal_dis<individualPOI->value_poi_vec.size(); cal_dis++) {
                double x_distance_cal =((teamRover->at(rover_number).x_position) -(individualPOI->x_position_poi_vec.at(cal_dis)));
                double y_distance_cal = ((teamRover->at(rover_number).y_position) -(individualPOI->y_position_poi_vec.at(cal_dis)));
                double distance = sqrt((x_distance_cal*x_distance_cal)+(y_distance_cal*y_distance_cal));
                if (teamRover->at(rover_number).network_for_agent.at(temp_variable_nn).closest_dist_to_poi.at(cal_dis) > distance) {
                    teamRover->at(rover_number).network_for_agent.at(temp_variable_nn).closest_dist_to_poi.at(cal_dis) = distance ;
                }
            }
        }
    }
    
    //check if closest distance changing
    for(int rover_number = 0; rover_number < policy_numbers.size(); rover_number++){
        int temp_policy_number = policy_numbers.at(rover_number);
        for (int check_closest_poi = 0; check_closest_poi < teamRover->at(rover_number).network_for_agent.at(temp_policy_number).closest_dist_to_poi.size(); check_closest_poi++) {
                double temp_var = 99999999.9999;
                assert( teamRover->at(rover_number).network_for_agent.at(temp_policy_number).closest_dist_to_poi.at(check_closest_poi) < temp_var );
        }
    }
    
    /******************************************************************************
     Calculate local reward
     1. for each policy in same team calculate local reward
     ****************************************************************************/
    for (int rover_number = 0 ; rover_number < policy_numbers.size(); rover_number++) {
        int temp_policy_number = policy_numbers.at(rover_number);
        double temp_local_reward = 0;
        for (int closest_distance_number = 0; closest_distance_number<teamRover->at(rover_number).network_for_agent.at(temp_policy_number).closest_dist_to_poi.size(); closest_distance_number++) {
            temp_local_reward += ((individualPOI->value_poi_vec.at(closest_distance_number))/(teamRover->at(rover_number).network_for_agent.at(temp_policy_number).closest_dist_to_poi.at(closest_distance_number)));
        }
        teamRover->at(rover_number).network_for_agent.at(temp_policy_number).local_reward_wrt_team = temp_local_reward;
    }
    
    /******************************************************************************
    Calculate global reward
     1. calculate closest distance to each poi for all policies in same team
     2. calculate global value
     3. assign global reward to each policy in same team
     ****************************************************************************/
    vector<double> closest_distance;
    for (int poi_number = 0; poi_number < individualPOI->value_poi_vec.size(); poi_number++) {
        double temp_best_distance = 9999999.99;
        for (int rover_number = 0; rover_number < policy_numbers.size(); rover_number++) {
            int temp_policy_number = policy_numbers.at(rover_number);
            if (temp_best_distance > teamRover->at(rover_number).network_for_agent.at(temp_policy_number).closest_dist_to_poi.at(poi_number)) {
                temp_best_distance = teamRover->at(rover_number).network_for_agent.at(temp_policy_number).closest_dist_to_poi.at(poi_number);
            }
        }
        closest_distance.push_back(temp_best_distance);
        assert(closest_distance.at(poi_number) < 9999999.99);
    }
    double temp_global = 0.0;
    for (int each_val = 0; each_val < closest_distance.size(); each_val++) {
        temp_global += (closest_distance.at(each_val)/individualPOI->value_poi_vec.at(each_val));
    }
    for (int rover_number = 0; rover_number < policy_numbers.size(); rover_number++) {
        int temp_policy_number = policy_numbers.at(rover_number);
        teamRover->at(rover_number).network_for_agent.at(temp_policy_number).global_reward_wrt_team = temp_global;
    }
    /****************************************************************************
     Calculate difference reward
     1. With out current policy and find the best closest distance
     2. With out current policy find global reward
     3. Difference of gloabl reward and with out current policy global reward is called difference reward
     ***************************************************************************/
    for (int rover_number = 0 ; rover_number < policy_numbers.size(); rover_number++) {
        vector<double> difference_reward_distance;
        int temp_policy_number = policy_numbers.at(rover_number);
        for (int poi_number = 0 ; poi_number < individualPOI->value_poi_vec.size(); poi_number++) {
            double best_distance = 99999999.99;
            for ( int rover_number_next = 0; rover_number_next < policy_numbers.size(); rover_number_next++) {
                int temp_policy_number_next = policy_numbers.at(rover_number_next);
                if (temp_policy_number_next == temp_policy_number) {
                    continue;
                }
                if (best_distance > teamRover->at(rover_number_next).network_for_agent.at(temp_policy_number_next).closest_dist_to_poi.at(poi_number)) {
                    best_distance = teamRover->at(rover_number_next).network_for_agent.at(temp_policy_number_next).closest_dist_to_poi.at(poi_number);
                }
            }
            difference_reward_distance.push_back(best_distance);
        }
        
        //for policy we get best distance of other policies.
        double temp_global_exculding = 0.0;
        for (int temp_each_poi = 0; temp_each_poi < difference_reward_distance.size(); temp_each_poi++) {
            temp_global_exculding += (difference_reward_distance.at(temp_each_poi)/individualPOI->value_poi_vec.at(temp_each_poi));
        }
        teamRover->at(rover_number).network_for_agent.at(temp_policy_number).difference_reward_wrt_team = temp_global_exculding;
        difference_reward_distance.clear();
        
    }
    
    policy_numbers.clear();
    
}

void select_hall_of_fame(vector<Rover>* teamRover,POI* individualPOI){
    for (int rover_number = 0; rover_number <teamRover->size(); rover_number++) {
        double temp_best_global = 0.0;
        int index = 0;
        for (int neural_network = 0; neural_network <teamRover->at(rover_number).network_for_agent.size(); neural_network++) {
            // check for highest value
            if (temp_best_global < teamRover->at(rover_number).network_for_agent.at(neural_network).global_reward_wrt_team) {
                temp_best_global = teamRover->at(rover_number).network_for_agent.at(neural_network).global_reward_wrt_team;
                index = neural_network;
            }
        }
        
        teamRover->at(rover_number).network_for_agent.at(index).hall_of_fame = true;
    }
}

void hall_of_fame_simulation(){
    
}

/***************************
    Main
 **************************/

int main(int argc, const char * argv[]) {
    cout << "Hello, World!\n"<<endl;
    bool VERBOSE = true;
    srand((unsigned)time(NULL));
    if (test_simulation) {
        
    }
    if (run_simulation) {
        if(VERBOSE)
            cout<<"Neural network"<<endl;
        
        //First set up environment
        int number_of_rovers = 2;
        int number_of_poi = 4;
        
        //object for environment
        Environment world;
        Environment* p_world = &world;
        
        //Set values of poi's
        POI individualPOI;
        POI* p_poi = &individualPOI;
        //randomly create x,y positions of rover
        individualPOI.x_position_poi_vec.push_back(50.0);
        individualPOI.y_position_poi_vec.push_back(100.0);
        individualPOI.x_position_poi_vec.push_back(100.0);
        individualPOI.y_position_poi_vec.push_back(150.0);
        individualPOI.x_position_poi_vec.push_back(50.0);
        individualPOI.y_position_poi_vec.push_back(150.0);
        individualPOI.x_position_poi_vec.push_back(25.0);
        individualPOI.y_position_poi_vec.push_back(50.0);
        individualPOI.value_poi_vec.push_back(100.0);
        individualPOI.value_poi_vec.push_back(100.0);
        individualPOI.value_poi_vec.push_back(100.0);
        individualPOI.value_poi_vec.push_back(100.0);
        
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
        
        
        //check if environment along with rovers are set properly
        assert(individualPOI.x_position_poi_vec.size() == individualPOI.y_position_poi_vec.size());
        assert(individualPOI.value_poi_vec.size() == individualPOI.y_position_poi_vec.size());
        assert(individualPOI.value_poi_vec.size() == number_of_poi);
        assert(teamRover.size() == number_of_rovers);
        
        //vector<Population> individual_population_rover; // Individual population is created
        
        
        //Second set up neural networks
        //Create numNN of neural network with pointer
        int numNN=10;
        vector<unsigned> topology;
        topology.clear();
        topology.push_back(8);
        topology.push_back(10);
        topology.push_back(2);
        
        for (int rover_number =0 ; rover_number < number_of_rovers; rover_number++) {
            teamRover.at(rover_number).create_neural_network_population(numNN, topology);
        }
        
        //Number neural networks
        for (int rover_number =0; rover_number< number_of_rovers; rover_number++) {
            for (int neural_network=0; neural_network < numNN; neural_network++) {
                teamRover.at(rover_number).network_for_agent.at(neural_network).my_id_wrt_rover = neural_network;
                teamRover.at(rover_number).network_for_agent.at(neural_network).my_rover_number = rover_number;
            }
        }
        
        //Find scaling number
        double scaling_number = find_scaling_number();
        
        //Setting distance to largest
        for (int network_number =0 ; network_number <numNN; network_number++) {
            for (int rover_number =0; rover_number < number_of_rovers; rover_number++) {
                for (int poi_number =0; poi_number<number_of_poi; poi_number++) {
                    teamRover.at(rover_number).network_for_agent.at(network_number).closest_dist_to_poi.push_back(99999999.9999);
                }
            }
        }
        
        //Select random Neural Networks for each agent
        generate_random_numbers(p_rover,numNN);
        int temp_check_policies = 0;
        while (temp_check_policies != 1234567) {
            temp_check_policies = check_policies(p_rover, numNN);
            if (temp_check_policies != 1234567) {
                generate_random_numbers_policies(p_rover,numNN,temp_check_policies);
            }
        }
        
        //team numbers will be assigned to each agent. Teams are developed on generate_random_numbers function
        assign_team_number(p_rover,numNN);
        
        for (int team_number = 0; team_number<numNN; team_number++) {
            simulation(team_number, p_rover, p_poi, scaling_number);
        }
        
        select_hall_of_fame(p_rover,p_poi);
        
        hall_of_fame_simulation();
        
        if (VERBOSE) {
            FILE* p_output_development;
            p_output_development = fopen("development.txt", "a");
            for (int rover_number = 0; rover_number<number_of_rovers; rover_number++) {
                fprintf(p_output_development, "%d \n",rover_number);
                for (int neural_network = 0; neural_network < numNN; neural_network++) {
                    fprintf(p_output_development, "%d \t %f \t %f \t %f \t %d \n",neural_network, teamRover.at(rover_number).network_for_agent.at(neural_network).local_reward_wrt_team,teamRover.at(rover_number).network_for_agent.at(neural_network).global_reward_wrt_team,teamRover.at(rover_number).network_for_agent.at(neural_network).difference_reward_wrt_team,teamRover.at(rover_number).network_for_agent.at(neural_network).hall_of_fame );
                }
            }
            fclose(p_output_development);
        }
        
        
    
        cout<<"Check"<<endl;
        
    }
    
    return 0;
}
