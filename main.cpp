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

using namespace std;

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
        z_outputWeights.back().weight = randomWeight();
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
    
        /*assert(temp_inputs.size() == z_layer[0].size()-1);
        for (unsigned i=0; i<temp_inputs.size(); ++i) {
            z_layer[0][i].setOutputVal(temp_inputs[i]);
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
            cout<<"This is value from outputlayer.getourputvalue:::::"<<outputLayer[n].getOutputVal()<<endl;
            double delta = temp_targets[n] - outputLayer[n].getOutputVal();
            cout<<"This is delta value::"<<delta;
            z_error_temp += delta * delta;
        }
        
        z_error_temp /= outputLayer.size() - 1; // get average error squared
        z_error_temp = sqrt(z_error)*100; // RMS
        cout<<"This is z_error_temp::"<<z_error_temp<<endl;
        z_error_vector.push_back(z_error_temp);
        
        temp_targets.clear();*/
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
        cout<<"This is value from outputlayer.getourputvalue:::::"<<outputLayer[n].getOutputVal()<<endl;
        //double delta = temp_targets[n] - outputLayer[n].getOutputVal();
        //cout<<"This is delta value::"<<delta;
        //z_error_temp += delta * delta;
        outputvaluesNN.push_back(outputLayer[n].getOutputVal());
    }
    
}

double Net::backProp(){
    z_error = 0.0;
    for (int temp = 0; temp< z_error_vector.size(); temp++) {
        cout<<"This is z_error_vector"<<temp<<" value::"<< z_error_vector[temp]<<endl;
        z_error += z_error_vector[temp];
    }
    cout<<"This is z_error::"<<z_error<<endl;
    return z_error;
}

/*
 double Net::backProp(vector<double> targetVals, int numCases){
 // Calculate overall net error (RMS of output neuron errors)
 z_error_vector.clear();
 int cycle_target = 0 ;
 while (cycle_target<=(targetVals.size())) {
 int push_vector = cycle_target;
 for ( int temp =0 ; temp<(targetVals.size()/numCases); temp++) {
 temp_targets.push_back(targetVals[push_vector]);
 push_vector++;
 }
 
 Layer &outputLayer = z_layer.back();
 z_error_temp = 0.0;
 
 for (unsigned n = 0; n < outputLayer.size() - 1; ++n) {
 double delta = temp_targets[n] - outputLayer[n].getOutputVal();
 z_error_temp += delta * delta;
 }
 z_error_temp /= outputLayer.size() - 1; // get average error squared
 z_error_temp = sqrt(z_error)*100; // RMS
 z_error_vector.push_back(z_error_temp);
 cycle_target += (targetVals.size()/numCases);
 }
 z_error = 0.0;
 for (int temp = 0; temp< z_error_vector.size(); temp++) {
 z_error += z_error_vector[temp];
 }
 return z_error;
 }
 */

//This is for population of neural network
class Population{
public:
    Population(int numNN,vector<unsigned> topology);
    vector<Net> popVector;
    void runNetwork(vector<double> inputVal);
    void sortError();
    void mutation(int numNN);
    void newerrorvector();
    void findindex();
    int returnIndex(int numNN);
    void repop(int numNN);
    
};

// variables used: indiNet -- object to Net
Population::Population(int numNN,vector<unsigned> topology){
    
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

void Population::runNetwork(vector<double> inputVals){
        popVector.at(0).feedForward(inputVals);
        popVector.at(0).backProp();
}

class Rover{
public:
    double x_position,y_position,x_initial,y_initial;
   // double x_position,y_position;
    vector<double> sensors;
    vector<Net> singleneuralNetwork;
    double sense_poi(double x_position_poi,double y_position_poi);
    double sense_rover(double x_position_otherrover, double y_position_otherrover);
    double initial_sense_poi(double x_position_poi,double y_position_poi);
    double initial_sense_rover(double x_position_otherrover, double y_position_otherrover);
    vector<double> controls;
    void get_all_sensorvalues(double x_position_poi,double y_position_poi,double x_position_otherrover, double y_position_otherrover);
    void get_all_sensorvalues_initial(double x_position_poi,double y_position_poi,double x_position_otherrover, double y_position_otherrover);
    double delta_x,delta_y;
    void teta();
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
    double distance = sqrt(pow(x_position-x_position_otherrover, 2)+pow(y_position-y_position_otherrover, 2));
    delta_sense_rover=(1/distance);
    return delta_sense_rover;
}

void Rover::get_all_sensorvalues(double x_position_poi,double y_position_poi,double x_position_otherrover, double y_position_otherrover){
    for (int i=0; i<4; i++) {
        sensors.push_back(sense_poi(x_position_poi, y_position_poi));
    }
    for (int i=0; i<4; i++) {
        sensors.push_back(sense_poi(x_position_otherrover,y_position_otherrover));
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

void Rover::teta(){
    
}

class POI{
public:
    double x_position_poi,y_position_poi,value_poi;
    //vector<Rover> individualRover;
};

class Environment{
public:
    vector<POI> individualPOI;
};

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
