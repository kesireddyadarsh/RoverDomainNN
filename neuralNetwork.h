//
//  neuralNetwork.h
//  roverDomainNN
//
//  Created by ak on 9/26/16.
//  Copyright © 2016 ak. All rights reserved.
//

#ifndef neuralNetwork_h
#define neuralNetwork_h
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



#endif /* neuralNetwork_h */
