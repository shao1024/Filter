#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <fstream>
#include <chrono>
using namespace std;

#define TRANS_X_STD_Q 0.5
#define TRANS_Y_STD_Q 0.5
#define velocity_X_STD_Q 0.01
#define velocity_Y_STD_Q 0.01

#define X_STD_R 0.1
#define Y_STD_R 0.1

#define T 1.0/20

typedef struct State{
    float X; /** current x coordinate */
    float Y; /** current y coordinate */
    float Vx; 
    float Vy;

} State;

typedef struct Observation{
    float X0; /** Zk */
    float Y0; /** Zk */

} Observation;


typedef struct particle {
    State cur_Xk;
    State prev_Xk;
    float cur_W; /** weight*/
    float pre_W;
} particle;


class particleFilter_CV {
public:
    particleFilter_CV(int NumOfParticles_temp);
    ~particleFilter_CV();

    
    /** Initializes particles */
    void initParticles(State init_Xk);
    
    /** Moves particles */
    void state_transition();

    /** Updates particle weights */
    void updateWeight(Observation cur_Zk );

    /** Normalize weights of particles */
    void normalizeWeights();
    /** Resamples particles */
    void resample();

    void SystemResample();

    void run(string filename);

    State caculate_cur_state();

    double generateGaussianNoise(double mean, double variance);
    
    int NumOfParticles = 101;
    vector<particle> particles;
private:
    
};
