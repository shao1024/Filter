#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <fstream>
using namespace std;

#define TRANS_X_STD_Q 0.05
#define TRANS_Y_STD_Q 0.03
#define velocity_X_STD_Q 0.001
#define velocity_Y_STD_Q 0.001

#define X_STD_Q 0.01
#define Y_STD_Q 0.01

#define T 1.0/20

typedef struct State{
    float X; /** current x coordinate */
    float Y; /** current y coordinate */
    float Vx; 
    float Vy;

} State;

typedef struct Observation{
    int X0; /** Zk */
    int Y0; /** Zk */

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

    void run(string filename);

    State caculate_cur_state();

    double generateGaussianNoise(double mean, double variance);
    
    int NumOfParticles = 101;
    vector<particle> particles;
private:
    
};
