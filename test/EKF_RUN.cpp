#include "KF_CV.h"

int main(int argc, char *argv[]) 
{
    string filename = "/home/ecs-user/Code/Filter/data/CV_data";
    // 构建kf类
    KalmanFilter kf;
    kf.run(filename);

    return 0;
};