#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <vector>
#include <chrono>
using namespace std;
using namespace Eigen;

#define TRANS_X_STD_Q 1.2
#define TRANS_Y_STD_Q 1.2
#define velocity_X_STD_Q 1.2
#define velocity_Y_STD_Q 1.2

#define X_STD_Q 1.2
#define Y_STD_Q 1.2

#define T 1.0/20

class KalmanFilter
{
private:
    int stateSize = 4;
    int measSize = 2;
    // 不考虑外部输入
    int uSize = 0;
    // xk = A*xk-1 + B*uk + wk 
    // 状态量
    Vector4d x;
    // 状态的协方差矩阵
    MatrixXd P;
    // 状态转移矩阵
    MatrixXd A;
    // 控制输入矩阵
    MatrixXd B;
    // 控制输入向量
    MatrixXd u;
    // 状态转移过程中的噪声
    MatrixXd Q;

    // zk = H*xk + vk
    // 观测量
    Vector2d z;
    // 观测矩阵
    MatrixXd H;
    // 观测过程中的噪声矩阵
    MatrixXd R;
    

    
public:
    KalmanFilter();
    ~KalmanFilter();
    void init(Vector4d x_);
    void predict();
    void update(Eigen::Vector2d z_meas);
    void run(string filename);
};


