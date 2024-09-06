#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <random>

using namespace std;

// 生成带噪声的数据
#define TRANS_X_STD_Q 0.5
#define TRANS_Y_STD_Q 0.5
#define velocity_X_STD_Q 0.1
#define velocity_Y_STD_Q 0.1

#define X_STD_R 0.1
#define Y_STD_R 0.1

class CV_State
{
public:
    double X_;
    double Y_;
    double Vx_;
    double Vy_;

    CV_State(double X, double Y, double Vx, double Vy):X_(X),Y_(Y),Vx_(Vx),Vy_(Vy){}
    ~CV_State(){}
};

class CV_MSE
{
public:
    double X_;
    double Y_;
    CV_MSE(double X, double Y):X_(X),Y_(Y){}
    ~CV_MSE(){}
};

// 生成具有给定均值和标准差的高斯分布随机数
double generateGaussianNoise(double mean, double variance) {
    static std::random_device rd;   // 用来获取随机种子
    static std::mt19937 gen(rd());  // 使用 Mersenne Twister 算法作为随机数生成器
    std::normal_distribution<double> dist(mean, variance);  //
    return dist(gen);
}



// 时间间隔T 要生成的数据数量frames 初始化的状态值  保存的路径
void PF_CV_GenData(double T, int frames, class CV_State Init_State, string file_name)
{
    vector<CV_State> ALL_State(frames,CV_State{0,0,0,0});
    vector<CV_MSE> ALL_MSE(frames,CV_MSE{0,0});
    ALL_State[0] = Init_State;
    // CV 模型状态方程
    for (size_t i = 1; i < frames; i++)
    {
        ALL_State[i].X_ = ALL_State[i-1].X_ + ALL_State[i-1].Vx_*T + generateGaussianNoise(0, pow(TRANS_X_STD_Q,2));
        ALL_State[i].Y_ = ALL_State[i-1].Y_ + ALL_State[i-1].Vy_*T + generateGaussianNoise(0, pow(TRANS_Y_STD_Q,2));
        ALL_State[i].Vx_ = ALL_State[i-1].Vx_ ;
        ALL_State[i].Vy_ = ALL_State[i-1].Vy_ ;
        // ALL_State[i].Vx_ = ALL_State[i-1].Vx_ + generateGaussianNoise(0, velocity_X_STD_Q);
        // ALL_State[i].Vy_ = ALL_State[i-1].Vy_ + generateGaussianNoise(0, velocity_Y_STD_Q);
    }
    // CV 观测方程
    for (size_t i = 0; i < frames; i++)
    {
        ALL_MSE[i].X_ = ALL_State[i].X_ + generateGaussianNoise(0, pow(X_STD_R,2));
        ALL_MSE[i].Y_ = ALL_State[i].Y_ + generateGaussianNoise(0, pow(Y_STD_R,2));
    }

    ofstream State_out(file_name + "/frame_state.csv");
    for (size_t i = 0; i < ALL_State.size(); i++)
    {
        State_out << ALL_State[i].X_ << "," << ALL_State[i].Y_ << "," << 
            ALL_State[i].Vx_ << "," << ALL_State[i].Vy_ << endl;
    }
    State_out.close();

    ofstream MSE_out(file_name + "/frame_mse.csv");
    for (size_t i = 0; i < ALL_MSE.size(); i++)
    {
        MSE_out << ALL_MSE[i].X_ << "," << ALL_MSE[i].Y_ << endl;
    }
    MSE_out.close();
    
}

int main()
{
    // string file_name = "/home/ecs-user/Code/Filter/data/EKF_data";
    string file_name = "/home/ecs-user/Code/Filter/data/CV_data";
    double T = 1.0 / 20;
    int frames = 60000;

    
    CV_State Init_State(0,0,11.1,5.5);     
    PF_CV_GenData(T,frames,Init_State,file_name);

    return 0;
}