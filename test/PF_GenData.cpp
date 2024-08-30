#include <iostream>
#include <string>
#include <vector>
#include <fstream>

using namespace std;

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


// 时间间隔T 要生成的数据数量frames 初始化的状态值  保存的路径
void PF_CV_GenData(double T, int frames, class CV_State Init_State, string file_name)
{
    vector<CV_State> ALL_State(frames,CV_State{0,0,0,0});
    vector<CV_MSE> ALL_MSE(frames,CV_MSE{0,0});
    ALL_State[0] = Init_State;
    // CV 模型状态方程
    for (size_t i = 1; i < frames; i++)
    {
        ALL_State[i].X_ = ALL_State[i-1].X_ + ALL_State[i-1].Vx_*T;
        //cout << ALL_State[i-1].Vx_ * T << endl;
        ALL_State[i].Y_ = ALL_State[i-1].Y_ + ALL_State[i-1].Vy_*T;
        ALL_State[i].Vx_ = ALL_State[i-1].Vx_;
        ALL_State[i].Vy_ = ALL_State[i-1].Vy_;
    }
    // CV 观测方程
    for (size_t i = 0; i < frames; i++)
    {
        ALL_MSE[i].X_ = ALL_State[i].X_;
        ALL_MSE[i].Y_ = ALL_State[i].Y_;
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
    string file_name = "/root/Shao/PF/my_pf/data/CV_data";
    double T = 1.0 / 20;
    int frames = 600;
    CV_State Init_State(0,0,11.1,5.5);     
    PF_CV_GenData(T,frames,Init_State,file_name);

    return 0;
}