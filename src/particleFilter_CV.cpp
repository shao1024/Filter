#include "particleFilter_CV.h"
#include <sstream>
particleFilter_CV::particleFilter_CV(int NumOfParticles_temp): NumOfParticles(NumOfParticles_temp)
{

}

particleFilter_CV::~particleFilter_CV() {
    
}


/** Initializes particles */
void particleFilter_CV::initParticles( State init_Xk)
{
    particles.resize(NumOfParticles);
    // 初始化了每一个粒子
    for (int i=0; i < NumOfParticles; i++) {
        particles[i].prev_Xk.X =  init_Xk.X;
        particles[i].prev_Xk.Y =  init_Xk.Y;
        particles[i].prev_Xk.Vx =  init_Xk.Vx;
        particles[i].prev_Xk.Vy =  init_Xk.Vy;

        particles[i].pre_W = 1.0/NumOfParticles;
    }
}


void particleFilter_CV::state_transition() {
    for (int i=0; i < NumOfParticles; i++) {
        particles[i].cur_Xk.X = particles[i].prev_Xk.X + particles[i].prev_Xk.Vx * T + generateGaussianNoise(0, pow(TRANS_X_STD_Q,2));
        particles[i].cur_Xk.Y = particles[i].prev_Xk.Y + particles[i].prev_Xk.Vy * T + generateGaussianNoise(0, pow(TRANS_Y_STD_Q,2));
        particles[i].cur_Xk.Vx = particles[i].prev_Xk.Vx + generateGaussianNoise(0, pow(velocity_X_STD_Q,2));
        particles[i].cur_Xk.Vy = particles[i].prev_Xk.Vy + generateGaussianNoise(0, pow(velocity_Y_STD_Q,2));

        particles[i].prev_Xk = particles[i].cur_Xk;
    }
}

void particleFilter_CV::updateWeight(Observation cur_Zk ) {
    for (int i=0; i < NumOfParticles; i++) 
    {
        Observation temp;
        temp.X0 = particles[i].cur_Xk.X + generateGaussianNoise(0, pow(X_STD_R,2));
        temp.Y0 = particles[i].cur_Xk.Y + generateGaussianNoise(0, pow(Y_STD_R,2));
        particles[i].cur_W = 1.0 /(sqrt(2 * M_PI) * X_STD_R) * exp(- pow(cur_Zk.X0 - temp.X0, 2) / (2 * pow(X_STD_R, 2)) ) * 
            1.0 /(sqrt(2 * M_PI) * Y_STD_R) * exp(- pow(cur_Zk.Y0 - temp.Y0, 2) / (2 * pow(Y_STD_R, 2)) ) * particles[i].pre_W ;
        
        particles[i].pre_W = particles[i].cur_W;
    }
}

void particleFilter_CV::normalizeWeights() {
    float sum_cur = 0;
    float sum_pre = 0;
    for (int i=0; i < NumOfParticles; i++) {
        sum_cur += particles[i].cur_W;
        sum_pre += particles[i].pre_W;
    }
    
    for (int i=0; i < NumOfParticles; i++) {
        particles[i].cur_W /= sum_cur;
        particles[i].pre_W /= sum_pre;
    }
}

void particleFilter_CV::resample() {
    int np, k = 0;
    vector<particle> newParticles;
    newParticles.resize(NumOfParticles);
    std::sort(particles.begin(), particles.end(), [](particle a, particle b) { return a.cur_W > b.cur_W;} );
    
    for (int i=0; i < NumOfParticles; i++) {
        np = particles[i].cur_W * NumOfParticles;
        for (int j=0; j < np; j++) {
            newParticles[k++] = particles[i];
            if (k == NumOfParticles) 
            {
                break;
            }
        }
        if (k == NumOfParticles) 
        {
                break;
        }
    }
    
    while (k < NumOfParticles) {
        newParticles[k++] = particles[0];
    }
    
    for (int i=0; i < NumOfParticles; i++) {
        particles[i] = newParticles[i];
    }
    // 将权重归一化
    //normalizeWeights();
    for(int i=0; i<particles.size(); i++)
    {
        particles[i].cur_W = particles[i].pre_W = 1.0 / NumOfParticles;
    }

}

void particleFilter_CV::SystemResample()
{
    
    vector<particle> newParticles;
    std::vector<double> cumulativeSum(particles.size());
    double totalWeight = 0.0;

    // 计算总权重和累积权重
    for (size_t i = 0; i < particles.size(); ++i) {
        totalWeight += particles[i].cur_W;
        cumulativeSum[i] = (i == 0) ? particles[i].cur_W : cumulativeSum[i - 1] + particles[i].cur_W;
    }

    // 归一化累积权重
    for (size_t i = 0; i < cumulativeSum.size(); ++i) {
        cumulativeSum[i] /= totalWeight;
    }

    std::vector<int> resampledIndices(particles.size());
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    double u;
    for (size_t i = 0; i < particles.size(); ++i) {
        u = dis(gen);
        auto it = std::lower_bound(cumulativeSum.begin(), cumulativeSum.end(), u);
        newParticles.push_back(particles[ it - cumulativeSum.begin()]);
    }

    particles.clear();
    particles = newParticles;

    for(int i=0; i<particles.size(); i++)
    {
        particles[i].cur_W = particles[i].pre_W = 1.0 / NumOfParticles;
    }

}


State particleFilter_CV::caculate_cur_state()
{
    float sum_X = 0,sum_Y = 0,sum_Vx = 0,sum_Vy = 0;
    for(int i=0; i<NumOfParticles; i++)
    {
        sum_X += particles[i].cur_Xk.X * particles[i].cur_W;
        sum_Y += particles[i].cur_Xk.Y * particles[i].cur_W;
        sum_Vx += particles[i].cur_Xk.Vx * particles[i].cur_W;
        sum_Vy += particles[i].cur_Xk.Vy * particles[i].cur_W;        
    }
    return State{sum_X,sum_Y,sum_Vx,sum_Vy};
    
}


void particleFilter_CV::run(string filename)
{
    // 为初始状态赋值
    State Init_state;
    ifstream frame_state(filename + "/frame_state.csv" );
    if (!frame_state) {
        cout << "Error opening file" << endl;
        return;
    }   
    std::string line;
    if (std::getline(frame_state, line)) 
    {
        std::istringstream iss(line);
        string temp_x;
        getline(iss,temp_x, ',');
        Init_state.X = stof(temp_x);

        string temp_y;
        getline(iss,temp_y, ',');
        Init_state.Y = stof(temp_y);

        string temp_vx;
        getline(iss,temp_vx, ',');
        Init_state.Vx = stof(temp_vx);

        string temp_vy;
        getline(iss,temp_vy, ',');
        Init_state.Vy = stof(temp_vy);
        
    }
    frame_state.close();

    vector<Observation> ALL_MSE;
    ifstream frame_mse(filename + "/frame_mse.csv" );
    if (!frame_mse) {
        cout << "Error opening file" << endl;
        return;
    }   
    std::string line_mse;
    while (std::getline(frame_mse, line)) {
        std::istringstream iss(line);
        Observation temp;

        string temp_x;
        getline(iss,temp_x, ',');
        temp.X0 = stof(temp_x);

        string temp_y;
        getline(iss,temp_y, ',');
        temp.Y0 = stof(temp_y);

        ALL_MSE.push_back(temp);
    }
    frame_mse.close();

    initParticles(Init_state);

    vector<State> ALL_State;
    ALL_State.push_back(Init_state);

    auto start = std::chrono::high_resolution_clock::now();
    for (int i=1; i < ALL_MSE.size(); i++) {
        state_transition();
        // 为观测赋值
        updateWeight(ALL_MSE[i]);
        normalizeWeights();
        resample();
        
        ALL_State.push_back(caculate_cur_state());
    }
    auto end = std::chrono::high_resolution_clock::now();

     // 计算时间差
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Time taken by PF: " << duration.count() << " microseconds" << std::endl;



    ofstream State_out(filename + "/PF_State_Out.csv");
    for (size_t i = 0; i < ALL_State.size(); i++)
    {
        State_out << ALL_State[i].X << "," << ALL_State[i].Y << "," << 
            ALL_State[i].Vx << "," << ALL_State[i].Vy << endl;
    }
    State_out.close();
   
}




// 生成具有给定均值和标准差的高斯分布随机数
double particleFilter_CV::generateGaussianNoise(double mean, double variance) {
    static std::random_device rd;   // 用来获取随机种子
    static std::mt19937 gen(rd());  // 使用 Mersenne Twister 算法作为随机数生成器
    std::normal_distribution<double> dist(mean, variance);  //
    return dist(gen);
}


