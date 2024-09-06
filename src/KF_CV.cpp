#include "KF_CV.h"

// 状态量4维 观测量2维
KalmanFilter::KalmanFilter()
{
    
}

KalmanFilter::~KalmanFilter()
{

}
 
void KalmanFilter::init(Vector4d x_) {
    // 状态量 [X Y Vx Vy]
    // 观测 [X Y]
    if (stateSize == 0 || measSize == 0) {
        cout << "Error" << endl;
    }
 
    // 状态量x获取初始状态值
    x = x_;
    
    // 状态转移矩阵
    A.resize(stateSize, stateSize);
    A << 1 , 0 , T , 0 ,
         0 , 1 , 0 , T ,
         0 , 0 , 1 , 0 ,
         0 , 0 , 0 , 1 ;
 
    // 初始状态X的协方差矩阵
    P.resize(stateSize, stateSize);
    P.setIdentity();
 
    // 观测矩阵
    H.resize(measSize, stateSize);
    H << 1 , 0 , 0 , 0 ,
         0 , 1 , 0 , 0 ;
         
    // 观测值
    z.resize(measSize);
    z.setZero();
 
    // 状态转移噪声矩阵 Q
    Q.resize(stateSize, stateSize);
    Q << pow(TRANS_X_STD_Q, 2) , 0 , 0 , 0 ,
         0 , pow(TRANS_Y_STD_Q, 2) , 0 , 0 ,
         0 , 0 , pow(velocity_X_STD_Q, 2) , 0 ,
         0 , 0 , 0 , pow(velocity_Y_STD_Q, 2) ;

    // 测量噪声矩阵 R
    R.resize(measSize, measSize);
    R << pow(X_STD_Q, 2) , 0 ,
         0 , pow(Y_STD_Q, 2) ;
}
 
void KalmanFilter::predict() {

    x = A * x ;
    MatrixXd A_t = A.transpose();
    P = A * P * A_t + Q;
   
}
 
void KalmanFilter::update(Eigen::Vector2d z_meas) {
    MatrixXd temp1, temp2, H_t;
    H_t = H.transpose();
    temp1 = H * P * H_t + R;
    temp2 = temp1.inverse();
    MatrixXd K = P * H_t * temp2;
    z = H * x;
    x = x + K * (z_meas - z);
    MatrixXd I = MatrixXd::Identity(stateSize, stateSize);
    P = (I - K * H) * P;
   
}

void KalmanFilter::run(string filename)
{
    ifstream frame_state(filename + "/frame_state.csv" );
    if (!frame_state) {
        cout << "Error opening file" << endl;
        return;
    }

    std::string line;
    if (std::getline(frame_state, line)) 
    {
        double X0, Y0, Vx0, Vy0;
        std::istringstream iss(line);
        string temp_x;

        
        getline(iss,temp_x, ',');
        X0 = stod(temp_x);

        string temp_y;
        getline(iss,temp_y, ',');
        Y0 = stod(temp_y);

        string temp_vx;
        getline(iss,temp_vx, ',');
        Vx0 = stod(temp_vx);

        string temp_vy;
        getline(iss,temp_vy, ',');
        Vy0 = stod(temp_vy);
        
        // 初始化状态
        init(Vector4d{X0, Y0, Vx0, Vy0});
    }
    frame_state.close();

    vector<Vector2d> ALL_MSE;
    ifstream frame_mse(filename + "/frame_mse.csv" );
    if (!frame_mse) {
        cout << "Error opening file" << endl;
        return;
    }   
    std::string line_mse;
    while (std::getline(frame_mse, line_mse)) {
        std::istringstream iss(line_mse);
        double Xz,Yz;

        string temp_x;
        getline(iss,temp_x, ',');
        Xz = stod(temp_x);

        string temp_y;
        getline(iss,temp_y, ',');
        Yz = stod(temp_y);

        ALL_MSE.emplace_back(Xz, Yz);
    }
    frame_mse.close();

    vector<Vector4d> ALL_State;
    ALL_State.emplace_back(x);
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 1; i < ALL_MSE.size(); i++)
    {
        predict();
        update(ALL_MSE[i]);
        ALL_State.emplace_back(x);
    }
    auto end = std::chrono::high_resolution_clock::now();
    // 计算时间差
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Time taken by KF: " << duration.count() << " microseconds" << std::endl;


    ofstream State_out(filename + "/EKF_State_Out.csv");
    for (size_t i = 0; i < ALL_State.size(); i++)
    {
        State_out << ALL_State[i][0]<< "," << ALL_State[i][1] << "," << 
            ALL_State[i][2] << "," << ALL_State[i][3] << endl;
    }
    State_out.close();
    
}



