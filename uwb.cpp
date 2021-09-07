#include <stdio.h>
#include <math.h>
#include <iostream>
#include"config.h"
#include "Cn2b.h"
#include "find_matched_ins.h"


double  d2r = pi / 180, r2d = 180 / pi;
double dh2rs = d2r / 3600;
double acc_sf = 0.05 * pow(2 ,-15);//加速度比例因子
double gyr_sf = 0.1 * pow(2 , -8);//角速度比例因子
double llh0[3] = { 39.95052634181 * d2r,116.330874837810 * d2r,45.6151000000000 };//纬度经度高程
double Lati0 = llh0[0];
double Longi0 = llh0[1];
double Alti0 = llh0[2];


int main()
{
    //---------------Initialize--------------------
    //-------1.imu、uwb初始化------------------
    
    FILE* RAWIMU_IN = NULL,*UWB = NULL;//imu原始数据
    Matrix IMU_info, INS_Time, Deltavel, Deltaang, UWB_info, UWB_Time, UWB_d;
           //Pfilter, Qkf, G, F, H, R, Xfilter, discreteF,
           //angle,velocity, err_gyr, err_acc, temp_err_gyr, temp_err_acc;
    double a0[3] = { 0,0,3.87 }, a1[3] = { -0.305, 32.732,3.610 }, a2[3] = { 6.956,32.732,4.158 }, a3[3] = { 6.977,0, 3.621 };//基站坐标
    double Pos[3];//位置
    double Vx, Vy, Vz;//速度
    double roll = 0.0513703720 * d2r;//滚转角
    double pitch = 0.437961887 * d2r;//俯仰角
    double yaw = (257.540582621) * d2r;//偏航角
    int UWB_num = 2;//做判断
    

    IMU_info.base = NULL;
    IMU_info.row = IMU_LENGTH;
    IMU_info.col = 7;
    INS_Time.base = NULL;
    INS_Time.row = IMU_LENGTH;
    INS_Time.col = 1;
    Deltavel.base = NULL;
    Deltavel.row = IMU_LENGTH;
    Deltavel.col = 3;
    Deltaang.base = NULL;
    Deltaang.row = IMU_LENGTH;
    Deltaang.col = 3;
    UWB_info.base = NULL;
    UWB_info.row = UWB_LENGTH;
    UWB_info.col = 8;
    UWB_Time.base = NULL;
    UWB_Time.row = UWB_LENGTH;
    UWB_Time.col = 1;
    UWB_d.base = NULL;
    UWB_d.row = UWB_LENGTH;
    UWB_d.col = 4;
    
    

    CreateMatrix(IMU_info);
    CreateMatrix(INS_Time);
    CreateMatrix(Deltavel);
    CreateMatrix(Deltaang);
    CreateMatrix(UWB_info);
    CreateMatrix(UWB_Time);
    CreateMatrix(UWB_d);
    

    RAWIMU_IN = fopen("imu.txt", "r");//imu文件
    UWB = fopen("uwb.txt", "r");      //uwb文件
    //-------imu文件读取--------------------
    fseek(RAWIMU_IN, sizeof(int), SEEK_CUR);
    for (int i = 0; i < IMU_LENGTH; i++)//读取imu文件
    {
        for (int j = 0; j < 7; j++)
        {
            fscanf(RAWIMU_IN, "%lf", &IMU_info.base[i][j]);
        }
        fseek(RAWIMU_IN, sizeof(int)+sizeof("\n"), SEEK_CUR);//每一行后有一个回车，占2个字节，加上第一列的4个字节，共6个字节
    }
    fclose(RAWIMU_IN);
    //-------imu文件读取结束-----------------
    //-------uwb文件读取--------------------
    for (int i = 0; i < UWB_LENGTH; i++)
    {
        fscanf(UWB, "%lf", &UWB_info.base[i][0]);
        fseek(UWB, 3 * sizeof(int) + 2 * sizeof(double), SEEK_CUR);
        for (int j = 1; j <= 7; j++)
            fscanf(UWB, "%lf", &UWB_info.base[i][j]);
    }
    fclose(UWB);
    //-------uwb文件读取结束-----------------

    for (int i = 0; i < IMU_LENGTH; i++)
    {
        INS_Time.base[i][0] = IMU_info.base[i][0];
        Deltavel.base[i][0] = -IMU_info.base[i][2] * acc_sf;
        Deltavel.base[i][1] = IMU_info.base[i][3] * acc_sf;
        Deltavel.base[i][2] = -IMU_info.base[i][1] * acc_sf;
        Deltaang.base[i][0] = -IMU_info.base[i][5] * gyr_sf / 3600 * d2r;
        Deltaang.base[i][1] = IMU_info.base[i][6] * gyr_sf / 3600 * d2r;
        Deltaang.base[i][2] = -IMU_info.base[i][4] * gyr_sf / 3600 * d2r;
    }

    for (int i = 0; i < UWB_LENGTH; i++)
    {
        UWB_Time.base[i][0] = UWB_info.base[i][0];
        UWB_d.base[i][0] = UWB_info.base[i][1]/1000;
        UWB_d.base[i][1] = UWB_info.base[i][2]/1000;
        UWB_d.base[i][2] = UWB_info.base[i][3]/1000;
        UWB_d.base[i][3] = UWB_info.base[i][4]/1000;
    }

    //基站坐标
    double Anchor[4][3];
    Anchor[0][0] = 0.0;
    Anchor[0][1] = 0.0;
    Anchor[0][2] = 3.87;
    Anchor[1][0] = -0.305;
    Anchor[1][1] = 32.732;
    Anchor[1][2] = 3.610;
    Anchor[2][0] = 6.956;
    Anchor[2][1] = 32.732;
    Anchor[2][2] = 4.158;
    Anchor[3][0] = 6.977;
    Anchor[3][1] = 0.0;
    Anchor[3][2] = 3.621;
    
    
    //---------姿态初始化---------
    double Cnb[3][3], Q[4];
    Cn2b(yaw, pitch, roll, Cnb, Q);
    //---------位置初始化---------
    Pos[0] = UWB_info.base[0][5];
    Pos[1] = UWB_info.base[0][6];
    Pos[2] = UWB_info.base[0][7];
    //---------速度初始化---------
    double V[3];
    Vx = 0.0;
    Vy = -0.001000;
    Vz = 0.0068;
    V[0] = Vx;
    V[1] = Vy;
    V[2] = Vz;
    //-------1.imu、uwb初始化结束------------------

    //-------2.KF初始化----------------------------
    double kfT = 0.25;//KF频率
    double dprh2rprs = (pi / 180) / sqrt(3600);
    double std_yaw = 2.2 * d2r;
    double std_pitch = 2.2 * d2r;
    double std_roll = 2.2 * d2r;
    double std_gyro = 0.1 * dprh2rprs;
    double std_acc = 5.9e-3 * g0;
    double std_pos[3] = { 10,10,20.0 };
    double std_vel[3] = { 5,5,5 };
    double std_Rab[3] = { 5.0,5.0,5.0 };
    double std_UWB_d[4] = { 0.2,0.2,0.2,0.2 };
    double std_UWB_vel[3] = { 0.1,0.1,0.1 };

    double Pdiag[D_X] = { std_roll * std_roll,std_pitch * std_pitch, std_yaw * std_yaw ,std_pos[0] * std_pos[0],
                         std_pos[1] * std_pos[1],std_pos[2] * std_pos[2],std_vel[0] * std_vel[0], std_vel[1] * std_vel[1] ,
                         std_vel[2] * std_vel[2] ,std_gyro * std_gyro ,std_gyro * std_gyro ,std_gyro * std_gyro ,
                         std_acc* std_acc ,std_acc * std_acc ,std_acc * std_acc };
    double Pfilter[D_X][D_X];
    for (int i = 0; i < D_X; i++)
        Pfilter[i][i] = Pdiag[i];
    //diag(Pfilter, Pdiag, 15);

    double Qdiag[6] = { std_gyro * std_gyro ,std_gyro * std_gyro ,std_gyro * std_gyro ,
                        std_acc * std_acc ,std_acc * std_acc ,std_acc * std_acc };
    double Qkf[6][6];
    for (int i = 0; i < 6; i++)
        Qkf[i][i] = Qdiag[i];
    //diag(Qkf, Qdiag, 6);

    double Rdiag[D_M] = { std_UWB_d[0] * std_UWB_d[0],std_UWB_d[1] * std_UWB_d[1],std_UWB_d[2] * std_UWB_d[2],std_UWB_d[3] * std_UWB_d[3] };
    double R[D_M][D_M];
    for (int i = 0; i < D_M; i++)
        R[i][i] = Rdiag[i];
    //diag(R, Rdiag, 4);
    /*
    eyes(discreteF, D_X);
    zeros(G,D_X, 6);
    zeros(F, D_X, D_X);
    zeros(H, D_M, D_X);
    zeros(Xfilter, D_X, 1);
    zeros(angle, 3, 1);
    zeros(velocity, 3, 1);
    zeros(err_gyr, 3, 1);
    zeros(err_acc, 3, 1);
    zeros(temp_err_gyr, 3, 1);
    zeros(temp_err_acc, 3, 1);
    */
    double discreteF[D_X][D_X];
    for (int i = 0; i < D_X; i++)
        for (int j = 0; j < D_X; j++)
            discreteF[i][j] = 1;

    double G[D_X][6];
    for (int i = 0; i < D_X; i++)
        for (int j = 0; j < 6; j++)
            discreteF[i][j] = 0;

    double F[D_X][D_X];
    for (int i = 0; i < D_X; i++)
        for (int j = 0; j < D_X; j++)
            discreteF[i][j] = 0;

    double H[D_X][D_X];
    for (int i = 0; i < D_M; i++)
        for (int j = 0; j < D_X; j++)
            discreteF[i][j] = 0;

    double Xfilter[D_X];
    for (int i = 0; i < D_X; i++)
         Xfilter[i] = 0;

    double angle[3];
    for (int i = 0; i < 3; i++)
         angle[i] = 0;

    double velocity[3];
    for (int i = 0; i < 3; i++)
        velocity[i] = 0;

    double err_gyr[3];
    for (int i = 0; i < 3; i++)
        err_gyr[i] = 0;

    double err_acc[3];
    for (int i = 0; i < 3; i++)
        err_acc[i] = 0;

    double temp_err_gyr[3];
    for (int i = 0; i < 3; i++)
        temp_err_gyr[i] = 0;

    double temp_err_acc[3];
    for (int i = 0; i < 3; i++)
        temp_err_acc[i] = 0;


    //-------------2.KF初始化结束----------------------

    //-------------3.imu_update and KF-----------------
    //--------------imu_update开始---------------------
    int count = 1;
    double ang_1[3], vel_1[3], ang_2[3], vel_2[3];
    for (int i = 0; i < IMU_LENGTH; i++)
    {
        if (i % 2 == 0)
        {
            //_PartAssign(Deltaang, i, i, 0, 2, ang_1);
            //_PartAssign(Deltavel, i, i, 0, 2, vel_1);
            for (int j = 0; j < 2; j++)
            {
                ang_1[j] = Deltaang.base[i][j];
                vel_1[j] = Deltavel.base[i][j];
            }
        }
        else {
            //_PartAssign(Deltaang, i, i, 0, 2, ang_2);
           // _PartAssign(Deltavel, i, i, 0, 2, vel_2);
            for (int j = 0; j < 2; j++)
            {
                ang_1[j] = Deltaang.base[i][j];
                vel_1[j] = Deltavel.base[i][j];
            }
            double vel_scull_b[3];
            //angle = Add(ang_1, ang_2);
            //velocity = Add(vel_1, vel_2);
            for (int i = 0; i < 3; i++)
                angle[i] = ang_1[i] + ang_2[i];
            for (int i = 0; i < 3; i++)
                velocity[i] = vel_1[i] + vel_2[i];
            double tmp1[3], tmp2[3], tmp3[3];//为求解vel_scull_b而设立的临时变量
            cross(angle, velocity, tmp1);
            cross(ang_1, vel_2, tmp2);
            cross(vel_1, ang_2, tmp3);

            for (int i = 0; i < 3; i++)
                vel_scull_b[i] = velocity[i] + 0.5 * tmp1[i] + 2.0 / 3.0 * (tmp2[i] + tmp3[i]);
           
            double wien[3], wenn[3], winn[3], winb[3], g_local[3];
            double temp = 1 - eeee * sin(Lati0) * sin(Lati0);
            double rn = r0 * (1 - eeee) / pow(temp, 1.5) + Alti0;
            double re = r0 / sqrt(temp) + Alti0;

            wien[0] = WIE * cos(Lati0);
            wien[1] = 0.0;
            wien[2] = -WIE * sin(Lati0);
            wenn[0] = Vy / (re + Alti0);
            wenn[1] = -Vx / (rn + Alti0);
            wenn[2] = -Vy * tan(Lati0) / (re + Alti0);

            //winn = Add(wien, wenn);
            //winb = Mul(Cnb, winn);
            for (int i = 0; i < 3; i++)
                winn[i] = wien[i] + wenn[i];
            for (int i = 0; i < 3; i++)
            {
                double sum = 0.0;
                for (int j = 0; j < 3; j++)
                    sum += Cnb[i][j] * winn[j];
                winb[i] = sum;
            }
            double g_u = g0 * (1 + 0.00193185138639 * sin(Lati0) * sin(Lati0)) / ((1 - 0.00669437999013 * sin(Lati0) * sin(Lati0)) * (1 - 0.00669437999013 * sin(Lati0) * sin(Lati0)) * (1.0 + Alti0 / r0) * (1.0 + Alti0 / r0));
            g_local[0] = 0;
            g_local[1] = 0;
            g_local[2] = g_u;

            //---------速度更新---------------
            for (int i = 0; i < 3; i++)
            {
                double sum = 0.0;
                for (int j = 0; j < 3; j++)
                    sum += Cnb[i][j] * vel_scull_b[j];
                V[i] += (sum + g_local[i] * 2 * T);
            }
            V[2] = 0.0;
            Vx = V[0];
            Vy = V[1];
            Vz = V[2];
            //----------速度更新结束----------

            //----------位置更新--------------
            Pos[0] += 2 * T * Vx;
            Pos[1] += 2 * T * Vy;
            Pos[2] -= 2 * T * Vz;
            //----------位置更新结束----------

            //----------姿态更新--------------
            for (int i = 0; i < 3; i++)
            {
                ang_1[i] -= winb[i] * T;
                ang_2[i] -= winb[i] * T;
            }
            double TurnVector[3];
            double tmp[3];
            cross(ang_1, ang_2, tmp);
            for (int i = 0; i < 3; i++)
                TurnVector[i] = ang_1[i] + ang_2[i] + 2.0 / 3.0 * tmp[i];
            double wnnb[3];
            for (int i = 0; i < 3; i++)
                wnnb[i] = TurnVector[i] / (2 * T);
            double NormSquare = TurnVector[0] * TurnVector[0] + TurnVector[1] * TurnVector[1] + TurnVector[2] * TurnVector[2];
            double dthetaMatrix[4][4] = { {0, -TurnVector[0], -TurnVector[1], -TurnVector[2]},
                                       {TurnVector[0], 0,TurnVector[2] ,-TurnVector[1] },
                                       {TurnVector[1], -TurnVector[2], 0, TurnVector[0]},
                                       {TurnVector[2], TurnVector[1], -TurnVector[0], 0}
            };
            double QuaternionMatrix[4][4];
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    QuaternionMatrix[i][j] = (1 - NormSquare / 8.0 + NormSquare * NormSquare / 384.0) + (0.5 - NormSquare / 48.0) * dthetaMatrix[i][j];
            double Q_update[4];
            for (int i = 0; i < 4; i++)
            {
                double sum = 0.0;
                for (int j = 0; j < 4; j++)
                    sum += QuaternionMatrix[i][j] * Q[j];
                Q_update[i] = sum;
            }
            for (int i = 0; i < 4; i++)
                Q[i] = Q_update[i] / norm(Q_update, 4);
            double q11 = Q[0] * Q[0], q12 = Q[0] * Q[1], q13 = Q[0] * Q[2], q14 = Q[0] * Q[3];
            double q22 = Q[1] * Q[1], q23 = Q[1] * Q[2], q24 = Q[1] * Q[3];
            double q33 = Q[2] * Q[2], q34 = Q[2] * Q[3];
            double q44 = Q[3] * Q[3];
            double Cbn[3][3] = { {q11 + q22 - q33 - q44, 2 * (q23 - q14), 2 * (q24 + q13)},
                                 {2 * (q23 + q14),q11 - q22 + q33 - q44,2 * (q34 - q12)},
                                 {2 * (q24 - q13),2 * (q34 + q12),q11 - q22 - q33 + q44}
            };
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    Cnb[i][j] = Cbn[j][i];
            yaw = atan2(Cnb[0][1], Cnb[0][0]) * r2d;
            pitch = asin(-Cnb[0][2]) * r2d;
            roll = atan2(Cnb[1][2], Cnb[2][2]) * r2d;
            //----------------姿态更新结束-----------------------------------------------
            //----------------imu_update结束-------------------------------------------

            //----------------KF开始---------------------------------------------------

            //----------------KF结束---------------------------------------------------
        }
    }

    DestroyMatrix(IMU_info);
    DestroyMatrix(INS_Time);
    DestroyMatrix(UWB_Time);
    DestroyMatrix(UWB_info);
    DestroyMatrix(UWB_d);
    DestroyMatrix(Deltaang);
    DestroyMatrix(Deltavel);

    return 0;
}

