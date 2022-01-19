#include <stdio.h>
#include <math.h>
#include"config.h"
#include "Cn2b.h"
#include "find_matched_ins.h"
#pragma warning(disable:4996)

#include <iostream>
using namespace std;

double  d2r = pi / 180, r2d = 180 / pi;
double dh2rs = d2r / 3600;
double acc_sf = 0.05 * pow(2, -15);//加速度比例因子
double gyr_sf = 0.1 * pow(2, -8);//角速度比例因子
double llh0[3] = { 39.95052634181 * d2r,116.330874837810 * d2r,45.6151000000000 };//纬度经度高程
double Lati0 = llh0[0];
double Longi0 = llh0[1];
double Alti0 = llh0[2];


int main()
{
    //---------------Initialize--------------------
    //-------1.imu、uwb初始化------------------

    FILE* RAWIMU_IN = NULL, * UWB = NULL;//imu原始数据
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



    CreateMatrix(&(IMU_info.base), IMU_info.col, IMU_info.row);
    CreateMatrix(&(INS_Time.base), INS_Time.col, INS_Time.row);
    CreateMatrix(&(Deltavel.base), Deltavel.col, Deltavel.row);
    CreateMatrix(&(Deltaang.base), Deltaang.col, Deltaang.row);
    CreateMatrix(&(UWB_info.base), UWB_info.col, UWB_info.row);
    CreateMatrix(&(UWB_Time.base), UWB_Time.col, UWB_Time.row);
    CreateMatrix(&(UWB_d.base), UWB_d.col, UWB_d.row);


    RAWIMU_IN = fopen("imu.txt", "r");//imu文件
    UWB = fopen("uwb2.txt", "r");      //uwb文件
    //-------imu文件读取--------------------
    fseek(RAWIMU_IN, sizeof(int), SEEK_CUR);
    for (int i = 0; i < IMU_LENGTH; i++)//读取imu文件
    {
        for (int j = 0; j < 7; j++)
        {
            fscanf(RAWIMU_IN, "%lf", &IMU_info.base[i][j]);
        }
        fseek(RAWIMU_IN, sizeof(int) + sizeof("\n"), SEEK_CUR);//每一行后有一个回车，占2个字节，加上第一列的4个字节，共6个字节
    }
    fclose(RAWIMU_IN);
    //-------imu文件读取结束-----------------
    //-------uwb文件读取--------------------
    for (int i = 0; i < UWB_LENGTH; i++)
    {
        fscanf(UWB, "%lf", &UWB_info.base[i][0]);
        for (int j = 1; j <= 7; j++)
        {
            fscanf(UWB, "%lf", &UWB_info.base[i][j]);
        }
        //fseek(UWB, sizeof("\n"), SEEK_CUR);//每一行后面有一个回车，需要跳过
    }
    fclose(UWB);
    //-------uwb文件读取结束-----------------
    for (int i = 0; i < 100; i++)
        printf("%f  %f  %f  %f  %f  %f  %f \n", IMU_info.base[i][0], IMU_info.base[i][1], IMU_info.base[i][2], IMU_info.base[i][3], IMU_info.base[i][4], IMU_info.base[i][5], IMU_info.base[i][6]);

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
        UWB_d.base[i][0] = UWB_info.base[i][1] / 1000;
        UWB_d.base[i][1] = UWB_info.base[i][2] / 1000;
        UWB_d.base[i][2] = UWB_info.base[i][3] / 1000;
        UWB_d.base[i][3] = UWB_info.base[i][4] / 1000;
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
    double Cnb[3][3], Q[4], Cbn[3][3];

    Cn2b(yaw, pitch, roll, Cnb, Q);
    //初始化Cbn
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Cbn[i][j] = Cnb[j][i];
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
                         std_acc * std_acc ,std_acc * std_acc ,std_acc * std_acc };
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

    double discreteF[D_X][D_X];
    for (int i = 0; i < D_X; i++)
    {
        for (int j = 0; j < D_X; j++)
        {
            if (i == j)
            {
                discreteF[i][j] = 1;
            }
            else
            {
                discreteF[i][j] = 0;
            }
        }
    }


    double G[D_X][6];
    for (int i = 0; i < D_X; i++)
        for (int j = 0; j < 6; j++)
            G[i][j] = 0;

    double F[D_X][D_X];
    for (int i = 0; i < D_X; i++)
        for (int j = 0; j < D_X; j++)
            F[i][j] = 0;

    double H[D_M][D_X];
    for (int i = 0; i < D_M; i++)
        for (int j = 0; j < D_X; j++)
            H[i][j] = 0;

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
    double ang_1[3] = { 0.0,0.0,0.0 }, vel_1[3] = { 0.0,0.0,0.0 }, ang_2[3] = { 0.0,0.0,0.0 }, vel_2[3] = { 0.0,0.0,0.0 };

    Matrix ins_result;
    ins_result.base = NULL;
    ins_result.row = IMU_LENGTH;
    ins_result.col = 10;
    CreateMatrix(&(ins_result.base), ins_result.col, ins_result.row);

    //----------------------------------大循环开始---------------------------------------------------------------------------------------------------------
    for (int i = 0; i < IMU_LENGTH; i++)
    {
        if (i % 2 == 0)
        {

            for (int j = 0; j < 3; j++)
            {
                ang_1[j] = Deltaang.base[i][j];
                vel_1[j] = Deltavel.base[i][j];
            }
        }
        else {

            for (int j = 0; j < 3; j++)
            {
                ang_2[j] = Deltaang.base[i][j];
                vel_2[j] = Deltavel.base[i][j];
            }
            double vel_scull_b[3];

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

            //-----------------调试点1-----------------------------------------------------------------------------------
            printf("%d  vel_scrull\n", i);

            cout << vel_scull_b[0] << "\t" << vel_scull_b[1] << "\t" << vel_scull_b[2] << endl;
            //---------------调试点1结束---------------------------------------------------------------------------------------

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

            //-----------------调试点2-----------------------------------------------------------------------------------

            printf("%d: before upgrade V\n", i);
            printf("%f\t%f\t%f\n", Vx, Vy, Vz);
            //---------------调试点2结束---------------------------------------------------------------------------------------

            //---------------查看Cnb、Q-------------------------------------------------------------------------------------------
            cout << i << "   " << "Cnb" << " before upgrade" << endl;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    cout << Cnb[i][j] << " ";
                }
                cout << endl;
            }
            cout << i << "    " << "Q" << " before upgrade" << endl;
            for (int i = 0; i < 4; i++)
                cout << Q[i] << " ";
            cout << endl;
            //---------------查看Cnb、Q结束---------------------------------------------

            for (int i = 0; i < 3; i++)
            {
                double sum = 0.0;
                for (int j = 0; j < 3; j++)
                    sum += Cbn[i][j] * vel_scull_b[j];
                V[i] += (sum + g_local[i] * 2 * T);
            }
            V[2] = 0.0;
            Vx = V[0];
            Vy = V[1];
            Vz = V[2];

            //-----------------调试点3-----------------------------------------------------------------------------------
            printf("%d after upgrade V\n", i);
            printf("%f\t%f\t%f\n", Vx, Vy, Vz);
            //---------------调试点3结束---------------------------------------------------------------------------------------

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
            //----------------------修改点2(2022.1.12)-------------------
            //修改原因:QuaternionMatrix建立在单位阵上，之前没注意这一点
            //eye(4)单位阵
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    if (i == j)
                        QuaternionMatrix[i][j] = 1;
                    else
                        QuaternionMatrix[i][j] = 0;

            for (int i = 0; i < 4; i++)
                QuaternionMatrix[i][i] *= (1 - NormSquare / 8.0 + NormSquare * NormSquare / 384.0);

            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    QuaternionMatrix[i][j] += (0.5 - NormSquare / 48.0) * dthetaMatrix[i][j];
            //----------------------修改点2结束-----------------------------
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

            //---------------查看更新后Cnb、Q-------------------------------------------------------------------------------------------
            cout << i << "   " << "Cnb" << " after upgrade" << endl;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    cout << Cnb[i][j] << " ";
                }
                cout << endl;
            }
            cout << i << "    " << "Q" << " after upgrade" << endl;
            for (int i = 0; i < 4; i++)
                cout << Q[i] << " ";
            cout << endl;
            //---------------查看Cnb、Q结束结束---------------------------------------------




            yaw = atan2(Cnb[0][1], Cnb[0][0]) * r2d;
            pitch = asin(-Cnb[0][2]) * r2d;
            roll = atan2(Cnb[1][2], Cnb[2][2]) * r2d;
            // ----------------姿态更新结束-----------------------------------------------
            // ----------------imu_update结束-------------------------------------------

            // ----------------KF开始---------------------------------------------------

            double fned[3];
            for (int i = 0; i < 3; i++)
            {
                double sum = 0.0;
                for (int j = 0; j < 3; j++)
                    sum += Cbn[i][j] * velocity[j];
                fned[i] = sum / (2 * T);
            }

            F[0][0] = 0;
            F[0][1] = -WIE * sin(Lati0);
            F[0][2] = 0;

            F[1][0] = 0;
            F[1][1] = 0;
            F[1][2] = WIE * cos(Lati0);

            F[2][0] = 0;
            F[2][1] = -WIE * cos(Lati0);
            F[2][2] = 0;

            for (int i = 0; i < 3; i++)
            {
                for (int j = 9; j < 12; j++)
                {
                    F[i][j] = -Cbn[i][j - 9];
                }
            }

            F[3][6] = 1;
            F[3][7] = 0;
            F[3][8] = 0;

            F[4][6] = 0;
            F[4][7] = 1;
            F[4][8] = 0;

            F[5][6] = 0;
            F[5][7] = 0;
            F[5][8] = -1;


            F[6][0] = 0;
            F[6][1] = -fned[2];
            F[6][2] = fned[1];

            F[7][0] = fned[2];
            F[7][1] = 0;
            F[7][2] = -fned[0];

            F[8][0] = -fned[1];
            F[8][1] = fned[0];
            F[8][2] = 0;


            F[6][6] = 0;
            F[6][7] = -WIE * sin(Lati0);
            F[6][8] = 0;

            F[7][6] = 2 * WIE * sin(Lati0);
            F[7][7] = 0;
            F[7][8] = 2 * WIE * cos(Lati0);

            F[8][6] = 0;
            F[8][7] = -2 * WIE * cos(Lati0);
            F[8][8] = 0;


            for (int i = 6; i < 9; i++)
            {
                for (int j = 12; j < 15; j++)
                {
                    F[i][j] = Cbn[i - 6][j - 12];
                }
            }

            //离散矩阵

            double phi_5_1[D_X][D_X];
            double matrix_multiple[D_X][D_X]; //存放中间结果

            MultiplyMatrix(F, F, matrix_multiple);

            for (int i = 0; i < D_X; i++)
            {
                for (int j = 0; j < D_X; j++)
                {
                    matrix_multiple[i][j] = matrix_multiple[i][j] * 0.5 * 4 * T * T;
                }
            }


            for (int i = 0; i < D_X; i++)
            {
                for (int j = 0; j < D_X; j++)
                {
                    phi_5_1[i][j] = discreteF[i][j] + F[i][j] * 2 * T + matrix_multiple[i][j];
                }
            }

            MultiplyMatrix(phi_5_1, discreteF, discreteF);


            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    G[i][j] = -Cbn[i][j];
                }
            }

            for (int i = 6; i < 9; i++)
            {
                for (int j = 3; j < 6; j++)
                {
                    G[i][j] = Cbn[i - 6][j - 3];
                }
            }


            int index_UWB;

            if (UWB_num < UWB_LENGTH)
            {
                index_UWB = find_matched_ins(INS_Time, UWB_Time.base[UWB_num - 1][0]);//这里减一是因为matlab下标从1开始
                                                                                      //C的下标从0开始
            }

            int UWB_feedback;
            UWB_feedback = 1;
            cout << "i  " << i << "  index_UWB  " << index_UWB << endl;
            cout << "i " << i << " UWB_num " << UWB_num << endl;

            if (i == index_UWB && UWB_feedback == 1)
            {
                //更改点(2022.1.12)
                //更改原因:transG为G的转置，行列维度要互换，之前没变
                double transG[6][D_X];
                for (int i = 0; i < 6; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        transG[i][j] = G[j][i];
                    }
                }

                double mid[D_X][6];
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        double sum = 0.0;
                        for (int k = 0; k < 6; k++)
                        {
                            sum += G[i][k] * Qkf[k][j];
                        }
                        mid[i][j] = sum;
                    }
                }

                double M1[D_X][D_X];
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        double sum = 0.0;
                        for (int k = 0; k < 6; k++)
                        {
                            sum += mid[i][k] * transG[k][j];
                        }
                        M1[i][j] = sum;
                    }
                }

                double M2[D_X][D_X], M3[D_X][D_X], M4[D_X][D_X],
                    M5[D_X][D_X], M6[D_X][D_X], M7[D_X][D_X], M8[D_X][D_X], M9[D_X][D_X], M10[D_X][D_X];
                double mid1[D_X][D_X], mid2[D_X][D_X];  //存放中间结果,mid1存放F*Mi,mid2存放Mi'*F'
                double transF[D_X][D_X];
                double transMs[D_X][D_X];  //存放M1-M9的转置结果
                MultiplyMatrix(F, M1, mid1);
                TransposeMatrix(F, transF);
                TransposeMatrix(M1, transMs);
                MultiplyMatrix(transMs, transF, mid2);
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        M2[i][j] = mid1[i][j] + mid2[i][j];
                    }
                }

                MultiplyMatrix(F, M2, mid1);
                TransposeMatrix(M2, transMs);
                MultiplyMatrix(transMs, transF, mid2);
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        M3[i][j] = mid1[i][j] + mid2[i][j];
                    }
                }

                MultiplyMatrix(F, M3, mid1);
                TransposeMatrix(M3, transMs);
                MultiplyMatrix(transMs, transF, mid2);
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        M4[i][j] = mid1[i][j] + mid2[i][j];
                    }
                }

                MultiplyMatrix(F, M4, mid1);
                TransposeMatrix(M4, transMs);
                MultiplyMatrix(transMs, transF, mid2);
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        M5[i][j] = mid1[i][j] + mid2[i][j];
                    }
                }

                MultiplyMatrix(F, M5, mid1);
                TransposeMatrix(M5, transMs);
                MultiplyMatrix(transMs, transF, mid2);
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        M6[i][j] = mid1[i][j] + mid2[i][j];
                    }
                }

                MultiplyMatrix(F, M6, mid1);
                TransposeMatrix(M6, transMs);
                MultiplyMatrix(transMs, transF, mid2);
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        M7[i][j] = mid1[i][j] + mid2[i][j];
                    }
                }

                MultiplyMatrix(F, M7, mid1);
                TransposeMatrix(M7, transMs);
                MultiplyMatrix(transMs, transF, mid2);
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        M8[i][j] = mid1[i][j] + mid2[i][j];
                    }
                }

                MultiplyMatrix(F, M8, mid1);
                TransposeMatrix(M8, transMs);
                MultiplyMatrix(transMs, transF, mid2);
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        M9[i][j] = mid1[i][j] + mid2[i][j];
                    }
                }

                MultiplyMatrix(F, M9, mid1);
                TransposeMatrix(M9, transMs);
                MultiplyMatrix(transMs, transF, mid2);
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        M10[i][j] = mid1[i][j] + mid2[i][j];
                    }
                }


                double discreteQ[D_X][D_X];
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        discreteQ[i][j] = M1[i][j] * kfT + M2[i][j] * pow(kfT, 2) / 2 + M3[i][j] * pow(kfT, 3) / 6 + M4[i][j] * pow(kfT, 4) / 24 + M5[i][j] * pow(kfT, 5) / 120 + M6[i][j] * pow(kfT, 6) / 720
                            + M7[i][j] * pow(kfT, 7) / 5040 + M8[i][j] * pow(kfT, 8) / 40320 + M9[i][j] * pow(kfT, 9) / 362880 + M10[i][j] * pow(kfT, 10) / 3628800;
                    }

                }



                double UWB_dis[4];
                for (int i = 0; i < 4; i++)
                {
                    //更改点(2022.1.12)
                    //更改原因:下标问题，UWB_num要减1
                    UWB_dis[i] = UWB_d.base[UWB_num - 1][i];
                }

                double INS_dis[4];
                for (int k = 0; k < Anchor_num; k++)
                {
                    INS_dis[k] = pow(pow(Pos[0] - Anchor[k][0], 2) + pow(Pos[1] - Anchor[k][1], 2) + pow(Pos[2] - Anchor[k][2], 2), 0.5);
                }


                double Y[Anchor_num];
                for (int i = 0; i < Anchor_num; i++)
                {
                    Y[i] = INS_dis[i] - UWB_dis[i];
                }


                double H[D_M][D_X];
                for (int i = 0; i < D_M; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        H[i][j] = 0;
                    }
                }

                for (int k = 0; k < Anchor_num; k++)
                {
                    for (int j = 3; j < 6; j++)
                    {
                        H[k][j] = (Pos[j - 3] - Anchor[k][j - 3]) / UWB_dis[k];
                    }
                }


                double Xfilter[D_X];
                for (int i = 0; i < D_X; i++)
                {
                    Xfilter[i] = 0;
                }

                double Xexpect[D_X];
                for (int i = 0; i < D_X; i++)
                {
                    double sum = 0.0;
                    for (int j = 0; j < D_X; j++)
                    {
                        sum += discreteF[i][j] * Xfilter[j];
                    }
                    Xexpect[i] = sum;
                }

                //-----------------Xexpect首次更新调试点--------------------------------------------
                printf("%d Xexpect\n", i);
                for (int i = 0; i < D_X; i++)
                    cout << Xexpect[i] << " ";
                cout << endl;
                //----------------调试点结束----------------------------------------


                double Pexpect[D_X][D_X];
                double transdiscreteF[D_X][D_X];
                double mid3[D_X][D_X];  //暂存中间结果
                double mid4[D_X][D_X];  //暂存中间结果
                TransposeMatrix(discreteF, transdiscreteF);
                MultiplyMatrix(discreteF, Pfilter, mid3);
                MultiplyMatrix(mid3, transdiscreteF, mid4);
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        Pexpect[i][j] = mid4[i][j] + discreteQ[i][j];
                    }
                }

                double mid5[D_X][D_M];
                double mid6[D_M][D_X];
                double mid7[D_M][D_M];
                double transH[D_X][D_M];
                double inv[D_M][D_M];
                double K[D_X][D_M];

                for (int i = 0; i < D_M; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        transH[j][i] = H[i][j];
                    }

                }


                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_M; j++)
                    {
                        double sum = 0.0;
                        for (int k = 0; k < D_X; k++)
                        {
                            sum += Pexpect[i][k] * transH[k][j];
                        }
                        mid5[i][j] = sum;
                    }
                }



                for (int i = 0; i < D_M; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        double sum = 0.0;
                        for (int k = 0; k < D_X; k++)
                        {
                            sum += H[i][k] * Pexpect[k][j];
                        }
                        mid6[i][j] = sum;
                    }
                }

                for (int i = 0; i < D_M; i++)
                {
                    for (int j = 0; j < D_M; j++)
                    {
                        double sum = 0.0;
                        for (int k = 0; k < D_X; k++)
                        {
                            sum += mid6[i][k] * transH[k][j];
                        }
                        mid7[i][j] = sum;
                    }
                }

                for (int i = 0; i < D_M; i++)
                {
                    for (int j = 0; j < D_M; j++)
                    {
                        mid7[i][j] = mid7[i][j] + R[i][j];
                    }
                }
                //------------矩阵求逆，不确定是否正确------------------

                //------------矩阵求逆：调试----------------------------

                printf("求逆前：\n");
                //double mid7[D_M][D_M];
                for (int i = 0; i < D_M; i++) {
                    for (int j = 0; j < D_M; j++) {
                        printf("%ld ", mid7[i][j]);
                    }
                    printf("\n");
                }

                //InverseMatrix(mid7, inv);
                Mat inter1;
                Mat inter2;
                MatCreate(&inter1, D_M, D_M);

                for (int i = 0; i < D_M; i++) {
                    for (int j = 0; j < D_M; j++) {
                        inter1.element[i][j] = mid7[i][j];
                    }
                }

                MatInv1(&inter1, &inter2);
                
                for (int i = 0; i < D_M; i++) {
                    for (int j = 0; j < D_M; j++) {
                        inv[i][j] = inter2.element[i][j];
                    }
                }


                printf("求逆后：\n");
                //double inv[D_M][D_M];
                for (int i = 0; i < D_M; i++) {
                    for (int j = 0; j < D_M; j++) {
                        printf("%ld ", inv[i][j]);
                    }
                    printf("\n");
                }



                //------------矩阵求逆：调试结束------------------------


                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_M; j++)
                    {
                        double sum = 0.0;
                        for (int k = 0; k < D_M; k++)
                        {
                            sum += mid5[i][k] * inv[k][j];
                        }
                        K[i][j] = sum;
                    }
                }

                double mid8[D_M];
                double mid9[D_X];

                for (int i = 0; i < D_M; i++)
                {
                    double sum = 0.0;
                    for (int k = 0; k < D_X; k++)
                    {
                        sum += H[i][k] * Xexpect[k];
                    }
                    mid8[i] = sum;
                }

                for (int i = 0; i < D_M; i++)
                {
                    mid8[i] = Y[i] - mid8[i];
                }


                //------------------调试点：此处mid8就是Y - H * Xexpect---------------
                printf("%d Y - H * Xexpect\n", i);
                for (int i = 0; i < D_M; i++) {
                    printf("%ld ", mid8[i]);
                }
                cout << endl;
                //------------------调试点结束----------------------------------------

                for (int i = 0; i < D_X; i++)
                {
                    double sum = 0.0;
                    for (int k = 0; k < D_M; k++)
                    {
                        sum += K[i][k] * mid8[k];
                    }
                    mid9[i] = sum;
                }

                for (int i = 0; i < D_X; i++)
                {
                    Xfilter[i] = Xexpect[i] + mid9[i];
                }

                //-----------------Xfilter最后一次更新调试点--------------------------------------------
                printf("%d Xfilter\n", i);
                for (int i = 0; i < D_X; i++)
                    cout << Xfilter[i] << " ";
                cout << endl;
                //----------------调试点结束----------------------------------------


                double eye[D_X][D_X];
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        if (i == j)
                        {
                            eye[i][j] = 1;
                        }
                        else
                        {
                            eye[i][j] = 0;
                        }
                    }
                }


                double mid10[D_X][D_X];
                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        double sum = 0.0;
                        for (int k = 0; k < D_M; k++)
                        {
                            sum += K[i][k] * H[k][j];
                        }
                        mid10[i][j] = sum;
                    }
                }

                for (int i = 0; i < D_X; i++)
                {
                    for (int j = 0; j < D_X; j++)
                    {
                        mid10[i][j] = eye[i][j] - mid10[i][j];
                    }
                }

                MultiplyMatrix(mid10, Pexpect, Pfilter);


                for (int i = 0; i < 3; i++)
                {
                    Pos[i] = Pos[i] - Xfilter[i + 3];
                }

                //double Delta_pos[UWB_LENGTH][3];
                Matrix Delta_pos;
                Delta_pos.base = NULL;
                Delta_pos.row = UWB_LENGTH;
                Delta_pos.col = 3;
                CreateMatrix(&(Delta_pos.base), Delta_pos.col, Delta_pos.row);
                for (int i = 0; i < UWB_LENGTH; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        Delta_pos.base[i][j] = 0;
                    }
                }
                for (int j = 0; j < 3; j++)
                {
                    Delta_pos.base[UWB_num - 1][j] = Xfilter[j + 3];
                }


                //KF后的V
                for (int i = 0; i < 3; i++)
                {
                    V[i] = V[i] - Xfilter[i + 6];
                }


                //double Delta_v[UWB_LENGTH][3];
                Matrix Delta_v;
                Delta_v.base = NULL;
                Delta_v.row = UWB_LENGTH;
                Delta_v.col = 3;
                CreateMatrix(&(Delta_v.base), Delta_v.col, Delta_v.row);
                for (int i = 0; i < UWB_LENGTH; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        Delta_v.base[i][j] = 0;
                    }
                }
                for (int j = 0; j < 3; j++)
                {
                    Delta_v.base[UWB_num - 1][j] = Xfilter[j + 6];
                }



                Vx = V[0];
                Vy = V[1];
                Vz = V[2];



                for (int i = 0; i < 3; i++)
                {
                    err_gyr[i] = err_gyr[i] + Xfilter[i + 9];
                }


                for (int i = 0; i < 3; i++)
                {
                    err_acc[i] = err_acc[i] + Xfilter[i + 12];
                }


                for (int i = 0; i < 3; i++)
                {
                    temp_err_gyr[i] = Xfilter[i + 9];
                }

                for (int i = 0; i < 3; i++)
                {
                    temp_err_acc[i] = Xfilter[i + 12];
                }

                Matrix err_gyr_rec;
                err_gyr_rec.base = NULL;
                err_gyr_rec.row = UWB_LENGTH;
                err_gyr_rec.col = 3;
                CreateMatrix(&(err_gyr_rec.base), err_gyr_rec.col, err_gyr_rec.row);
                for (int i = 0; i < UWB_LENGTH; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        err_gyr_rec.base[i][j] = 0;
                    }
                }
                for (int j = 0; j < 3; j++)
                {
                    err_gyr_rec.base[UWB_num - 1][j] = err_gyr[j];
                }


                Matrix err_acc_rec;
                err_acc_rec.base = NULL;
                err_acc_rec.row = UWB_LENGTH;
                err_acc_rec.col = 3;
                CreateMatrix(&(err_acc_rec.base), err_acc_rec.col, err_acc_rec.row);
                for (int i = 0; i < UWB_LENGTH; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        err_acc_rec.base[i][j] = 0;
                    }
                }
                for (int j = 0; j < 3; j++)
                {
                    err_acc_rec.base[UWB_num - 1][j] = err_acc[j];
                }


                double d_yaw, d_pitch, d_roll;
                d_yaw = Xfilter[2];
                d_pitch = Xfilter[1];
                d_roll = Xfilter[0];


                double d_Cnb[3][3] = { {cos(d_yaw) * cos(d_pitch), sin(d_yaw) * cos(d_pitch), -sin(d_pitch)},
                                 {-sin(d_yaw) * cos(d_roll) + cos(d_yaw) * sin(d_pitch) * sin(d_roll), cos(d_yaw) * cos(d_roll) + sin(d_yaw) * sin(d_pitch) * sin(d_roll),cos(d_pitch) * sin(d_roll)},
                                 {sin(d_yaw) * sin(d_roll) + cos(d_yaw) * sin(d_pitch) * cos(d_roll),-cos(d_yaw) * sin(d_roll) + sin(d_yaw) * sin(d_pitch) * cos(d_roll),cos(d_pitch) * cos(d_roll)}
                };



                double mid11[3][3]; //暂存Cnb * d_Cnb的中间结果
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        double sum = 0.0;
                        for (int k = 0; k < 3; k++)
                        {
                            sum += Cnb[i][k] * d_Cnb[k][j];
                        }
                        mid11[i][j] = sum;
                    }
                }

                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        Cnb[i][j] = mid11[i][j];
                    }
                }

                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        Cbn[i][j] = Cnb[j][i];
                    }
                }

                UWB_num = UWB_num + 1;
                DestroyMatrix(&(Delta_pos.base), Delta_pos.col, Delta_pos.row);
                DestroyMatrix(&(Delta_v.base), Delta_v.col, Delta_v.row);
                DestroyMatrix(&(err_gyr_rec.base), err_gyr_rec.col, err_gyr_rec.row);
                DestroyMatrix(&(err_acc_rec.base), err_acc_rec.col, err_acc_rec.row);
            }


            ins_result.base[count][0] = INS_Time.base[i][0];
            ins_result.base[count][1] = Pos[0];
            ins_result.base[count][2] = Pos[1];
            ins_result.base[count][3] = Pos[2];
            ins_result.base[count][4] = V[0];
            ins_result.base[count][5] = V[1];
            ins_result.base[count][6] = V[2];
            ins_result.base[count][7] = roll;
            ins_result.base[count][8] = pitch;
            ins_result.base[count][9] = yaw;

            printf("%d:Ins_Time\t    Pos[0]\t    Pos[1]\t    Pos[2]\t    V[0]\t    V[1]\t    V[2]\t    roll\t    pitch\t    yaw\n", count);
            printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", ins_result.base[count][0], ins_result.base[count][1], ins_result.base[count][2],
                ins_result.base[count][3], ins_result.base[count][4], ins_result.base[count][5], ins_result.base[count][6],
                ins_result.base[count][7], ins_result.base[count][8], ins_result.base[count][9]);
            count = count + 1;

            //----------------KF结束---------------------------------------------------

        }
    }

    DestroyMatrix(&(IMU_info.base), IMU_info.col, IMU_info.row);
    DestroyMatrix(&(INS_Time.base), INS_Time.col, INS_Time.row);
    DestroyMatrix(&(UWB_Time.base), UWB_Time.col, UWB_Time.row);
    DestroyMatrix(&(UWB_info.base), UWB_info.col, UWB_info.row);
    DestroyMatrix(&(UWB_d.base), UWB_d.col, UWB_d.row);
    DestroyMatrix(&(Deltaang.base), Deltaang.col, Deltaang.row);
    DestroyMatrix(&(Deltavel.base), Deltavel.col, Deltavel.row);
    DestroyMatrix(&(ins_result.base), ins_result.col, ins_result.row);

    return 0;
}
