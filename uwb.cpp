#include <stdio.h>
#include <math.h>
#include <iostream>
#include"config.h"
#include "Cn2b.h"
using namespace std;
#define IMU_LENGTH 82040//imu文件行数
#define UWB_LENGTH 1466//uwb文件行数
#pragma warning(disable : 4996)


#define g0 9.7803267714      //重力加速度
#define pi 3.1415926535897931
#define r0 6378137           //地球长半轴
#define WIE 7.2921151647e-5  //地球自转角速度
#define GM 3.986005e+14      //地球万有引力与地球质量乘积
#define eeee 0.00669438000426//偏心率
#define f (1 / 298.257)      //扁率
#define T 0.005              //IMU频率
#define T_UWB 0.25           //UWB频率
#define Anchor_num 4

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
    Matrix IMU_info ,INS_Time, Deltavel, Deltaang,UWB_info,UWB_Time,UWB_d,Anchor,Cnb,Q, 
           Pfilter, Qkf, G, F, H, R, Xfilter, discreteF,
           angle,velocity, err_gyr, err_acc, temp_err_gyr, temp_err_acc;
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
    Anchor.base = NULL;
    Anchor.row = Anchor_num;
    Anchor.col = 3;
    Qkf.base = NULL;
    Qkf.col = 6;
    Qkf.row = 6;
    

    CreateMatrix(IMU_info);
    CreateMatrix(INS_Time);
    CreateMatrix(Deltavel);
    CreateMatrix(Deltaang);
    CreateMatrix(UWB_info);
    CreateMatrix(UWB_Time);
    CreateMatrix(UWB_d);
    CreateMatrix(Anchor);
    CreateMatrix(Qkf);

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

    Anchor.base[0] = a0;
    Anchor.base[1] = a1;
    Anchor.base[2] = a2;
    Anchor.base[3] = a3;
    
    //姿态初始化
    Cn2b(yaw, pitch, roll, Cnb, Q);
    //位置初始化
    Pos[0] = UWB_info.base[0][5];
    Pos[1] = UWB_info.base[0][6];
    Pos[2] = UWB_info.base[0][7];
    //速度初始化
    Vx = 0.0;
    Vy = -0.001000;
    Vz = 0.0068;
    //-------1.imu、uwb初始化结束------------------

    //-------2.KF初始化----------------------------
    int D_X = 15;//状态向量大小
    int D_M = Anchor_num;//观测向量大小
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

    double Pdiag[15] = { std_roll * std_roll,std_pitch * std_pitch, std_yaw * std_yaw ,std_pos[0] * std_pos[0],
                         std_pos[1] * std_pos[1],std_pos[2] * std_pos[2],std_vel[0] * std_vel[0], std_vel[1] * std_vel[1] ,
                         std_vel[2] * std_vel[2] ,std_gyro * std_gyro ,std_gyro * std_gyro ,std_gyro * std_gyro ,
                         std_acc* std_acc ,std_acc * std_acc ,std_acc * std_acc };
    diag(Pfilter, Pdiag, 15);

    double Qdiag[6] = { std_gyro * std_gyro ,std_gyro * std_gyro ,std_gyro * std_gyro ,
                        std_acc * std_acc ,std_acc * std_acc ,std_acc * std_acc };
    diag(Qkf, Qdiag, 6);

    double Rdiag[4] = { std_UWB_d[0] * std_UWB_d[0],std_UWB_d[1] * std_UWB_d[1],std_UWB_d[2] * std_UWB_d[2],std_UWB_d[3] * std_UWB_d[3] };
    diag(R, Rdiag, 4);

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
    //-------------2.KF初始化结束----------------------

    //-------------3.imu_update and KF-----------------
    int count = 1;
    Matrix ang_1, vel_1, ang_2, vel_2;
    for (int i = 0; i < IMU_LENGTH; i++)
    {
        if (i % 2 == 0)
        {           
            _PartAssign(Deltaang, i, i, 0, 2, ang_1);
            _PartAssign(Deltavel, i, i, 0, 2, vel_1);               
        }
        else {
            _PartAssign(Deltaang, i, i, 0, 2, ang_2);
            _PartAssign(Deltavel, i, i, 0, 2, vel_2);
            Matrix angle,velocity, vel_scull_b;
            angle = Add(ang_1, ang_2);
            velocity = Add(vel_1, vel_2);
            //----vel_scull_b求解中需实现cross向量叉乘，暂没想好-------
            Matrix wien, wenn, winn,winb,g_local;
            double temp = 1 - eeee * sin(Lati0) * sin(Lati0);
            double rn = r0 * (1 - eeee) / pow(temp, 1.5) + Alti0;
            double re = r0 / sqrt(temp) + Alti0;

            wien.base = NULL;
            wien.row = 3;
            wien.col = 1;
            CreateMatrix(wien);
            wenn.base = NULL;
            wenn.row = 3;
            wenn.col = 1;
            CreateMatrix(wenn);
            g_local.base = NULL;
            g_local.row = 3;
            g_local.col = 1;
            CreateMatrix(g_local);
            wien.base[0][0] = WIE * cos(Lati0);
            wien.base[1][0] = 0.0;
            wien.base[2][0] = -WIE * sin(Lati0);
            wenn.base[0][0] = Vy / (re + Alti0);
            wenn.base[1][0] = -Vx / (rn + Alti0);
            wenn.base[2][0] = -Vy * tan(Lati0) / (re + Alti0);
            winn = Add(wien, wenn);
            winb = Mul(Cnb, winn);
            double g_u = g0 * (1 + 0.00193185138639 * sin(Lati0)* sin(Lati0)) / ((1 - 0.00669437999013 * sin(Lati0)* sin(Lati0))* (1 - 0.00669437999013 * sin(Lati0) * sin(Lati0)) * (1.0 + Alti0 / r0)* (1.0 + Alti0 / r0));

        }
    }

    DestroyMatrix(IMU_info);

    return 0;
}

