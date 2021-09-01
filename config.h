#pragma once
#include <math.h>
#define IMU_LENGTH 82040//imu文件行数
#define UWB_LENGTH 1466//uwb文件行数
#define g0 9.7803267714      //重力加速度
#define pi 3.1415926535897931
#define r0 6378137           //地球长半轴
#define WIE 7.2921151647e-5  //地球自转角速度
#define GM 3.986005e+14      //地球万有引力与地球质量乘积
#define eeee 0.00669438000426//偏心率
#define ff (1 / 298.257)      //扁率,系统中已有f常量
#define T 0.005              //IMU频率
#define T_UWB 0.25           //UWB频率
#define Anchor_num 4
#define D_X 15//状态向量大小
#define D_M 4//观测向量大小

extern double  d2r ;
extern double dh2rs;
extern double acc_sf ;//加速度比例因子
extern double gyr_sf ;//角速度比例因子
extern double llh0[3] ;//纬度经度高程
extern double Lati0 ;
extern double Longi0;
extern double Alti0 ;


typedef struct matrix {
	double** base;//首地址
	int row;      //行数
	int col;      //列数
}Matrix;

void CreateMatrix(Matrix& m);//创建矩阵
void CreateMatrix(double** m, int col, int row);
void DestroyMatrix(Matrix& m);//销毁矩阵
void DestroyMatrix(double** m, int col, int row);

void cross(double m[3], double t[3],double res[3]);

void CopyMatrix(Matrix s, Matrix& t);//复制矩阵

double norm(double Q[], int n);//求向量范数