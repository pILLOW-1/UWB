#ifndef _CONFIG_H
#define _CONFIG_H

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

typedef double** DD;

extern double  d2r;
extern double dh2rs;
extern double acc_sf;//加速度比例因子
extern double gyr_sf;//角速度比例因子
extern double llh0[3];//纬度经度高程
extern double Lati0;
extern double Longi0;
extern double Alti0;


typedef struct matrix {
	double** base;//首地址
	int row;      //行数
	int col;      //列数
}Matrix;

typedef struct {
	int row;
	int col;
	double** element;
	unsigned char init;
}Mat;  //专用于矩阵求逆


void CreateMatrix(DD* m, int col, int row);

void DestroyMatrix(DD* m, int col, int row);

void cross(double m[3], double t[3], double res[3]);

void CopyMatrix(Matrix s, Matrix* t);//复制矩阵

double norm(double Q[], int n);//求向量范数

//void MultiplyMatrix(Matrix a, Matrix b, Matrix& c); //D_X*D_X的矩阵a和矩阵b相乘，结果存与矩阵c中
void MultiplyMatrix(double a[D_X][D_X], double b[D_X][D_X], double c[D_X][D_X]);

void TransposeMatrix(double a[D_X][D_X], double b[D_X][D_X]); //D_X*D_X的矩阵a转置，结果存入矩阵b

//int getA(double arcs[D_M][D_M], int n); //按第一行展开求|A|

//void getAStart(double arcs[D_M][D_M], int n, double ans[D_M][D_M]); //计算每一行每一列的每个元素所对应的余子式，组成A*

//void InverseMatrix(double a[D_M][D_M], double b[D_M][D_M]);//D_M*D_M的矩阵a，求它的逆矩阵并存入矩阵b中

//以下为库中求逆的函数

Mat* MatCreate(Mat* mat, int row, int col);

void MatDelete(Mat* mat);

Mat* MatEye(Mat* mat, int n);

Mat* MatSwapRow(Mat* src, int r1, int r2);

Mat* MatScaleRow(Mat* dst, int r, double scalar);

Mat* MatShearRow(Mat* src, int r1, int r2, double scalar);

Mat* MatInv1(Mat* src, Mat* dst);

#endif
