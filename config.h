#pragma once
#include <math.h>
#define IMU_LENGTH 82040//imu�ļ�����
#define UWB_LENGTH 1466//uwb�ļ�����
#define g0 9.7803267714      //�������ٶ�
#define pi 3.1415926535897931
#define r0 6378137           //���򳤰���
#define WIE 7.2921151647e-5  //������ת���ٶ�
#define GM 3.986005e+14      //����������������������˻�
#define eeee 0.00669438000426//ƫ����
#define ff (1 / 298.257)      //����,ϵͳ������f����
#define T 0.005              //IMUƵ��
#define T_UWB 0.25           //UWBƵ��
#define Anchor_num 4
#define D_X 15//״̬������С
#define D_M 4//�۲�������С

extern double  d2r ;
extern double dh2rs;
extern double acc_sf ;//���ٶȱ�������
extern double gyr_sf ;//���ٶȱ�������
extern double llh0[3] ;//γ�Ⱦ��ȸ߳�
extern double Lati0 ;
extern double Longi0;
extern double Alti0 ;


typedef struct matrix {
	double** base;//�׵�ַ
	int row;      //����
	int col;      //����
}Matrix;

void CreateMatrix(Matrix& m);//��������
void CreateMatrix(double** m, int col, int row);
void DestroyMatrix(Matrix& m);//���پ���
void DestroyMatrix(double** m, int col, int row);

void cross(double m[3], double t[3],double res[3]);

void CopyMatrix(Matrix s, Matrix& t);//���ƾ���

double norm(double Q[], int n);//����������