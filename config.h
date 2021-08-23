#pragma once

typedef struct matrix {
	double** base;//首地址
	int row;      //行数
	int col;      //列数
}Matrix;

void CreateMatrix(Matrix& m);//创建矩阵
void DestroyMatrix(Matrix& m);//销毁矩阵
Matrix Add(Matrix m, Matrix t);//矩阵相加
Matrix Del(Matrix m, Matrix t);//矩阵相减
Matrix Mul(Matrix m, Matrix t);//矩阵相乘

void CopyMatrix(Matrix s, Matrix& t);//复制矩阵
void Transposition(Matrix s, Matrix& t);//矩阵转置
void zeros(Matrix& m,int row,int col);//创建全零矩阵
void diag(Matrix& m, double diag[],int n);//以diag数组为对角元素创建矩阵
void eyes(Matrix& m, int n);//创建大小为n的单位阵
//以brow为起始行，erow为结束行，bcol为起始列，ecol为结束列将t的内容赋值到m的对应位置
void PartAssign(Matrix& m, int brow, int erow, int bcol, int ecol, Matrix t);
//以brow为起始行，erow为结束行，bcol为起始列，ecol为结束列将m的内容赋为t
void _PartAssign(Matrix m, int brow, int erow, int bcol, int ecol, Matrix& t);
