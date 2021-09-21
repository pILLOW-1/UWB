#include "config.h"
#include <stdio.h>
#include <stdlib.h>


void zeros(Matrix& m, int row, int col);//创建全零矩阵

void CreateMatrix(Matrix& m)//创建矩阵
{

	m.base = (double**)malloc(sizeof(double*) * m.row);
	if (!m.base)
	{
		printf("CreateMatrix failed!");
		exit(1);
	}
	for (int i = 0; i < m.row; i++)
	{
		m.base[i] = (double*)malloc(sizeof(double) * m.col);
		if (!m.base[i])
		{
			printf("CreateMatrix failed!");
			exit(1);
		}
	}


}

void CreateMatrix(double** m, int col, int row)
{
	m = (double**)malloc(sizeof(double*) * row);
	if (!m)
	{
		printf("CreateMatrix failed!");
		exit(1);
	}
	for (int i = 0; i < row; i++)
	{
		m[i] = (double*)malloc(sizeof(double) * col);
		if (!m[i])
		{
			printf("CreateMatrix failed!");
			exit(1);
		}
	}
}

void DestroyMatrix(Matrix& m)//销毁矩阵
{
	for (int i = 0; i < m.row; i++)
		free(m.base[i]);
	free(m.base);
}

void DestroyMatrix(double** m, int col, int row)
{
	for (int i = 0; i < row; i++)
		free(m[i]);
	free(m);
}




void cross(double m[3], double t[3], double res[3])
{
	res[0] = m[1] * t[2] - m[2] * t[1];
	res[1] = m[0] * t[2] - m[2] * t[0];
	res[2] = m[0] * t[1] - m[1] * t[0];
}



void CopyMatrix(Matrix s, Matrix& t)//复制矩阵
{
	if (s.base)
	{
		t.row = s.row;
		t.col = s.col;
		t.base = NULL;
		CreateMatrix(t);
		for (int i = 0; i < s.row; i++)
			for (int j = 0; j < s.col; j++)
				t.base[i][j] = s.base[i][j];
	}
	else {
		t.row = s.row;
		t.col = s.col;
		t.base = NULL;
	}
}


double norm(double Q[], int n)//求向量范数
{
	double sum = 0.0, norm;
	for (int i = 0; i < n; i++)
		sum += Q[i] * Q[i];
	norm = sqrt(sum);
	return norm;
}


/*
void MultiplyMatrix(Matrix a, Matrix b, Matrix& c)
{
	
	for (int i = 0; i < a.row; i++)
	{
		for (int j = 0; j < b.col; j++)
		{
			double sum = 0.0;
			for (int k = 0; k < a.col; k++)
			{
				sum += a.base[i][k] * b.base[k][j];
			}
			c.base[i][j] = sum;
		}
	}
}
*/

void MultiplyMatrix(double a[D_X][D_X], double b[D_X][D_X], double c[D_X][D_X])
{
	for (int i = 0; i < D_X; i++)
	{
		for (int j = 0; j < D_X; j++)
		{
			double sum = 0.0;
			for (int k = 0; k < D_X; k++)
			{
				sum += a[i][k] * b[k][j];
			}
			c[i][j] = sum;
		}
	}
}


void TransposeMatrix(double a[D_X][D_X], double b[D_X][D_X])
{
	for (int i = 0; i < D_X; i++)
	{
		for (int j = 0; j < D_X; j++)
		{
			b[j][i] = a[i][j];
		}

	}
}
