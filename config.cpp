#include "config.h"
#include <stdio.h>
#include <stdlib.h>

void zeros(Matrix& m, int row, int col);//����ȫ�����

void CreateMatrix(Matrix& m)//��������
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

void DestroyMatrix(Matrix& m)//���پ���
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




void cross(double m[3], double t[3],double res[3])
{
	res[0] = m[1] * t[2] - m[2] * t[1];
	res[1] = m[0] * t[2] - m[2] * t[0];
	res[2] = m[0] * t[1] - m[1] * t[0];
}



void CopyMatrix(Matrix s, Matrix& t)//���ƾ���
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


double norm(double Q[], int n)//����������
{
	double sum=0.0,norm;
	for (int i = 0; i < n; i++)
		sum += Q[i] * Q[i];
	norm = sqrt(sum);
	return norm;
}