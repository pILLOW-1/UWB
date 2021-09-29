#include "config.h"
#include <stdio.h>
#include <stdlib.h>




void CreateMatrix(DD* m, int col, int row)
{
	*m = (double**)malloc(sizeof(double*) * row);
	if (!*m)
	{
		printf("CreateMatrix failed!");
		exit(1);
	}
	for (int i = 0; i < row; i++)
	{
		(*m)[i] = (double*)malloc(sizeof(double) * col);
		if (!(*m)[i])
		{
			printf("CreateMatrix failed!");
			exit(1);
		}
	}
}


void DestroyMatrix(DD* m, int col, int row)
{
	for (int i = 0; i < row; i++)
		free((*m)[i]);
	free(*m);
}




void cross(double m[3], double t[3], double res[3])
{
	res[0] = m[1] * t[2] - m[2] * t[1];
	res[1] = m[0] * t[2] - m[2] * t[0];
	res[2] = m[0] * t[1] - m[1] * t[0];
}


//This function has been changed since pure C code is required, so C++ has been removed.
void CopyMatrix(Matrix s, Matrix* t)//复制矩阵
{
	if (s.base)
	{
		t->row = s.row;
		t->col = s.col;
		t->base = NULL;
		CreateMatrix(&(t->base),t->col,t->row);
		for (int i = 0; i < s.row; i++)
			for (int j = 0; j < s.col; j++)
				t->base[i][j] = s.base[i][j];
	}
	else {
		t->row = s.row;
		t->col = s.col;
		t->base = NULL;
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

int getA(double arcs[D_M][D_M], int n)
{
	if (n == 1)
	{
		return arcs[0][0];
	}
	double ans = 0;
	double temp[D_M][D_M];
	int i, j, k;
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n - 1; j++)
		{
			for (k = 0; k < n - 1; k++)
			{
				temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];

			}
		}
		int t = getA(temp, n - 1);
		if (i % 2 == 0)
		{
			ans += arcs[0][i] * t;
		}
		else
		{
			ans -= arcs[0][i] * t;
		}
	}
	return ans;
}


void getAStart(double arcs[D_M][D_M], int n, double ans[D_M][D_M])
{
	if (n == 1)
	{
		ans[0][0] = 1;
		return;
	}
	int i, j, k, t;
	double temp[D_M][D_M];
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			for (k = 0; k < n - 1; k++)
			{
				for (t = 0; t < n - 1; t++)
				{
					temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
				}
			}


			ans[i][j] = getA(temp, n - 1);
			if ((i + j) % 2 == 1)
			{
				ans[i][j] = -ans[i][j];
			}
		}
	}

}



void InverseMatrix(double a[D_M][D_M], double b[D_M][D_M])
{
	int k = getA(a, D_M);
	if (k == 0)
	{
		printf("can not transform!\n");
	}
	else
	{
		getAStart(a, D_M, b);
		for (int i = 0; i < D_M; i++)
		{
			for (int j = 0; j < D_M; j++)
			{
				b[i][j] = b[i][j] / k;
			}
		}
	}
}
