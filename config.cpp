#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <algorithm>

using namespace std;


#define MAT_LEGAL_CHECKING
#define MAT_INIT_FLAG  0x5C

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
		CreateMatrix(&(t->base), t->col, t->row);
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


//矩阵求逆-原

/*
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

*/


//矩阵求逆-修改





Mat* MatCreate(Mat* mat, int row, int col)
{
	int i;

#ifdef MAT_LEGAL_CHECKING
	if (mat->init == MAT_INIT_FLAG) {
		if (mat->row == row && mat->col == col)
			return mat;
		else
			MatDelete(mat);
	}
#endif
	// 消除C6085和C6086警告
	if (row <= 0 || col <= 0) {
		printf("行数或列数不合法！\n");
		exit(-1);
	}

	// 动态申请内存 因此若是访问超出矩阵行和列的地方会警告
	mat->element = (double**)malloc(row * sizeof(double*));//行 
	for (i = 0; i < row; i++) {
		mat->element[i] = (double*)malloc(col * sizeof(double));//列
	}

	if (mat->element == NULL) {
		return NULL;
	}
	mat->row = row;
	mat->col = col;
	mat->init = MAT_INIT_FLAG;

	return mat;
}


void MatDelete(Mat* mat)
{
	int i;

#ifdef MAT_LEGAL_CHECKING
	if (mat->init != MAT_INIT_FLAG) {
		return;
	}
#endif

	for (i = 0; i < mat->row; i++)
		free(mat->element[i]);
	free(mat->element);

	mat->init = 0;
}


Mat* MatEye(Mat* mat, int n)
{
	MatCreate(mat, n, n);
	int i;

#ifdef MAT_LEGAL_CHECKING
	if (mat->init != MAT_INIT_FLAG) {
		printf("err check, none init matrix for MatEye\n");
		return NULL;
	}
#endif

	for (int row = 0; row < mat->row; row++) {
		for (int col = 0; col < mat->col; col++) {
			mat->element[row][col] = 0.0f;
		}
	}

	for (i = 0; i < min(mat->row, mat->col); i++) {
		mat->element[i][i] = 1.0f;
	}

	return mat;
}


/* 交换矩阵的两行 */
Mat* MatSwapRow(Mat* src, int r1, int r2)
{
	double* tmp;
	assert(r1 != r2);//assert断言 断言表示为一些布尔表达式，程序员相信在程序中的某个特定点该表达式值为真。
	tmp = src->element[r1];
	src->element[r1] = src->element[r2];
	src->element[r2] = tmp;
	return src;
}

/* 矩阵某行乘以一个系数 */
Mat* MatScaleRow(Mat* dst, int r, double scalar)
{
	int i;
	assert(scalar != 0.0);
	for (i = 0; i < dst->col; ++i)
	{
		dst->element[r][i] *= scalar;
	}
	return dst;
}


/* Add scalar * row r2 to row r1. */
Mat* MatShearRow(Mat* src, int r1, int r2, double scalar)
{
	int i;
	assert(r1 != r2);
	for (i = 0; i < src->col; ++i)
	{
		src->element[r1][i] += scalar * src->element[r2][i];
	}
	return src;
}


Mat* MatInv1(Mat* src, Mat* dst)
{
	MatCreate(dst, src->row, src->col);
	int i;
	int j;
	int r;
	double scalar;
	double shear_needed;
	assert(src->row == src->col);
	assert(src->row == dst->row);
	assert(src->row == dst->col);

	//MatSetIdent(&dst);
	MatEye(dst, dst->col);
	/* Convert input to the identity matrix via elementary row operations.
	   The ith pass through this loop turns the element at i,i to a 1
	   and turns all other elements in column i to a 0. */

	for (i = 0; i < dst->row; ++i) {

		if (dst->element[i][i] == 0.0) {
			/* We must swap rows to get a nonzero diagonal element. */

			for (r = i + 1; r < src->row; ++r) {
				if (src->element[r][i] != 0.0) {
					break;
				}
			}
			if (r == src->row) {
				/* Every remaining element in this column is zero, so this
				   matrix cannot be inverted. */
				return 0;
			}
			src = MatSwapRow(src, i, r);
			dst = MatSwapRow(dst, i, r);
		}

		/* Scale this row to ensure a 1 along the diagonal.
		   We might need to worry about overflow from a huge scalar here. */
		scalar = 1.0 / src->element[i][i];
		src = MatScaleRow(src, i, scalar);
		dst = MatScaleRow(dst, i, scalar);

		/* Zero out the other elements in this column. */
		for (j = 0; j < src->row; ++j) {
			if (i == j) {
				continue;
			}
			shear_needed = -src->element[j][i];
			src = MatShearRow(src, j, i, shear_needed);
			dst = MatShearRow(dst, j, i, shear_needed);
		}
	}
	return dst;
}

