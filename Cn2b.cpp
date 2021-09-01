#include <stdio.h>
#include <math.h>
#include "config.h"

void Cn2b(double yaw, double pitch, double roll, Matrix& cnb, Matrix& Q)
{
	Matrix cbn;
	cnb.base = NULL;
	cnb.col = 3;
	cnb.row = 3;
	Q.base = NULL;
	Q.col = 1;
	Q.row = 4;

	CreateMatrix(cnb);
	CreateMatrix(Q);
	cnb.base[0][0] = cos(yaw) * cos(pitch);
	cnb.base[0][1] = sin(yaw) * cos(pitch);
	cnb.base[0][2] = -sin(pitch);
	cnb.base[1][0] = -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll);
	cnb.base[1][1] = cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll);
	cnb.base[1][2] = cos(pitch) * sin(roll);
	cnb.base[2][0] = sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll);
	cnb.base[2][1] = -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll);
	cnb.base[2][2] = cos(pitch) * cos(roll);

	Transposition(cnb, cbn);
	Q.base[0][0]= 0.5 * sqrt(1 + cbn.base[0][0] + cbn.base[1][1] + cbn.base[3][3]);
	Q.base[1][0]= 0.25 * (cbn.base[2][1] - cbn.base[1][2]) / Q.base[0][0];
	Q.base[2][0]= 0.25 * (cbn.base[0][2] - cbn.base[2][0]) / Q.base[0][0];
	Q.base[3][0]= 0.25 * (cbn.base[1][0] - cbn.base[0][1]) / Q.base[0][0];

	double norm = 0.0, sum = 0.0;
	for (int i = 0; i < Q.row; i++)
		sum += Q.base[i][0] * Q.base[i][0];
	norm = sqrt(sum);
	for (int i = 0; i < Q.row; i++)
		Q.base[i][0] /= norm;
}

void Cn2b(double yaw, double pitch, double roll, double cnb[3][3], double Q[4])
{
	cnb[0][0] = cos(yaw) * cos(pitch);
	cnb[0][1] = sin(yaw) * cos(pitch);
	cnb[0][2] = -sin(pitch);
	cnb[1][0] = -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll);
	cnb[1][1] = cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll);
	cnb[1][2] = cos(pitch) * sin(roll);
	cnb[2][0] = sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll);
	cnb[2][1] = -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll);
	cnb[2][2] = cos(pitch) * cos(roll);
	double cbn[3][3];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cbn[i][j] = cnb[j][i];
	Q[0] = 0.5 * sqrt(1 + cbn[0][0] + cbn[1][1] + cbn[2][2]);
	Q[1] = 0.25 * (cbn[2][1] - cbn[1][2]) / Q[0];
	Q[2] = 0.25 * (cbn[0][2] - cbn[2][0]) / Q[0];
	Q[3] = 0.25 * (cbn[1][0] - cbn[0][1]) / Q[0];
	double norm = 0.0, sum = 0.0;
	for (int i = 0; i < 4; i++)
		sum += Q[i] * Q[i];
	norm = sqrt(sum);
	for (int i = 0; i < 4; i++)
		Q[i] /= norm;
}