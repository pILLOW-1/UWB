#include <stdio.h>
#include "config.h"
#include <math.h>

int find_matched_ins(Matrix INS_Time, double current_time)
{
	//double temp_time[IMU_LENGTH];
	Matrix temp_time;
	temp_time.base = NULL;
	temp_time.row = INS_Time.row;
	temp_time.col = INS_Time.col;
	CreateMatrix(temp_time);
	for (int i = 0; i < INS_Time.row; i++)
	{
		temp_time.base[i][0] = fabs(INS_Time.base[i][0] - current_time);
	}
	int index_LF = 0;
	for (int i = 0; i < INS_Time.row; i++)
		if (temp_time.base[i][0] < temp_time.base[index_LF][0])
			index_LF = i;
	if (index_LF % 2)
	{
		Matrix temp_imu_sec;
		CopyMatrix(INS_Time, temp_imu_sec);
		temp_imu_sec.base[index_LF][0] = INS_Time.base[0][0];
		for (int i = 0; i < INS_Time.row; i++)
		{
			temp_time.base[i][0] = fabs(temp_imu_sec.base[i][0] - current_time);
		}
		int index_LF = 0;
		for (int i = 0; i < INS_Time.row; i++)
			if (temp_time.base[i][0] < temp_time.base[index_LF][0])
				index_LF = i;
		DestroyMatrix(temp_imu_sec);
	}
	DestroyMatrix(temp_time);
	return index_LF;
}