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

void DestroyMatrix(Matrix& m)//���پ���
{
	for (int i = 0; i < m.row; i++)
		free(m.base[i]);
	free(m.base);
}

Matrix Add(Matrix m, Matrix t)//�������
{
	if (m.col == t.col && m.row == t.row && m.base && t.base)
	{
		Matrix tmp;
		tmp.base = NULL;
		tmp.col = m.col;
		tmp.row = m.row;
		CreateMatrix(tmp);
		for (int i = 0; i < m.row; i++)
			for (int j = 0; j < m.col; j++)
				tmp.base[i][j] = t.base[i][j] + m.base[i][j];
		return tmp;
	}
}



Matrix Del(Matrix m, Matrix t)//�������
{
	if (m.col == t.col && m.row == t.row && m.base && t.base)
	{
		Matrix tmp;
		tmp.base = NULL;
		tmp.col = m.col;
		tmp.row = m.row;
		CreateMatrix(tmp);
		for (int i = 0; i < m.row; i++)
			for (int j = 0; j < m.col; j++)
				tmp.base[i][j] = m.base[i][j] - t.base[i][j];
		return tmp;
	}
}



Matrix Mul(Matrix m, Matrix t)//�������
{
	if (m.base && t.base && m.col == t.row)
	{
		Matrix tmp;
		zeros(tmp, m.row, t.col);
		for (int i = 0; i < m.row; i++)
			for (int j = 0; j < t.col; j++)
			{
				tmp.base[i][j] = 0.0;
				for (int k = 0; k < m.col; k++)
					tmp.base[i][j] += m.base[i][k] * t.base[k][j];
			}
		return tmp;
	}
}


void Transposition(Matrix s, Matrix& t)//����ת��
{
	if (s.base)
	{
		t.col = s.row;
		t.row = s.col;
		CreateMatrix(t);
		for (int i = 0; i < s.row; i++)
			for (int j = 0; j < s.col; j++)
				t.base[j][i] = s.base[i][j];
	}
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

void zeros(Matrix& m, int row, int col)//����ȫ�����
{
	m.base = NULL;
	m.row = row;
	m.col = col;
	CreateMatrix(m);
	for (int i = 0; i < m.row; i++)
		for (int j = 0; j < m.col; j++)
			m.base[i][j] = 0;
}

void diag(Matrix& m, double diag[], int n)//��diag����Ϊ�Խ�Ԫ�ش�������
{
	m.base = NULL;
	m.col = m.row = n;
	CreateMatrix(m);
	for (int i = 0; i < n; i++)
		m.base[i][i] = diag[i];
}

void eyes(Matrix& m, int n)//������СΪn�ĵ�λ��
{
	m.base = NULL;
	m.col = m.row = n;
	CreateMatrix(m);
	for (int i = 0; i < n; i++)
		m.base[i][i] = 1.0;
}

//��browΪ��ʼ�У�erowΪ�����У�bcolΪ��ʼ�У�ecolΪ�����н�t�����ݸ�ֵ��m�Ķ�Ӧλ��
void PartAssign(Matrix& m, int brow, int erow, int bcol, int ecol, Matrix t)
{
	if (!m.base)
		return;
	if (brow >= 0 && erow < m.row && bcol >= 0 && ecol < m.col)
	{
		for (int i = brow; i <= erow; i++)
			for (int j = bcol; j <= ecol; j++)
				m.base[i][j] = t.base[i][j];
	}
}

//��browΪ��ʼ�У�erowΪ�����У�bcolΪ��ʼ�У�ecolΪ�����н�m�����ݸ�Ϊt
void _PartAssign(Matrix m, int brow, int erow, int bcol, int ecol, Matrix& t)
{
	if (!m.base)
		return;
	if (brow >= 0 && erow < m.row && bcol >= 0 && ecol < m.col)
	{
		t.base = NULL;
		t.row = erow - brow + 1;
		t.col = ecol - bcol + 1;
		CreateMatrix(t);
		for (int i = brow; i <= erow; i++)
			for (int j = bcol; j <= ecol; j++)
				t.base[i][j] = m.base[i][j];
	}
}

