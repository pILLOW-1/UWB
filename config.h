#pragma once

typedef struct matrix {
	double** base;//�׵�ַ
	int row;      //����
	int col;      //����
}Matrix;

void CreateMatrix(Matrix& m);//��������
void DestroyMatrix(Matrix& m);//���پ���
Matrix Add(Matrix m, Matrix t);//�������
Matrix Del(Matrix m, Matrix t);//�������
Matrix Mul(Matrix m, Matrix t);//�������

void CopyMatrix(Matrix s, Matrix& t);//���ƾ���
void Transposition(Matrix s, Matrix& t);//����ת��
void zeros(Matrix& m,int row,int col);//����ȫ�����
void diag(Matrix& m, double diag[],int n);//��diag����Ϊ�Խ�Ԫ�ش�������
void eyes(Matrix& m, int n);//������СΪn�ĵ�λ��
//��browΪ��ʼ�У�erowΪ�����У�bcolΪ��ʼ�У�ecolΪ�����н�t�����ݸ�ֵ��m�Ķ�Ӧλ��
void PartAssign(Matrix& m, int brow, int erow, int bcol, int ecol, Matrix t);
//��browΪ��ʼ�У�erowΪ�����У�bcolΪ��ʼ�У�ecolΪ�����н�m�����ݸ�Ϊt
void _PartAssign(Matrix m, int brow, int erow, int bcol, int ecol, Matrix& t);
