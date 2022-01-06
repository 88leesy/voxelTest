#pragma once
#include<iostream>
#include<vector>

typedef struct SY_Point
{
	double x, y;
	SY_Point(double nX, double nY) : x(nX), y(nY) {}
} SY_Point;


class SY_Labeling
{
public:
	SY_Labeling(int width, int height);
	~SY_Labeling();

public:
	int labeling(unsigned char* srcImg, int* dstImg);
	int labeling(unsigned short* srcImg, unsigned short* dstImg);

	//���� ����� �� �ʿ䰡 ����. -> ��� ������ �� ������.
	void centeroidWithWeighted(int *labeledImg, int cx, int cy);

private:
	void connected_components();

public:
	int m_nNumOfLabels;
	std::vector<SY_Point> m_vecPt;
	std::vector<int> m_vecPixelCnt;

private:
	int m_nWidth, m_nHeight;

};

