#include "SY_Labeling.h"
#include <stack>

SY_Labeling::SY_Labeling(int width, int height)
{
	m_nWidth = width; m_nHeight = height;
	m_nNumOfLabels = 0;
}


SY_Labeling::~SY_Labeling()
{

}

int SY_Labeling::labeling(unsigned char* srcImg, int* dstImg)
{
	std::stack<SY_Point> st_pt;

	for (int y = 1; y < m_nHeight - 1; y++)
	{
		for (int x = 1; x < m_nWidth - 1; x++)
		{
			// source image�� 255�� ��� + Labeling ������� ���� �ȼ������� labeling process ����
			if (srcImg[m_nWidth * y + x] == 0 || dstImg[m_nWidth * y + x] != 0) continue;

			m_nNumOfLabels++;

			// ���ο� label seed�� stack�� push
			st_pt.push(SY_Point(x,y));

			// �ش� label seed�� labeling�� ��(stack�� �� ��) ���� ����
			while (!st_pt.empty())
			{
				// stack top�� label point�� �ް� pop
				int ky = st_pt.top().y;
				int kx = st_pt.top().x;
				st_pt.pop();

				// label seed�� label number�� result image�� ����
				dstImg[m_nWidth * ky + kx] = m_nNumOfLabels;

				// search 8-neighbor
				for (int ny = ky - 1; ny <= ky + 1; ny++){
					// y�� ������ ����� �� ����
					if (ny < 0 || ny >= m_nHeight) continue;
					for (int nx = kx - 1; nx <= kx + 1; nx++){
						// x�� ������ ����� �� ����
						if (nx < 0 || nx >= m_nWidth) continue;

						// source image�� ���� �ְ� labeling�� �ȵ� ��ǥ�� stack�� push
						if (srcImg[m_nWidth * ny + nx] == 0 || dstImg[m_nWidth * ny + nx] != 0) continue;
						st_pt.push(SY_Point(nx, ny));

						// Ž���� �ȼ��̴� labeling
						dstImg[m_nWidth * ny + nx] = m_nNumOfLabels;
					}
				}
			}
		}
	}

	return m_nNumOfLabels;
}

int SY_Labeling::labeling(unsigned short* srcImg, unsigned short* dstImg)
{
	m_vecPt.clear();
	m_vecPixelCnt.clear();
	std::stack<SY_Point> st_pt;

	int pxCnt = 1;
	for (int y = 1; y < m_nHeight - 1; y++)
	{
		for (int x = 1; x < m_nWidth - 1; x++)
		{
			unsigned short estIlluRatio = srcImg[m_nWidth * y + x];

			// source image�� 0�� ���� ��� + Labeling ������� ���� �ȼ������� labeling process ����
			if (estIlluRatio == 0 || dstImg[m_nWidth * y + x] != 0) continue;

			m_nNumOfLabels++;
			pxCnt = 1;

			// ���ο� label seed�� stack�� push
			st_pt.push(SY_Point(x, y));

			double weightedSumX = 0, weightedSumY = 0;
			double ratioSum = 0;
			weightedSumX += x*estIlluRatio; weightedSumY += y*estIlluRatio;
			ratioSum += estIlluRatio;

			// �ش� label seed�� labeling�� ��(stack�� �� ��) ���� ����
			while (!st_pt.empty())
			{
				// stack top�� label point�� �ް� pop
				int ky = st_pt.top().y;
				int kx = st_pt.top().x;
				st_pt.pop();

				// label seed�� label number�� result image�� ����
				dstImg[m_nWidth * ky + kx] = m_nNumOfLabels;

				// search 8-neighbor
				for (int ny = ky - 1; ny <= ky + 1; ny++){
					// y�� ������ ����� �� ����
					if (ny < 0 || ny >= m_nHeight) continue;
					for (int nx = kx - 1; nx <= kx + 1; nx++){
						// x�� ������ ����� �� ����
						if (nx < 0 || nx >= m_nWidth) continue;

						estIlluRatio = srcImg[m_nWidth * ny + nx];
						// source image�� ���� �ְ� labeling�� �ȵ� ��ǥ�� stack�� push
						if (estIlluRatio == 0 || dstImg[m_nWidth * ny + nx] != 0) continue;
						st_pt.push(SY_Point(nx, ny));

						// Ž���� �ȼ��̴� labeling
						dstImg[m_nWidth * ny + nx] = m_nNumOfLabels;
						pxCnt++;

						weightedSumX += nx*estIlluRatio; weightedSumY += ny*estIlluRatio;
						ratioSum += estIlluRatio;
					}
				}
			} //while

			double cx = weightedSumX / ratioSum;
			double cy = weightedSumY / ratioSum;	
			m_vecPt.push_back(SY_Point(cx,cy));

			m_vecPixelCnt.push_back(pxCnt);
		}
	}

	return m_nNumOfLabels;
}

//���� ���� ����... �� �ʿ伺�� ũ�� ����...
void SY_Labeling::centeroidWithWeighted(int *labeledImg, int cx, int cy)
{
	for (int y = 0; y < m_nHeight; y++)
	{
		for (int x = 0; x < m_nWidth; x++)
		{
			int label = labeledImg[y*m_nWidth + x];
			if (label != 0)
			{
				;
			}
		}
	}
}