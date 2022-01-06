#include "PlaneRANSAC.h"
#include <time.h>

void PlaneRANSAC::calcPlaneCoefficient(DB_Point p1, DB_Point p2, DB_Point p3, Plane &result) {

	// 	result.equation.a = p1.y * (p2.z - p3.z) + p2.y * (p3.z - p1.z) + p3.y * (p1.z - p2.z);
	// 	result.equation.b = p1.z * (p2.x - p3.x) + p2.z * (p3.x - p1.x) + p3.z * (p1.x - p2.x);
	// 	result.equation.c = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
	// 	result.equation.d = (-1) * (	p1.x * (p2.y * p3.z - p3.y * p2.z) + 	p2.x * (p3.y * p1.z - p1.y * p3.z) + 	p3.x * (p1.y * p2.z - p2.y * p1.z));

	double x1 = p2.x - p1.x;
	double y1 = p2.y - p1.y;
	double z1 = p2.z - p1.z;

	double x2 = p3.x - p1.x;
	double y2 = p3.y - p1.y;
	double z2 = p3.z - p1.z;

	//surface noramal vector
	double x3 = y1 * z2 - z1 * y2;
	double y3 = z1 * x2 - x1 * z2;
	double z3 = x1 * y2 - y1 * x2;
	result.surfaceNomalX = x3;
	result.surfaceNomalY = y3;
	result.surfaceNomalZ = z3;

	double l = sqrt(x3*x3 + y3 * y3 + z3 * z3);

	result.equation.a = (x3) / (x3*(-p1.x) + y3 * (-p1.y) + z3 * (-p1.z));
	result.equation.b = (y3) / (x3*(-p1.x) + y3 * (-p1.y) + z3 * (-p1.z));
	result.equation.c = (z3) / (x3*(-p1.x) + y3 * (-p1.y) + z3 * (-p1.z));


	//nomalized vector
 	result.equation.a = x3/l;
 	result.equation.b = y3/l;
 	result.equation.c = z3/l;
	result.equation.d = -(result.equation.a*p1.x + result.equation.b*p1.y + result.equation.c*p1.z);
}

float PlaneRANSAC::calcDistance(Plane &phi, DB_Point p) {
	return abs(phi.equation.a * p.x + phi.equation.b * p.y + phi.equation.c * p.z + phi.equation.d) / sqrt(phi.equation.a * phi.equation.a + phi.equation.b * phi.equation.b + phi.equation.c * phi.equation.c);
}

//void PlaneRANSAC::run(vector<DB_Point>& vec_originPointCloudData, vector<DB_Point>& vec_superpixelXYZ, vector<Plane> &bestInlierPlane, int &planeLabel)
void PlaneRANSAC::run(vector<DB_Point>& vec_originPointCloudData, vector<Plane> &bestInlierPlane, int &planeLabel)
{
	/*
	int superCount = 0;
	int totalSize = 0, inputSize = 0, prevtotal_inputsize = 0;

	vector<int> vec_reductionSuperpixelXYZIdx;
	vector<int> vec_reductionPointCloudDataIdx;

	for (int i = 0; i < vec_originPointCloudData.size(); i++)
	{
		vec_originPointCloudData[i].clusterID = -1;
	}

	planeLabel = 1;

	for (int i = 0; i < vec_superpixelXYZ.size(); i++)
	{
		totalSize += vec_superpixelXYZ[i].size();
	}

	for (int i = 0; i < vec_originPointCloudData.size(); i++)
		vec_reductionPointCloudDataIdx.push_back(i);

	for (int superCount = 0; superCount < vec_superpixelXYZ.size(); superCount++)
	{
		if (superCount > 0)
			prevtotal_inputsize += vec_superpixelXYZ[superCount - 1].size();
		inputSize = vec_superpixelXYZ[superCount].size();

		int iterCntPlane = 0;
		int maxIterCntPlane = m_maxPlanes;
		int iterCnt = 0;

		int maxIterCnt = m_maxIteration;
		maxIterCnt = (int)(1 + log(1. - 0.99) / log(1. - pow(0.5, 3)));

		vector<int> vec_bestInlierOriginPointCloudDataIdx;
		vector<int> vec_bestReductiontPointCloudDataIdx;
		vector<int> vec_bestInlierIdx;
		Plane tempBestInlierPlane;
		DB_Point tempBestInlier3DPoints;

		float InlierAvrDist = 0;
		float lnlierDist = 0;
		float outlierDist = 0;
		int outlierCnt = 0;
		float InlierRatio = 0;

		iterCnt = 0;
		while (iterCnt < maxIterCnt)
		{
			// inlier�� ���� ���, Point Cloud Data
			Plane tempPlane;
			vector<int> vec_tempReductionPointCloudDataIdx;
			vector<int> vec_tempInlierIdx;

			// ������ �� 3�� �����ϱ�
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			XYZ srcPoint[3];
			int srcPointIdx[3];

			// �Է� Point Cloud Data���� �ߺ����� �ʴ� �� 3���� �����Ѵ�.
			srcPointIdx[0] = rand() % inputSize;
			srcPoint[0] = vec_superpixelXYZ[superCount][srcPointIdx[0]];

			while (true) {
				srcPointIdx[1] = rand() % inputSize;
				if (srcPointIdx[0] == srcPointIdx[1]) continue;
				else break;
			}
			while (true) {
				srcPointIdx[2] = rand() % inputSize;
				if (srcPointIdx[0] == srcPointIdx[2] || srcPointIdx[1] == srcPointIdx[2]) continue;
				else break;
			}
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



			// �� 3���� ���� ����� �������� �����Ѵ�.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			Plane phi;
			DB_Point p1 = vec_superpixelXYZ[superCount][srcPointIdx[0]].getXYZ();
			DB_Point p2 = vec_superpixelXYZ[superCount][srcPointIdx[1]].getXYZ();
			DB_Point p3 = vec_superpixelXYZ[superCount][srcPointIdx[2]].getXYZ();

			calcPlaneCoefficient(p1, p2, p3, phi);
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

			// ������ 3������ ���� ���� ������ Point Cloud Point�鰣�� �Ÿ��� ���Ͽ�
			// threshold���� ���� ��� inlier�� �Ǵ��Ѵ�.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

			//<<<<<<<<<<<<<<<<<<<<<<
			float threshold = m_distanceThreshold;

			//method3
			float tempinlierDist = 0;
			float tempoutlierDist = 0;
			int tempoutliercnt = 0;
			for (int i = 0; i < inputSize; i++)
			{
				if (threshold > calcDistance(phi, vec_superpixelXYZ[superCount][i].getXYZ()))
				{
					vec_tempReductionPointCloudDataIdx.push_back(vec_reductionPointCloudDataIdx[vec_superpixelXYZ[superCount][i].idx]);

					// 	//inlier ratio ���ϱ� ���� �͵�.
					// 	vec_tempInlierIdx.push_back(i);
					// 	tempinlierDist += calcDistance(phi,vec_superpixelXYZ[superCount][i].getXYZ());
				}
				else
				{
					// 	//outlier ration ���ϱ� ���� �͵�.(oulier �Ÿ�)
					// 	tempoutlierDist += calcDistance(phi,vec_superpixelXYZ[superCount][i].getXYZ());
					//  tempoutliercnt++;
				}
			}


			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


			// iteration ��ŭ �ݺ��Ǵ� ���μ��� �� ���� inlier�� ���� ����� ����� ã�´�.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

			if (vec_tempReductionPointCloudDataIdx.size() > m_minInlier)
			{
				if (vec_tempReductionPointCloudDataIdx.size() > vec_bestInlierOriginPointCloudDataIdx.size())
				{
					// ���� ����
					vec_bestInlierOriginPointCloudDataIdx.clear();
					vec_bestInlierOriginPointCloudDataIdx.resize(vec_tempReductionPointCloudDataIdx.size());
					copy(vec_tempReductionPointCloudDataIdx.begin(), vec_tempReductionPointCloudDataIdx.end(), vec_bestInlierOriginPointCloudDataIdx.begin());

					// ����Ʈ ��� �Ҵ�
					tempBestInlierPlane = phi;

					// 					//�ζ��̾�, �ƿ����̾� �Ÿ�
					// 					lnlierDist = tempinlierDist;
					// 					outlierDist = tempoutlierDist;
					// 					outlierCnt = tempoutliercnt;
					// 					tempBestInlier3DPoints = p1;
				}
			}
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			iterCnt++;
		}


		// 		//��. �� ��鿡 ���� lnlier�� ����
		// 		InlierRatio = (float)vec_bestInlierOriginPointCloudDataIdx.size()/(float)inputSize;
		// 
		// 		//��. �� ��鿡 ���� inlier�� ��հŸ�/outlier�� ��հŸ�
		// 		InlierAvrDist = lnlierDist/vec_bestInlierOriginPointCloudDataIdx.size();
		// 		outlierDist = outlierDist/outlierCnt;
		// 
		// 		//��. ��. ����
		// 		FILE *in, *in2, *in3;
		// 		in = fopen("InlierRatio.txt", "a");
		// 		fprintf(in, "%f\n", InlierRatio);
		// 		in2 = fopen("InlierAvrDist.txt", "a");
		// 		fprintf(in2, "%f\n", InlierAvrDist);
		// 		in3 = fopen("outlierDist.txt", "a");
		// 		fprintf(in3, "%f\n", outlierDist);
		// 		fclose(in);
		// 		fclose(in2);
		// 		fclose(in3);


				// ����Ʈ ��� �Ҵ�
		bestInlierPlane.push_back(tempBestInlierPlane);

		// ���� Point Cloud Data���� ����� �󺧸��Ѵ�.
		for (int i = vec_bestInlierOriginPointCloudDataIdx.size() - 1; i >= 0; i--)
			vec_originPointCloudData[vec_bestInlierOriginPointCloudDataIdx[i]].clusterID = planeLabel;

		cout << "best inlier cnt: " << vec_bestInlierOriginPointCloudDataIdx.size() << endl;
		cout << "bestInlierPlane: " << planeLabel << ":" << bestInlierPlane.size() << endl;

		planeLabel++;

	}
	*/


	

	// �迭�ε� pointCloudData�� ���� �������� ����
	vector<DB_Point> vec_reductionPointCloudData;
	for(int i=0; i < vec_originPointCloudData.size(); i++)
		vec_reductionPointCloudData.push_back(vec_originPointCloudData[i]);

	int totalSize, inputSize;
	totalSize = inputSize = vec_reductionPointCloudData.size();

	// Point Cloud Data�� �ε����� �����Ѵ�.
	vector<int> vec_reductionPointCloudDataIdx;
	for(int i=0; i<vec_reductionPointCloudData.size(); i++)
		vec_reductionPointCloudDataIdx.push_back(i);


	int iterCntPlane = 0;
	int maxIterCntPlane = m_maxPlanes;
	int iterCnt = 0;

	int maxIterCnt = m_maxIteration;
	//int maxIterCnt = (int)(1 + log(1. - 0.99)/log(1. - pow(0.5, 3)));

	planeLabel = 1;

	// ã�� ����� �ִ� ����� ������ �ʰ��ϴ� ��� �����Ѵ�.
	while(inputSize > totalSize * m_remainPointsRatio && iterCntPlane < maxIterCntPlane){
		vector<int> vec_bestInlierOriginPointCloudDataIdx;
		vector<int> vec_bestReductiontPointCloudDataIdx;
		Plane tempBestInlierPlane;

		iterCnt = 0;
		while(iterCnt < maxIterCnt)
		{
			// inlier�� ���� ���, Point Cloud Data
			Plane tempPlane;
			vector<int> vec_tempReductionPointCloudDataIdx;
			vector<int> vec_tempReductionPointCloudDataIdxOfIdx;

			// ������ �� 3�� �����ϱ�
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			DB_Point srcPoint[3];
			int srcPointIdx[3];

			////////////////
			// seed�� ���������־� �Է� Point Cloud Data�� ���� ���͵�
			// seed���� 0���� �����Ǿ� 0���� �ʱ�ȭ �� ���¿��� rand()�� �ʱ�ȭ �Ǳ� ������
			// �� �̹����� ���� seed������ ����� �������� �ٲ����� ����.
			srand(0);

			// �Է� Point Cloud Data���� �ߺ����� �ʴ� �� 3���� �����Ѵ�.
			srcPointIdx[0]= rand() % inputSize;
			srcPoint[0] = vec_reductionPointCloudData[srcPointIdx[0]];

			while(true){
				srcPointIdx[1]= rand() % inputSize;
				if(srcPointIdx[0] == srcPointIdx[1]) continue;
				else break;
			}
			while(true){
				srcPointIdx[2]= rand() % inputSize;
				if(srcPointIdx[0] == srcPointIdx[2] || srcPointIdx[1] == srcPointIdx[2]) continue;
				else break;
			}
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



			// �� 3���� ���� ����� �������� �����Ѵ�.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			Plane phi;
			DB_Point p1 = vec_reductionPointCloudData[srcPointIdx[0]].getXYZ();
			DB_Point p2 = vec_reductionPointCloudData[srcPointIdx[1]].getXYZ();
			DB_Point p3 = vec_reductionPointCloudData[srcPointIdx[2]].getXYZ();

			calcPlaneCoefficient(p1,p2,p3,phi);
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

			// ������ 3������ ���� ���� ������ Point Cloud Point�鰣�� �Ÿ��� ���Ͽ�
			// threshold���� ���� ��� inlier�� �Ǵ��Ѵ�.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

			//<<<<<<<<<<<<<<<<<<<<<<
			float threshold = m_distanceThreshold;
			for(int i=0; i<inputSize; i++){
				if( threshold > calcDistance(phi,vec_reductionPointCloudData[i].getXYZ()) ){
					vec_tempReductionPointCloudDataIdx.push_back( vec_reductionPointCloudDataIdx[i] );
					vec_tempReductionPointCloudDataIdxOfIdx.push_back(i);
				}
			}
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


			// iteration ��ŭ �ݺ��Ǵ� ���μ��� �� ���� inlier�� ���� ����� ����� ã�´�.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

			if(vec_tempReductionPointCloudDataIdx.size() > m_minInlier )
			{
				if( vec_tempReductionPointCloudDataIdx.size() > vec_bestInlierOriginPointCloudDataIdx.size() )
				{
					// ���� ����
					vec_bestInlierOriginPointCloudDataIdx.clear();
					vec_bestInlierOriginPointCloudDataIdx.resize(vec_tempReductionPointCloudDataIdx.size());
					copy(vec_tempReductionPointCloudDataIdx.begin(), vec_tempReductionPointCloudDataIdx.end(), vec_bestInlierOriginPointCloudDataIdx.begin());

					vec_bestReductiontPointCloudDataIdx.clear();
					vec_bestReductiontPointCloudDataIdx.resize(vec_tempReductionPointCloudDataIdxOfIdx.size());
					copy(vec_tempReductionPointCloudDataIdxOfIdx.begin(),vec_tempReductionPointCloudDataIdxOfIdx.end(),vec_bestReductiontPointCloudDataIdx.begin());

					// ����Ʈ ��� �Ҵ�
					tempBestInlierPlane = phi;
				}
			}
			// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			iterCnt++;
		}

		//if(vec_bestInlierOriginPointCloudDataIdx.size() > (inputSize*0.3))
		//{
		//	// ����Ʈ ��� �Ҵ�
		//	bestInlierPlane.push_back(tempBestInlierPlane);
		//}

		// ����Ʈ ��� �Ҵ�
		bestInlierPlane.push_back(tempBestInlierPlane);

		// ���� Point Cloud Data���� ����� �󺧸��Ѵ�.
		for(int i=vec_bestInlierOriginPointCloudDataIdx.size()-1; i>=0; i--)
			vec_originPointCloudData[ vec_bestInlierOriginPointCloudDataIdx[i] ].clusterID = planeLabel;

		// �Է� �����Ϳ��� �ζ��̾� Point Cloud Data�� �����Ѵ�.
		// �� �κ��� �ð��� �����ɸ�
		for(int i = vec_bestReductiontPointCloudDataIdx.size()-1; i>=0; i--){
			vec_reductionPointCloudData.erase( vec_reductionPointCloudData.begin() + vec_bestReductiontPointCloudDataIdx[i] );
			vec_reductionPointCloudDataIdx.erase( vec_reductionPointCloudDataIdx.begin() + vec_bestReductiontPointCloudDataIdx[i] );
		}

		cout << "best inlier cnt: "<<vec_bestInlierOriginPointCloudDataIdx.size() << endl;
		cout << "bestInlierPlane: "<<  planeLabel << ":" <<bestInlierPlane.size() << endl;
		cout << "iterCntPlane: "<<  iterCntPlane  << endl;

		inputSize = vec_reductionPointCloudData.size();
		planeLabel++;
		iterCntPlane++;
	}
	

}