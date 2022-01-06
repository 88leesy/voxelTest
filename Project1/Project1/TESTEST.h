#pragma once
#pragma once

//#include <iostream>
//#include <fstream>
//#include <vector>
//#include <string>
//#include <algorithm>
#include <math.h>
#include <time.h>
#include <iomanip>

#define _USE_MATH_DEFINES //<cmath>���� M_PI ����Ϸ���...
#include <cmath> 
using namespace std;
#include "DataInterface.h"

#include "voxel_DB.h"
#include "kmeans.h"

#include <opencv2\opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2\cudafeatures2d.hpp>

#define NOMINMAX  //windows.h ��� ���Ͽ��� min�� max�� ��ó���� ��ũ�η� ���Ǹ� �ؼ� �߻��ϴ� ������ �ذ��ϱ� ����.

//#if _DEBUG
//	#pragma comment(lib, "opencv_world343d.lib")
//#endif
//	#pragma comment(lib, "opencv_world343.lib")
#if _DEBUG
#pragma comment(lib, "opencv_world345d.lib")
#endif
#pragma comment(lib, "opencv_world345.lib")
using namespace cv;

vector<string> split(string str, char delimiter) {
	vector<string> internal;
	stringstream ss(str);
	string temp;

	while (getline(ss, temp, delimiter)) {
		internal.push_back(temp);
	}

	return internal;
}

//File loader supports .nvm format and bundler format
bool LoadModelFile(string& name, string& dataname, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc, int cubeSize);
void SaveNVM(const char* filename, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc);
void SaveBundlerModel(const char* filename, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx);

//////////////////////////////////////////////////////////////////
void AddNoise(vector<CameraT>& camera_data, vector<Point3D>& point_data, float percent);
void AddStableNoise(vector<CameraT>& camera_data, vector<Point3D>& point_data,
	const vector<int>& ptidx, const vector<int>& camidx, float percent);
bool RemoveInvisiblePoints(vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<int>& ptidx, vector<int>& camidx,
	vector<Point2D>& measurements, vector<string>& names, vector<int>& ptc);

void GetQuaternionRotationByPnP(cv::Mat R_matrix, cv::Mat t_matrix, float q2[4]);
bool EstimatePoseByPnP(vector<Point2d> features_2d, vector<Point3d> features_3d, double fx, double fy, double cx, double cy,
	cv::Mat &A_matrix, cv::Mat &R_matrix, cv::Mat &t_matrix);

bool EstimatePoseByPnP(vector<Point2d> list_points2d, vector<Point3d> list_points3d, double fx, double fy, double cx, double cy,
	cv::Mat &A_matrix, cv::Mat &R_matrix, cv::Mat &t_matrix)
{
	const double params[] = { fx,   // fx
							  fy,  // fy
							  cx,      // cx	
							  cy };    // cy

	A_matrix.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
	A_matrix.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
	A_matrix.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
	A_matrix.at<double>(1, 2) = params[3];
	A_matrix.at<double>(2, 2) = 1;

	//micro ��ķ... �ְ���
	//double k1 = 0.022774;
	//double k2 = -0.041311;
	//double p1 = -0.0055;
	//double p2 = -0.0009367;
	double k1 = 0;
	double k2 = -0;
	double p1 = -0;
	double p2 = -0;
	double d[] = { k1, k2, p1, p2 };
	cv::Mat distCoeffs(4, 1, CV_64FC1, d);

	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	bool useExtrinsicGuess = true;
	//bool correspondence = cv::solvePnP(list_points3d, list_points2d, A_matrix, distCoeffs, rvec, tvec,
	//	useExtrinsicGuess, SOLVEPNP_ITERATIVE);
	bool correspondence = cv::solvePnPRansac(list_points3d, list_points2d, A_matrix, distCoeffs, rvec, tvec, useExtrinsicGuess);

	if (correspondence)
	{
		//R|t TEST
		Rodrigues(rvec, R_matrix);
		t_matrix = tvec;

		return correspondence;
	}
	else
		return correspondence;
}


void GetQuaternionRotationByPnP(cv::Mat R_matrix, cv::Mat t_matrix, float q[4])
{
	float r00 = R_matrix.at<double>(0, 0);
	float r01 = R_matrix.at<double>(0, 1);
	float r02 = R_matrix.at<double>(0, 2);
	float r10 = R_matrix.at<double>(1, 0);
	float r11 = R_matrix.at<double>(1, 1);
	float r12 = R_matrix.at<double>(1, 2);
	float r20 = R_matrix.at<double>(2, 0);
	float r21 = R_matrix.at<double>(2, 1);
	float r22 = R_matrix.at<double>(2, 2);
	float t1 = t_matrix.at<double>(0);
	float t2 = t_matrix.at<double>(1);
	float t3 = t_matrix.at<double>(2);

	q[0] = 1 + r00 + r11 + r22;
	if (q[0] > 0.000000001)
	{
		q[0] = sqrt(q[0]) / 2.0;
		q[1] = (r21 - r12) / (4.0 *q[0]);
		q[2] = (r02 - r20) / (4.0 *q[0]);
		q[3] = (r10 - r01) / (4.0 *q[0]);
	}
	else
	{
		double s;
		if (r00 > r11 && r00 > r22)
		{
			s = 2.0 * sqrt(1.0 + r00 - r11 - r22);
			q[1] = 0.25 * s;
			q[2] = (r01 + r10) / s;
			q[3] = (r02 + r20) / s;
			q[0] = (r12 - r21) / s;
		}
		else if (r11 > r22)
		{
			s = 2.0 * sqrt(1.0 + r11 - r00 - r22);
			q[1] = (r01 + r10) / s;
			q[2] = 0.25 * s;
			q[3] = (r12 + r21) / s;
			q[0] = (r02 - r20) / s;
		}
		else
		{
			s = 2.0 * sqrt(1.0 + r22 - r00 - r11);
			q[1] = (r02 + r20) / s;
			q[2] = (r12 + r21) / s;
			q[3] = 0.25f * s;
			q[0] = (r01 - r10) / s;
		}
	}
}


//  1 : nvm 3d ��ǥ���� �翵�� 2d points�� label image ���� ������, 2d-3d corresponding ���� ���Ѱ�.
//  2 : fitting plane �ȿ� ���ϴ� 3d points���� min,max XYZ ���� �������� �翵�� 2d points�� label image ���� ������, 2d-3d corresponding ���� ���Ѱ�.
#define Voxel_Labeling_Method 1

/////////////////////////////////////////////////////////////////////////////
bool LoadNVM(ifstream& in, string& dataname, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc, int cubeSize)
{
	int rotation_parameter_num = 4;
	bool format_r9t = false;
	string token;
	if (in.peek() == 'N')
	{
		in >> token; //file header
		if (strstr(token.c_str(), "R9T"))
		{
			rotation_parameter_num = 9;    //rotation as 3x3 matrix
			format_r9t = true;
		}
	}


	int ncam = 0, npoint = 0, nproj = 0;
	// read # of cameras
	in >> ncam;  if (ncam <= 1) return false;

	//read the camera parameters
	camera_data.resize(ncam); // allocate the camera data
	names.resize(ncam);
	float radial_distortion = 0;
	for (int i = 0; i < ncam; ++i)
	{
		double f, q[9], c[3], d[2];
		in >> token >> f;
		for (int j = 0; j < rotation_parameter_num; ++j) in >> q[j];
		in >> c[0] >> c[1] >> c[2] >> d[0] >> d[1];

		//q�� ���ʹϾ�, c�� translation


		camera_data[i].SetFocalLength(f);
		if (format_r9t)
		{
			camera_data[i].SetMatrixRotation(q);
			camera_data[i].SetTranslation(c);
		}
		else
		{
			//older format for compability -> ���ʹϾ�� ī�޶� �߽��� R�� T�� ��ȯ ������
			camera_data[i].SetQuaternionRotation(q);        //quaternion from the file
			camera_data[i].SetCameraCenterAfterRotation(c); //camera center from the file
		}
		camera_data[i].SetNormalizedMeasurementDistortion(d[0]);
		names[i] = token;
		radial_distortion = d[0];
	}

	//////////////////////////////////////
	in >> npoint;   if (npoint <= 0) return false;

	//read image projections and 3D points.
	vector<DB_Point_3D> vec_db_pt_3d;
	vector<vector<Point2d>> features_2d;
	vector<vector<Point3d>> features_3d;
	vector<Point3d> test_features_3d;
	//featureImages.resize(ncam);
	features_2d.resize(ncam);
	features_3d.resize(ncam);
	point_data.resize(npoint);
	for (int i = 0; i < npoint; ++i)
	{
		DB_Point_3D db_pt_3d;

		float pt[3]; int cc[3], npj;
		in >> pt[0] >> pt[1] >> pt[2]
			>> cc[0] >> cc[1] >> cc[2] >> npj;

		db_pt_3d.setXYZ(pt[0], pt[1], pt[2]);
		db_pt_3d.pt_3d_id = i;

		for (int j = 0; j < npj; ++j)
		{
			int cidx, fidx; float imx, imy;
			in >> cidx >> fidx >> imx >> imy;

			//imx = imx * camera_data[cidx].f;
			//imy = imy * camera_data[cidx].f;

			camidx.push_back(cidx);    //camera index
			ptidx.push_back(fidx);        //point index

			//add a measurment to the vector
			measurements.push_back(Point2D(imx, imy));
			nproj++;

			float pt_2d[2] = { 0 };
			camera_data[cidx].GetProjectionPoint(pt, pt_2d);


			float f = camera_data[cidx].f;

			float x = camera_data[cidx].m[0][0] * pt[0]
				+ camera_data[cidx].m[0][1] * pt[1]
				+ camera_data[cidx].m[0][2] * pt[2];

			float y = camera_data[cidx].m[1][0] * pt[0]
				+ camera_data[cidx].m[1][1] * pt[1]
				+ camera_data[cidx].m[1][2] * pt[2];

			float z = camera_data[cidx].m[2][0] * pt[0]
				+ camera_data[cidx].m[2][1] * pt[1]
				+ camera_data[cidx].m[2][2] * pt[2];

			x = x + camera_data[cidx].t[0] ;
			y = y + camera_data[cidx].t[1];
			z = z + camera_data[cidx].t[2];

			float pnp_feature_x = f * x / z + 320;
			float pnp_feature_y = f * y / z + 240;

			//string text = names[i];
			//text.replace(text.find("jpg"), 3, "txt");


			features_2d[cidx].push_back(Point2d(imx, imy));
			features_3d[cidx].push_back(Point3d(pt[0], pt[1], pt[2]));

			db_pt_3d.vec_2d_pt.push_back(Point_2D(imx, imy));
			db_pt_3d.pt_2d_ids.push_back(fidx);
			db_pt_3d.img_IDs.push_back(cidx);
		}

		//printf("i:%d\n", i);

		point_data[i].SetPoint(pt);
		ptc.insert(ptc.end(), cc, cc + 3);

		vec_db_pt_3d.push_back(db_pt_3d);
	}

	float reSize_H = 768, reSize_W = 1024;   //640, 480


	for (int i=0; i < features_2d.size(); ++i)
	{
		///** The calibration matrix */
		cv::Mat A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
		cv::Mat R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
		cv::Mat t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix

		///** The computed projection matrix */
		cv::Mat _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);

		int c_w = 1024/2;   
		int c_h = 768/2;

		float fx = camera_data[i].f;
		float fy = camera_data[i].f;


		if (names[i].find("c0") != string::npos) {
			fx = 868.99;
			fy = 866.06;
			c_w = 525.94;
			c_h = 420.04;
		}
		else if (names[i].find("c1") != string::npos)
		{
			fx = 873.38;
			fy = 876.49;
			c_w = 529.32;
			c_h = 397.27;
		}


		vector<Point2d> vec_features_2d;
		vector<Point3d> vec_features_3d;

		vector<float> vec_error_t;
		vector<float> vec_error_r;
		for (int k = 0; k < features_2d[i].size(); ++k)
		{
			float feature_x = features_2d[i][k].x * fx  + c_w + 0.5;
			float feature_y = features_2d[i][k].y * fy + c_h + 0.5;

			feature_x = feature_x / ((float)1024 / reSize_W);
			feature_y = feature_y / ((float)768 / reSize_H);


			vec_features_2d.push_back(Point2d(feature_x, feature_y));
		}

		fx = fx / ((float)1024 / reSize_W);
		fy = fy / ((float)768 / reSize_H);
		c_w = c_w / ((float)1024 / reSize_W);
		c_h = c_h / ((float)768 / reSize_H);


		bool correspondence = EstimatePoseByPnP(vec_features_2d, features_3d[i], fx, fy, c_w, c_h, A_matrix, R_matrix, t_matrix);

		float t1 = t_matrix.at<double>(0);
		float t2 = t_matrix.at<double>(1);
		float t3 = t_matrix.at<double>(2);

		float q[4] = { 0 };
		GetQuaternionRotationByPnP(R_matrix, t_matrix, q);

		float q2[4];
		camera_data[i].GetQuaternionRotation(q2);
		
		float dot_prod = q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3];
		if (dot_prod > 1) dot_prod = 1;
		if (dot_prod < -1) dot_prod = -1;
		float theta2 = acos(dot_prod) * 180 / M_PI;

		float error_x = sqrt(pow(t1 - camera_data[i].t[0], 2) + pow(t2 - camera_data[i].t[1], 2) + pow(t3 - camera_data[i].t[2], 2));

		vec_error_t.push_back(error_x);
		vec_error_r.push_back(theta2);
		printf("id:%d, Error XYZ (m):%f, theta:%f\n", i, error_x, theta2);

		int debug = 0;
	}




	printf("finish!\n");


	/*
	DBSCAN Clustering
	*/
#define MINIMUM_POINTS 80    // minimum number of cluster
	//#define EPSILON (0.75*0.75)  // distance for clustering, metre^2
#define EPSILON (0.08)  // distance for clustering, metre^2

	vector<DB_Point> points;
	int nLabel = 0;

	for (int i = 0; i < npoint; ++i)
	{
		DB_Point tmp_pt;
		tmp_pt.clusterID = UNCLASSIFIED;
		tmp_pt.x = point_data[i].xyz[0];
		tmp_pt.y = point_data[i].xyz[1];
		tmp_pt.z = point_data[i].xyz[2];
		points.push_back(tmp_pt);
	}

	//#define NUM_COLOR 300
	srand(0);
	float color[2000][3] = { 0 };
	for (int i = 0; i < 2000; i++)
	{
		color[i][0] = (rand() % 255);
		color[i][1] = (rand() % 255);
		color[i][2] = (rand() % 255);
	}

	


	VOXEL_DB ds(MINIMUM_POINTS, EPSILON, vec_db_pt_3d, cubeSize);
	ds.voxelFitting3(ds.m_points, ds.getTotalPointSize(), color, nLabel);  //��������.

	int num_points = ds.m_points.size();
	printf("\n num_points:%d\n", num_points);

	vector<DB_Point_3D> tmp_points;
	for (int i = 0; i < num_points; ++i)
	{
		if (ds.m_points[i].clusterID != -1)
		{
			tmp_points.push_back(ds.m_points[i]);
		}
	}
	printf("tmp_points size:%d\n", tmp_points.size());
	printf("nLabel:%d\n", nLabel);

	/*
	label���� ����
	*/
	vector<Point3f> vec_centroid_points;
	vector<vector<DB_Point_3D>> vec_label_points;
	vec_label_points.resize(nLabel);

	printf("-------------------------------------------\n");
	for (int i = 0; i < num_points; ++i)
	{
		int label = tmp_points[i].clusterID - 1;
		vec_label_points[label].push_back(tmp_points[i]);
	}


	/*
	plane fitting���� inlier�� �̾Ƴ��� �����.
	*/
	vector<DB_Point_3D> vec_inlierData;
	float **coefs = new float *[nLabel];
	for (int n = 0; n < nLabel; ++n)
	{
		coefs[n] = new float[4];
	}

	float sumFit_distErr = 0;
	for (int i = 0; i < nLabel; ++i)
	{
		vector<DB_Point_3D> tmp_inlierData;
		float coef[4] = { 0 };
		//float meanErr = ds.PlaneFitting(vec_label_points[i], vec_label_points[i].size(), tmp_inlierData, color);
		float meanErr = ds.PlaneFitting_ver2(vec_label_points[i], vec_label_points[i].size(), tmp_inlierData, color, coef);
		coefs[i][0] = coef[0]; coefs[i][1] = coef[1]; coefs[i][2] = coef[2]; coefs[i][3] = coef[3];
		printf("%f, %f, %f, %f\n", coefs[i][0], coefs[i][1], coefs[i][2], coefs[i][3]);

		sumFit_distErr += meanErr;
		for (int n = 0; n < tmp_inlierData.size(); ++n)
		{
			vec_inlierData.push_back(tmp_inlierData[n]);
		}

		// ���� ����
		vec_label_points[i].clear();
		vec_label_points[i].resize(tmp_inlierData.size());
		copy(tmp_inlierData.begin(), tmp_inlierData.end(), vec_label_points[i].begin());

	}
	printf("avgFit_distErr:%f\n", sumFit_distErr / nLabel);
	printf("vec_inlierData:%d\n", vec_inlierData.size());
	ds.printResults(vec_inlierData, vec_inlierData.size(), true, color);




	printf("--------- centroid points extracting... -----------\n");
	int sumEachVoxelPtCnt = 0;
	for (int i = 0; i < nLabel; ++i)
	{
		float sumX = 0, sumY = 0, sumZ = 0;
		float avrX = 0, avrY = 0, avrZ = 0;
		for (int j = 0; j < vec_label_points[i].size(); ++j)
		{
			float x = vec_label_points[i][j].x;
			float y = vec_label_points[i][j].y;
			float z = vec_label_points[i][j].z;

			sumX += x; sumY += y; sumZ += z;
		}
		avrX = sumX / vec_label_points[i].size();
		avrY = sumY / vec_label_points[i].size();
		avrZ = sumZ / vec_label_points[i].size();

		Point3f centroidPt;
		int centId = -1;
		float minDist = 99999;
		for (int j = 0; j < vec_label_points[i].size(); ++j)
		{
			float x = vec_label_points[i][j].x;
			float y = vec_label_points[i][j].y;
			float z = vec_label_points[i][j].z;

			float dist = sqrt(pow(avrX - x, 2) + pow(avrY - y, 2) + pow(avrZ - z, 2));
			if (dist < minDist)
			{
				minDist = dist;
				centId = j;
			}
		}

		centroidPt.x = vec_label_points[i][centId].x;
		centroidPt.y = vec_label_points[i][centId].y;
		centroidPt.z = vec_label_points[i][centId].z;
		//centroidPt.x = avrX;
		//centroidPt.y = avrY;
		//centroidPt.z = avrZ;

		vec_centroid_points.push_back(centroidPt);

		sumEachVoxelPtCnt += vec_label_points[i].size();
		printf("centId:%d, minDist:%f, vec_label_points[i]:%d\n", centId, minDist, vec_label_points[i].size());
	}
	int avgEachVoxelPtCnt = sumEachVoxelPtCnt / nLabel;
	printf("sumEachVoxelPtCnt:%d, avgEachVoxelPtCnt:%d\n", sumEachVoxelPtCnt, avgEachVoxelPtCnt);
	printf("vec_centroid_points:%d\n", vec_centroid_points.size());

	/*
	Write 3d centroid points
	*/
	string save_3dpt_path = "F:/_voxelFeatureMap_CMU/_single_Voxel_set/" + dataname + "_3d_nearest_centroid.txt";
	ofstream output(save_3dpt_path);
	output << vec_centroid_points.size() << "\n";
	for (int i = 0; i < vec_centroid_points.size(); ++i)
	{
		float x = vec_centroid_points[i].x;
		float y = vec_centroid_points[i].y;
		float z = vec_centroid_points[i].z;

		output << x << " " << y << " " << z << "\n";
	}
	output.close();


	//printf("finish!\n");
	//exit(0);


	//std::ofstream oStream_dbData("F:/_voxelFeatureMap_CMU/_single_Voxel_set/" + dataname + "db_data.txt", ios::out | ios::binary); //ios::app => �̾��

	//�̹������� �ҷ�����
	vector<float> vec_error_t;
	vector<float> vec_error_r;
	for (int i = 0; i < ncam; ++i)
	{
		string img_path = "F:/CMU-Seasons/images/" + dataname + names[i];

		cout << names[i] << endl;

		Mat img = imread(img_path);
		Mat down_img = img.clone();
		Mat voxel_down_img = img.clone();
		resize(down_img, down_img, Size(reSize_W, reSize_H));  //455, 256
		resize(voxel_down_img, voxel_down_img, Size(reSize_W, reSize_H));  //455, 256

		Mat label_img = Mat::zeros(down_img.rows, down_img.cols, CV_16U);
		Mat kp_label_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);

		int size = features_3d[i].size();

		Mat voxel_fillContour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		Mat ref_voxel_fillContour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);

		Mat voxel_Contour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		Mat ref_voxel_Contour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);

		int c_w = img.cols / 2;
		int c_h = img.rows / 2;

		float fx = camera_data[i].f;
		float fy = camera_data[i].f;

		if (names[i].find("c0") != string::npos) {
			fx = 868.99;
			fy = 866.06;
			c_w = 525.94;
			c_h = 420.04;
		}
		else if (names[i].find("c1") != string::npos)
		{
			fx = 873.38;
			fy = 876.49;
			c_w = 529.32;
			c_h = 397.27;
		}




		cout << "img.cols:" << img.cols << ", img.rows:" << img.rows << endl;


		/*
		clustering 3D Points�� ���� �̹����� ������ 3D Point�� ���� ������,
		�� �̹����� ������ �� 3D Point�� clustering 3D Points�� ���� �ο�.
		�׸��� �󺧺��� 2d Point�� �ٽ� ����.
		-> �� �� Ŭ�����Ϳ� ����Ʈ ������ ������, Ư¡�� �׸�ŭ ���� ���̹Ƿ�, ���Ŀ� patch�μ� ���� ���� ��Ű�� ����.
		*/
		vector<vector<DB_Point>> eachLabelPt;
		int ss = vec_label_points.size();
		eachLabelPt.resize(ss);
		for (int k = 0; k < vec_label_points.size(); ++k)
		{
			for (int l = 0; l < vec_label_points[k].size(); ++l)
			{
				int imgs_size = vec_label_points[k][l].img_IDs.size();
				for (int m = 0; m < imgs_size; ++m)
				{
					if (vec_label_points[k][l].img_IDs[m] == i)
					{
						float feature_x = vec_label_points[k][l].vec_2d_pt[m].x * fx + c_w + 0.5;
						float feature_y = vec_label_points[k][l].vec_2d_pt[m].y * fy + c_h + 0.5;

						feature_x = feature_x / ((float)img.cols / reSize_W);
						feature_y = feature_y / ((float)img.rows / reSize_H);

						DB_Point tmp_db;
						int color_lb = vec_label_points[k][l].clusterID - 1;
						tmp_db.clusterID = color_lb + 1;  //����� 0 �󺧷� �ֱ�����... 
						tmp_db.x = vec_label_points[k][l].x;
						tmp_db.y = vec_label_points[k][l].y;
						tmp_db.z = vec_label_points[k][l].z;
						tmp_db.x_2d = feature_x;
						tmp_db.y_2d = feature_y;
						tmp_db.pt_3d_id = vec_label_points[k][l].pt_3d_id;
						tmp_db.pt_2d_id = vec_label_points[k][l].pt_2d_ids[m];

						eachLabelPt[color_lb].push_back(tmp_db);
					}

				}

			}

		}

		DB_Image2 DB_img;

		float q[4]; float t[3];
		camera_data[i].GetQuaternionRotation(q);
		camera_data[i].GetCameraCenter(t);
		//printf("q:%f %f %f %f, t:%f %f %f\n", q[0], q[1], q[2], q[3], t[0], t[1], t[2]);

		DB_img.quat[0] = q[0]; DB_img.quat[1] = q[1]; DB_img.quat[2] = q[2]; DB_img.quat[3] = q[3];
		DB_img.camCent[0] = t[0]; DB_img.camCent[1] = t[1]; DB_img.camCent[2] = t[2];
		DB_img.img_ID = i;

		string reName = names[i];
		reName.replace(reName.find("jpg"), 3, "png");
		DB_img.img_path = reName;
		//DB_img.img_path = names[i];

		double scaleX = (float)img.cols / reSize_W;
		double scaleY = (float)img.rows / reSize_H;


		DB_img.focal_lenX = fx / scaleX;
		DB_img.focal_lenY = fy / scaleY;
		DB_img.Cx = c_w / scaleX;
		DB_img.Cy = c_h / scaleY;

		int donw_c_w = down_img.cols / 2;
		int donw_c_h = down_img.rows / 2;

		cv::Mat A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
		cv::Mat _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);
		const double params[] = { DB_img.focal_lenX,   // fx
								  DB_img.focal_lenY,  // fy
								  DB_img.Cx,      // cx	
								  DB_img.Cy };    // cy

		A_matrix.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
		A_matrix.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
		A_matrix.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
		A_matrix.at<double>(1, 2) = params[3];
		A_matrix.at<double>(2, 2) = 1;
		_P_matrix.at<double>(0, 0) = camera_data[i].m[0][0];
		_P_matrix.at<double>(0, 1) = camera_data[i].m[0][1];
		_P_matrix.at<double>(0, 2) = camera_data[i].m[0][2];
		_P_matrix.at<double>(1, 0) = camera_data[i].m[1][0];
		_P_matrix.at<double>(1, 1) = camera_data[i].m[1][1];
		_P_matrix.at<double>(1, 2) = camera_data[i].m[1][2];
		_P_matrix.at<double>(2, 0) = camera_data[i].m[2][0];
		_P_matrix.at<double>(2, 1) = camera_data[i].m[2][1];
		_P_matrix.at<double>(2, 2) = camera_data[i].m[2][2];
		_P_matrix.at<double>(0, 3) = camera_data[i].t[0];
		_P_matrix.at<double>(1, 3) = camera_data[i].t[1];
		_P_matrix.at<double>(2, 3) = camera_data[i].t[2];

		cv::Mat AP_mat = cv::Mat(4, 3, CV_64FC1);
		AP_mat = A_matrix * _P_matrix;
		cv::Mat AP_mat_inv = AP_mat.inv(cv::DECOMP_SVD);

		vector<vector<DB_Point>> vec_tmpLabel_2d_3d_pts;
		vector<vector<DB_Point>> vec_label_2d_3d_pts;
		vec_tmpLabel_2d_3d_pts.resize(down_img.cols*down_img.rows);
		vec_label_2d_3d_pts.resize(down_img.cols*down_img.rows);

		printf("2222222222222222222222222222222222222222222222222222222222222222222\n");

		Mat voxel_label_img = label_img.clone();
		Mat ref_voxel_label_img = label_img.clone();
		//Mat ref_edge_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		if (Voxel_Labeling_Method == 1)
		{
			//j�� �� �� ��ȣ.
			for (int j = 0; j < eachLabelPt.size(); ++j)
			{
				if (eachLabelPt[j].size() < 4)
					continue;

				Mat tmpLabel_img = Mat::zeros(down_img.rows, down_img.cols, CV_16U);
				vector<Point> contour;
				Scalar cvColor(color[j][0], color[j][1], color[j][2]);
				for (int k = 0; k < eachLabelPt[j].size(); ++k)
				{
					int feature_x = eachLabelPt[j][k].x_2d + 0.5;
					int feature_y = eachLabelPt[j][k].y_2d + 0.5;

					contour.push_back(Point(feature_x, feature_y));
				}
				bool isClosed = true;
				vector<Point> hull(contour.size());
				convexHull(contour, hull);
				const Point *pts = (const Point*)Mat(hull).data;
				int npts = Mat(hull).rows;

				ushort val = eachLabelPt[j][0].clusterID;
				fillPoly(tmpLabel_img, &pts, &npts, 1, Scalar(val));
				ushort *tmpVoxlb_data_ptr = tmpLabel_img.ptr<ushort>(0);
				for (int yy = 0; yy < label_img.rows; ++yy)
				{
					for (int xx = 0; xx < label_img.cols; ++xx)
					{
						ushort val = tmpVoxlb_data_ptr[yy*label_img.cols + xx];

						if (val == 0) continue;

						int feature_x = xx;
						int feature_y = yy;

						DB_Point tmp;
						tmp.clusterID = val;   //����� 0�̶�... id�� 1���� ������.
						tmp.x_2d = feature_x;
						tmp.y_2d = feature_y;

						cv::Mat Point2f_vec = cv::Mat(3, 1, CV_64FC1);
						Point2f_vec.at<double>(0) = feature_x - donw_c_w; //focal_len = focal_len / scale;, scale = img.rows / reSize_H;												
						Point2f_vec.at<double>(1) = feature_y - donw_c_h;
						Point2f_vec.at<double>(2) = 1;
						cv::Mat point_3d = AP_mat_inv * Point2f_vec;
						point_3d.at<double>(0) = point_3d.at<double>(0) / point_3d.at<double>(3);
						point_3d.at<double>(1) = point_3d.at<double>(1) / point_3d.at<double>(3);
						point_3d.at<double>(2) = point_3d.at<double>(2) / point_3d.at<double>(3);
						point_3d.at<double>(3) = point_3d.at<double>(3) / point_3d.at<double>(3);

						//������ ����� ����(Pi) ���ϱ�
						float pt1[3] = { t[0], t[1], t[2] };  //������ǥ��󿡼� ī�޶� �߽� ��ǥ(ī�޶� ��ġ)
						float pt2[3] = { point_3d.at<double>(0), point_3d.at<double>(1), point_3d.at<double>(2) };  //�������� ���� Pl 
						float vv[3] = { pt2[0] - pt1[0], pt2[1] - pt1[1], pt2[2] - pt1[2] };  //������ �������� ���⺤��
						float mag = sqrt((vv[0] * vv[0]) + (vv[1] * vv[1]) + (vv[2] * vv[2]));
						float v[3] = { vv[0], vv[1], vv[2] };  //������ �������� ���� ���� ����
						float n[3] = { coefs[tmp.clusterID - 1][0], coefs[tmp.clusterID - 1][1], coefs[tmp.clusterID - 1][2] };
						float v_dot_n = v[0] * n[0] + v[1] * n[1] + v[2] * n[2];
						if (v_dot_n == 0)
						{
							printf("������ 0 �̹Ƿ�, ������ ����� ������ ����!\n");
						}
						float tt = (n[0] * pt1[0] + n[1] * pt1[1] + n[2] * pt1[2] + coefs[tmp.clusterID - 1][3])
							/ (n[0] * (pt1[0] - pt2[0]) + n[1] * (pt1[1] - pt2[1]) + n[2] * (pt1[2] - pt2[2]));

						float Pi[3] = { pt1[0] + tt * v[0], pt1[1] + tt * v[1], pt1[2] + tt * v[2] };

						tmp.x = Pi[0];
						tmp.y = Pi[1];
						tmp.z = Pi[2];

						vec_tmpLabel_2d_3d_pts[feature_y*down_img.cols + feature_x].push_back(tmp);

					}
				}

			}

			float cameraPt[3] = { t[0], t[1], t[2] };  //������ǥ��󿡼� ī�޶� �߽� ��ǥ(ī�޶� ��ġ)
			for (int j = 0; j < vec_tmpLabel_2d_3d_pts.size(); ++j)
			{
				if (vec_tmpLabel_2d_3d_pts[j].size() == 0) continue;

				if (vec_tmpLabel_2d_3d_pts[j].size() > 1)
				{
					int bestId = -1;
					float lowDist = 99999;
					for (int k = 0; k < vec_tmpLabel_2d_3d_pts[j].size(); ++k)
					{
						float x = vec_tmpLabel_2d_3d_pts[j][k].x;
						float y = vec_tmpLabel_2d_3d_pts[j][k].y;
						float z = vec_tmpLabel_2d_3d_pts[j][k].z;
						float dist = sqrt((cameraPt[0] - x)*(cameraPt[0] - x) + (cameraPt[1] - y)*(cameraPt[1] - y) + (cameraPt[2] - z)*(cameraPt[2] - z));
						if (lowDist > dist)
						{
							bestId = k;
							lowDist = dist;
						}
					}
					vec_label_2d_3d_pts[j].push_back(vec_tmpLabel_2d_3d_pts[j][bestId]);
				}
				else
					vec_label_2d_3d_pts[j] = vec_tmpLabel_2d_3d_pts[j];

			}

			//���� �󺧺��� contour ������...!!! 
			vector<vector<DB_Point>> vec_each_lb_pts;
			vec_each_lb_pts.resize(nLabel + 1);
			for (int j = 0; j < vec_label_2d_3d_pts.size(); ++j)
			{
				if (vec_label_2d_3d_pts[j].size() == 0) continue;

				ushort val = vec_label_2d_3d_pts[j][0].clusterID;
				vec_each_lb_pts[val].push_back(vec_label_2d_3d_pts[j][0]);
			}

			for (int lb = 0; lb < vec_each_lb_pts.size(); ++lb)
			{
				if (vec_each_lb_pts[lb].size() == 0) continue;

				ushort val = vec_each_lb_pts[lb][0].clusterID;
				Mat ref_kp_label_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
				for (int k = 0; k < vec_each_lb_pts[lb].size(); ++k)
				{
					int feature_x = vec_each_lb_pts[lb][k].x_2d;
					int feature_y = vec_each_lb_pts[lb][k].y_2d;

					ref_kp_label_img.data[feature_y*ref_kp_label_img.cols + feature_x] = 255;
					voxel_label_img.at<ushort>(Point(feature_x, feature_y)) = val;
					voxel_fillContour_img.data[feature_y*ref_kp_label_img.cols + feature_x] = 255;
				}

				//Sobel(ref_kp_label_img, ref_kp_label_img);
				Canny(ref_kp_label_img, ref_kp_label_img, 10, 20);
				bitwise_or(ref_kp_label_img, voxel_Contour_img, voxel_Contour_img);
				//imshow("voxel_Contour_img", voxel_Contour_img);
				//imshow("ref_kp_label_img", ref_kp_label_img);
				//waitKey(0);
			}


		} //Voxel_Labeling_Method == 1

		

		printf("333333333333333333333333333333333333333333333333333333333333333333333\n");



		for (int j = 0; j < eachLabelPt.size(); ++j)
		{
			if (eachLabelPt[j].size() < 3)
				continue;

			DB_img.voxel_db_pt.push_back(eachLabelPt[j]);

			vector<Point> contour, resized_contour;
			Scalar cvColor(color[j][0], color[j][1], color[j][2]);
			for (int k = 0; k < eachLabelPt[j].size(); ++k)
			{
				int feature_x = eachLabelPt[j][k].x_2d;
				int feature_y = eachLabelPt[j][k].y_2d;

				if (feature_x<0 || feature_x>down_img.cols)
					printf("feature_x:%d\n", feature_x);
				if (feature_y<0 || feature_y>down_img.rows)
					printf("feature_y:%d\n", feature_y);

				//circle(down_img, Point(feature_x, feature_y), 1, cvColor, -1);
				resized_contour.push_back(Point(feature_x, feature_y));

				kp_label_img.data[feature_y*kp_label_img.cols + feature_x] = 255;
			}
			bool isClosed = true;

			vector<Point> hull(resized_contour.size());
			convexHull(resized_contour, hull);
			const Point *pts = (const Point*)Mat(hull).data;
			int npts = Mat(hull).rows;

			//polylines(down_img, &pts, &npts, 1, isClosed, cvColor, 1);

			ushort val = eachLabelPt[j][0].clusterID;
			//printf("val:%d, j:%d\n", val, j);
			fillPoly(label_img, &pts, &npts, 1, Scalar(val));

		}
		imshow("kp_label_img", kp_label_img);
		imshow("down_img", down_img);
		//cv::waitKey(0);

		//cv::Mat tmp2 = cv::imread("__asdf__.png", cv::IMREAD_ANYDEPTH); //IMREAD_ANYDEPTH
		//for (int y = 0; y < tmp2.rows; ++y)
		//{
		//	for (int x = 0; x < tmp2.cols; ++x)
		//	{
		//		if(tmp2.at<ushort>(y, x) > 255)
		//			printf("%hd\n", tmp2.at<ushort>(y,x)); // .at<type>(y, x)
		//	}
		//}

		cv::Mat pic16bit;
		label_img.convertTo(pic16bit, CV_16U, 255); //convert to 16-bit by multiplying all values by 255
		cv::namedWindow("seg");
		cv::imshow("seg", pic16bit);
		cv::waitKey(0);


		vector<Point2d> vec_features_2d;
		vector<Point3d> vec_features_3d;



		//cv::Mat A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
		cv::Mat R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
		cv::Mat t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix


		for (int k = 0; k < DB_img.voxel_db_pt.size(); ++k)
		{
			for (int l = 0; l < DB_img.voxel_db_pt[k].size(); ++l)
			{
				float feature_x = DB_img.voxel_db_pt[k][l].x_2d;
				float feature_y = DB_img.voxel_db_pt[k][l].y_2d;

				float x = DB_img.voxel_db_pt[k][l].x;
				float y = DB_img.voxel_db_pt[k][l].y;
				float z = DB_img.voxel_db_pt[k][l].z;

				vec_features_2d.push_back(Point2d(feature_x, feature_y));
				vec_features_3d.push_back(Point3d(x, y,z));
			}

		}

		
		//fx = fx / ((float)img.cols / reSize_W);
		//fy = fy / ((float)img.rows / reSize_H);
		//c_w = c_w / ((float)img.cols / reSize_W);
		//c_h = c_h / ((float)img.rows / reSize_H);
		fx = DB_img.focal_lenX;
		fy = DB_img.focal_lenY;
		c_w = DB_img.Cx;
		c_h = DB_img.Cy;


		bool correspondence = EstimatePoseByPnP(vec_features_2d, vec_features_3d, fx, fy, c_w, c_h, A_matrix, R_matrix, t_matrix);

		float t1 = t_matrix.at<double>(0);
		float t2 = t_matrix.at<double>(1);
		float t3 = t_matrix.at<double>(2);

		GetQuaternionRotationByPnP(R_matrix, t_matrix, q);

		float q2[4];
		camera_data[i].GetQuaternionRotation(q2);

		float dot_prod = q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3];
		if (dot_prod > 1) dot_prod = 1;
		if (dot_prod < -1) dot_prod = -1;
		float theta2 = 2*acos(dot_prod) * 180 / M_PI;

		float error_x = sqrt(pow(t1 - camera_data[i].t[0], 2) + pow(t2 - camera_data[i].t[1], 2) + pow(t3 - camera_data[i].t[2], 2));

		vec_error_t.push_back(error_x);
		vec_error_r.push_back(theta2);
		printf("id:%d, Error XYZ (m):%f, theta:%f\n", i, error_x, theta2);

		int debug = 0;

	}


	double sum = std::accumulate(vec_error_t.begin(), vec_error_t.end(), 0.0);
	double mean = sum / vec_error_t.size();

	double sum_q = std::accumulate(vec_error_r.begin(), vec_error_r.end(), 0.0);
	double mean_q = sum_q / vec_error_r.size();

	printf("vec_error_t mean:%f, mean_q:%f \n", mean, mean_q);

	printf("DBSCAN finish!\n");

	for (int x = 0; x < nLabel; x++)
	{
		delete[] coefs[x];
	}
	delete[] coefs;

	return true;
}



bool LoadModelFile(string& name, string& dataname, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc, int cubeSize)
{
	if (name == "")return false;
	ifstream in(name);

	std::cout << "Loading cameras/points: " << name << "\n";
	if (!in.is_open()) return false;

	return LoadNVM(in, dataname, camera_data, point_data, measurements, ptidx, camidx, names, ptc, cubeSize);


}


