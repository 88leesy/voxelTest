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

#define _USE_MATH_DEFINES //<cmath>에서 M_PI 사용하려고...
#include <cmath> 
using namespace std;
#include "DataInterface.h"


#include "H_voxel_DB.h"
//#include "voxel_DB.h"
#//include "kmeans.h"
//#include "dbscan.h"

#include <opencv2\opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2\cudafeatures2d.hpp>

#define NOMINMAX  //windows.h 헤더 파일에서 min과 max를 전처리기 매크로로 정의를 해서 발생하는 문제를 해결하기 위함.

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
bool LoadModelFile(const char* name, vector<CameraT>& camera_data, vector<Point3D>& point_data,
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
	vector<Point2D>& measurements, vector<string>& names, vector<int>& ptc, vector<DB_Point_3D> & vec_db_pt_3d);

void GetQuaternionRotationByPnP(cv::Mat R_matrix, cv::Mat t_matrix, float q2[4]);
bool EstimatePoseByPnP(vector<Point2d> features_2d, vector<Point3d> features_3d, double focal_length,
	cv::Mat &A_matrix, cv::Mat &R_matrix, cv::Mat &t_matrix);

void plane_normVec(DB_Plane& plane);

bool EstimatePoseByPnP(vector<Point2d> list_points2d, vector<Point3d> list_points3d, double focal_length,
	cv::Mat &A_matrix, cv::Mat &R_matrix, cv::Mat &t_matrix)
{
	double f = focal_length;
	const double params[] = { f,   // fx
							  f,  // fy
							  0,      // cx	
							  0 };    // cy

	A_matrix.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
	A_matrix.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
	A_matrix.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
	A_matrix.at<double>(1, 2) = params[3];
	A_matrix.at<double>(2, 2) = 1;

	//micro 웹캠... 왜곡계수
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

//  1 : nvm 3d 좌표에서 사영된 2d points로 label image 만든 것으로, 2d-3d corresponding 관계 구한것.
//  2 : fitting plane 안에 속하는 3d points에서 min,max XYZ 구한 꼭지점을 사영한 2d points로 label image 만든 것으로, 2d-3d corresponding 관계 구한것.
#define Voxel_Labeling_Method 1


/////////////////////////////////////////////////////////////////////////////
bool LoadNVM(ifstream& in, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc, int cubeSize)
{

#define NUM_COLOR 1000000
	srand(0);
	int(*color)[3] = new int[NUM_COLOR][3];
	for (int i = 0; i < NUM_COLOR; i++)
	{
		color[i][0] = (rand() % 255);
		color[i][1] = (rand() % 255);
		color[i][2] = (rand() % 255);
	}

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

		//q는 쿼터니언, c는 translation


		camera_data[i].SetFocalLength(f);
		if (format_r9t)
		{
			camera_data[i].SetMatrixRotation(q);
			camera_data[i].SetTranslation(c);
		}
		else
		{
			//older format for compability -> 쿼터니언과 카메라 중심을 R과 T로 변환 시켜줌
			camera_data[i].SetQuaternionRotation(q);        //quaternion from the file
			camera_data[i].SetCameraCenterAfterRotation(c); //camera center from the file
		}
		camera_data[i].SetNormalizedMeasurementDistortion(d[0]);
		camera_data[i].radial = d[0];
		names[i] = token;
		radial_distortion = d[0];


		//string absPath2;
		//absPath2 = "F:/_voxelFeatureMap_Aachen2/_single_Voxel_set/";
		//vector<string> _line_str = split(names[0], '/');
		//string save_path_rgb2 = absPath2 + _line_str[1] + ".png";
		//string _save_path = absPath2 + _line_str[1] + "_label_" + ".png";
		//string _save_path2 = absPath2 + _line_str[1] + "_kp_" + ".png";
		//string _save_path3 = absPath2 + _line_str[1] + "_3dpts" + ".yml";
		//string _save_path4 = absPath2 + _line_str[1] + "_contour" + ".png";
		//string _save_path5 = absPath2 + _line_str[1] + "_fillcontour" + ".png";
		//cout << _save_path << endl;
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

			camidx.push_back(cidx);    //camera index
			ptidx.push_back(fidx);        //point index

			//add a measurment to the vector
			measurements.push_back(Point2D(imx, imy));
			nproj++;

			//float pt_2d[2] = { 0 };
			//camera_data[cidx].GetProjectionPoint(pt, pt_2d);

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

	//bool removed = RemoveInvisiblePoints(camera_data, point_data, ptidx, camidx, measurements, names, ptc, vec_db_pt_3d);

	printf("finish!\n");


	/*
	DBSCAN Clustering
	*/
#define MINIMUM_POINTS 80    // minimum number of cluster
	//#define EPSILON (0.75*0.75)  // distance for clustering, metre^2
#define EPSILON (0.08)  // distance for clustering, metre^2

	vector<DB_Point> points;
	int nLabel = 0;


	vector<DB_Voxels> vec_voxels;

	//DBSCAN ds(MINIMUM_POINTS, EPSILON, vec_db_pt_3d, cubeSize);
	//ds.run();
	//ds.printResults(ds.m_points, ds.getTotalPointSize(), true, color);

	H_VOXEL_DB ds(MINIMUM_POINTS, EPSILON, vec_db_pt_3d, cubeSize);

	//ds.voxelFitting_aachen(ds.m_points, ds.getTotalPointSize(), color, nLabel);  //만들어야함.
	ds.H_voxelFitting(ds.m_points, vec_voxels, ds.getTotalPointSize(), color, nLabel);  //만들어야함.
	//ds.voxelFitting_octree(ds.m_points, ds.getTotalPointSize(), color, nLabel);  //만들어야함.

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
	label별로 묶기
	*/
	vector<Point3f> vec_centroid_points;
	//vector<vector<DB_Point_3D>> vec_label_points;
	vector< H_Voxels> vec_label_points;
	vec_label_points.resize(nLabel);

	printf("-------------------------------------------\n");
	for (int i = 0; i < tmp_points.size(); ++i)
	{
		int label = tmp_points[i].clusterID - 1;
		vec_label_points[label].push_back(tmp_points[i]);
	}

	int nThresh_points = 4;

	/*
	plane fitting으로 inlier만 뽑아내서 만들기.
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
		if (vec_label_points[i].size() < nThresh_points)
		{
			vec_label_points[i].clear();
			continue;
		}

		vector<DB_Point_3D> tmp_inlierData;
		//float meanErr = ds.PlaneFitting(vec_label_points[i], vec_label_points[i].size(), tmp_inlierData, color);
		float coef[4] = { 0 };
		float meanErr = ds.PlaneFitting_ver2(vec_label_points[i], vec_label_points[i].size(), tmp_inlierData, color, coef);
		sumFit_distErr += meanErr;

		int lb = vec_label_points[i][0].clusterID;

		coefs[i][0] = coef[0]; coefs[i][1] = coef[1]; coefs[i][2] = coef[2]; coefs[i][3] = coef[3];
		printf("i:%d- %f, %f, %f, %f - lb:%d\n", i, coefs[i][0], coefs[i][1], coefs[i][2], coefs[i][3], lb);

		for (int n = 0; n < tmp_inlierData.size(); ++n)
		{
			vec_inlierData.push_back(tmp_inlierData[n]);
		}

		// 벡터 복사
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
		Point3f centroidPt;
		if (vec_label_points[i].size() < nThresh_points)
		{
			//vec_label_points[i].clear();

			centroidPt.x = -1000;
			centroidPt.y = -1000;
			centroidPt.z = -1000;
			vec_centroid_points.push_back(centroidPt);
			continue;
		}

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

		//		Point3f centroidPt;
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

		vec_centroid_points.push_back(centroidPt);

		sumEachVoxelPtCnt += vec_label_points[i].size();
		//printf("centId:%d, minDist:%f, vec_label_points[i]:%d\n", centId, minDist, vec_label_points[i].size());
	}
	int avgEachVoxelPtCnt = sumEachVoxelPtCnt / nLabel;
	printf("sumEachVoxelPtCnt:%d, avgEachVoxelPtCnt:%d\n", sumEachVoxelPtCnt, avgEachVoxelPtCnt);
	printf("vec_centroid_points:%d\n", vec_centroid_points.size());

	string absPath;
	if (Voxel_Labeling_Method == 1)
		absPath = "F:/_voxelFeatureMap_Aachen3/_single_Voxel_set/";
	//else if (Voxel_Labeling_Method == 2)
	//	absPath = "F:/_voxelFeatureMap_Aachen3/_single_Voxel_set/";
	//else
	//	absPath = "F:/_voxelFeatureMap_Aachen/_single_Voxel_set/";

	/*
	Write 3d centroid points
	*/
	//string save_path = "F:/_voxelFeatureMap_Aachen/_single_Voxel_set/_3d_nearest_centroid.txt";
	string save_path = absPath + "_3d_nearest_centroid.txt";
	ofstream output(save_path);
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


	std::ofstream oStream_dbData(absPath + "db_data.txt", ios::out | ios::binary); //ios::app => 이어쓰기


	//이미지별로 불러오기
	vector<float> vec_error_t;
	vector<float> vec_error_r;
	//id 894
	for (int i = 0; i < ncam; ++i)
	{
		string img_path = "F:/Aachen-Day-Night dataset/images/images_upright/" + names[i];

		cout << names[i] << endl;

		Mat img = imread(img_path);
		Mat down_img = img.clone();

		//float target_size = 256;
		//float scale = max(img.cols, img.rows);
		//scale = target_size / scale;
		//int reSize_W = img.cols*scale;
		//int reSize_H = img.rows*scale;

		int reSize_W = img.cols / 4;
		int reSize_H = img.rows / 4;

		//if (reSize_H < 176) reSize_H = 176;
		//if (reSize_W < 176) reSize_W = 176;

		resize(down_img, down_img, Size(reSize_W, reSize_H));
		Mat down_img2 = down_img.clone();

		Mat label_img = Mat::zeros(down_img.rows, down_img.cols, CV_16U);   //여기 어떻게 할지... 워낙 라벨 범위가 크다...
		Mat kp_label_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);

		Mat voxel_fillContour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		Mat ref_voxel_fillContour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);

		Mat voxel_Contour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		Mat ref_voxel_Contour_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);

		int c_w = img.cols / 2;   //여기 생각좀 하자.
		int c_h = img.rows / 2;

		float fx = camera_data[i].f;
		float fy = camera_data[i].f;

		cout << "img.cols:" << img.cols << ", img.rows:" << img.rows << ", f:" << fx << ", CX:" << c_w << ", CY:" << c_h << endl;
		int size = features_3d[i].size();

		/*
		clustering 3D Points와 기존 이미지별 들어오는 3D Point와 같은 곳에서,
		각 이미지별 들어오는 각 3D Point에 clustering 3D Points의 라벨을 부여.
		그리고 라벨별로 2d Point를 다시 묶음.
		-> 그 라벨 클러스터에 포인트 개수가 적으면, 특징이 그만큼 적은 것이므로, 이후에 patch로서 볼때 제외 시키기 위함.
		*/
		vector<vector<DB_Point>> eachLabelPt;
		int ss = vec_label_points.size();
		eachLabelPt.resize(ss);
		for (int k = 0; k < vec_label_points.size(); ++k)
		{
			if (vec_label_points[k].size() < nThresh_points)
			{
				continue;
			}
			for (int l = 0; l < vec_label_points[k].size(); ++l)
			{
				int imgs_size = vec_label_points[k][l].img_IDs.size();
				for (int m = 0; m < imgs_size; ++m)
				{
					if (vec_label_points[k][l].img_IDs[m] == i)
					{
						float feature_x = vec_label_points[k][l].vec_2d_pt[m].x;
						float feature_y = vec_label_points[k][l].vec_2d_pt[m].y;

						feature_x = feature_x / ((float)img.cols / reSize_W);
						feature_y = feature_y / ((float)img.rows / reSize_H);


						//printf("%f, %f\n ", feature_x, feature_y);

						DB_Point tmp_db;
						int color_lb = vec_label_points[k][l].clusterID - 1;
						tmp_db.clusterID = color_lb + 1;  //배경을 0 라벨로 주기위해... 
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

		DB_Image_Aachen DB_img;

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

		float focal_len = camera_data[i].GetFocalLength();
		double scale = (float)img.rows / reSize_H;
		focal_len = focal_len / scale;
		DB_img.focal_len = focal_len;
		DB_img.Cx = c_w;
		DB_img.Cy = c_h;
		DB_img.r = camera_data[i].radial;

		int donw_c_w = down_img.cols / 2;
		int donw_c_h = down_img.rows / 2;


		//Mat label_img = Mat::zeros(down_img.rows, down_img.cols, CV_16U);
		//Mat kp_label_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		//for (int j = 0; j < eachLabelPt.size(); ++j)
		//{
		//	if (eachLabelPt[j].size() < 4)
		//		continue;

		//	DB_img.voxel_db_pt.push_back(eachLabelPt[j]);

		//	vector<Point> contour, resized_contour;
		//	Scalar cvColor(color[j][0], color[j][1], color[j][2]);
		//	for (int k = 0; k < eachLabelPt[j].size(); ++k)
		//	{
		//		int feature_x = eachLabelPt[j][k].x_2d;
		//		int feature_y = eachLabelPt[j][k].y_2d;

		//		if (feature_x<0 || feature_x>down_img.cols)
		//			printf("feature_x:%d\n", feature_x);
		//		if (feature_y<0 || feature_y>down_img.rows)
		//			printf("feature_y:%d\n", feature_y);

		//		//circle(down_img, Point(feature_x, feature_y), 1, cvColor, -1);
		//		resized_contour.push_back(Point(feature_x, feature_y));

		//		//char txt[20];    // 크기가 20인 char형 배열을 선언
		//		//sprintf(txt, "%d", j);
		//		//putText(img, txt, Point(feature_x, feature_y), CV_FONT_HERSHEY_SIMPLEX, 0.4, Scalar(b, g, r));

		//		kp_label_img.data[feature_y*kp_label_img.cols + feature_x] = 255;
		//	}
		//	bool isClosed = true;

		//	vector<Point> hull(resized_contour.size());
		//	convexHull(resized_contour, hull);
		//	const Point *pts = (const Point*)Mat(hull).data;
		//	int npts = Mat(hull).rows;
		//	//polylines(down_img, &pts, &npts, 1, isClosed, cvColor, 1);

		//	ushort val = eachLabelPt[j][0].clusterID;
		//	//printf("val:%d, j:%d\n", val, j);
		//	fillPoly(label_img, &pts, &npts, 1, Scalar(val));

		//}
		//imshow("kp_label_img", kp_label_img);
		//imshow("down_img", down_img);

		//cv::Mat pic16bit;
		//label_img.convertTo(pic16bit, CV_16U, 255); //convert to 16-bit by multiplying all values by 255
		//cv::namedWindow("seg");
		//cv::imshow("seg", pic16bit);
		//cv::waitKey(0);


		cv::Mat A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
		cv::Mat _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);
		const double params[] = { focal_len,   // fx
								  focal_len,  // fy
								  0,      // cx	
								  0 };    // cy

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

		//printf("2222222222222222222222222222222222222222222222222222222222222222222\n");

		Mat voxel_label_img = label_img.clone();
		Mat ref_voxel_label_img = label_img.clone();
		//Mat ref_edge_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		if (Voxel_Labeling_Method == 1)
		{
			//j는 각 라벨 번호.
			for (int j = 0; j < eachLabelPt.size(); ++j)
			{
				//if (eachLabelPt[j].size() < 4)
				//	continue;
				if (eachLabelPt[j].size() < nThresh_points)  // eachLabelPt[j].size() < 3
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

				int val = eachLabelPt[j][0].clusterID;
				fillPoly(tmpLabel_img, &pts, &npts, 1, Scalar(val));
				ushort *tmpVoxlb_data_ptr = tmpLabel_img.ptr<ushort>(0);
				for (int yy = 0; yy < label_img.rows; ++yy)
				{
					for (int xx = 0; xx < label_img.cols; ++xx)
					{
						//float val = tmpVoxlb_data_ptr[yy*label_img.cols + xx];
						ushort val = tmpLabel_img.at<ushort>(Point(xx, yy));

						if (val == 0) continue;

						int feature_x = xx;
						int feature_y = yy;

						DB_Point tmp;
						tmp.clusterID = (int)val;   //배경이 0이라서... id는 1부터 시작함.
						tmp.x_2d = feature_x;
						tmp.y_2d = feature_y;

						cv::Mat Point2f_vec = cv::Mat(3, 1, CV_64FC1);
						Point2f_vec.at<double>(0) = feature_x; //focal_len = focal_len / scale;, scale = img.rows / reSize_H;												
						Point2f_vec.at<double>(1) = feature_y;
						Point2f_vec.at<double>(2) = 1;
						cv::Mat point_3d = AP_mat_inv * Point2f_vec;
						point_3d.at<double>(0) = point_3d.at<double>(0) / point_3d.at<double>(3);
						point_3d.at<double>(1) = point_3d.at<double>(1) / point_3d.at<double>(3);
						point_3d.at<double>(2) = point_3d.at<double>(2) / point_3d.at<double>(3);
						point_3d.at<double>(3) = point_3d.at<double>(3) / point_3d.at<double>(3);

						//직선과 평면의 교점(Pi) 구하기
						float pt1[3] = { t[0], t[1], t[2] };  //월드좌표계상에서 카메라 중심 좌표(카메라 위치)
						float pt2[3] = { point_3d.at<double>(0), point_3d.at<double>(1), point_3d.at<double>(2) };  //공간상의 한점 Pl 
						float vv[3] = { pt2[0] - pt1[0], pt2[1] - pt1[1], pt2[2] - pt1[2] };  //직선의 방정식의 방향벡터
						float mag = sqrt((vv[0] * vv[0]) + (vv[1] * vv[1]) + (vv[2] * vv[2]));
						float v[3] = { vv[0], vv[1], vv[2] };  //직선의 방정식의 방향 단위 벡터
						float n[3] = { coefs[tmp.clusterID - 1][0], coefs[tmp.clusterID - 1][1], coefs[tmp.clusterID - 1][2] };
						float v_dot_n = v[0] * n[0] + v[1] * n[1] + v[2] * n[2];
						if (v_dot_n == 0)
						{
							printf("내적이 0 이므로, 직선과 평면이 나란한 경우다!\n");

							printf("label:%d, v-:%f, %f, %f, ---- n: %f, %f, %f\n", (int)val, v[0], v[1], v[2], n[0], n[1], n[2]);
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

			float cameraPt[3] = { t[0], t[1], t[2] };  //월드좌표계상에서 카메라 중심 좌표(카메라 위치)
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

			//여기 라벨별로 contour 따야함...!!! 
			vector<vector<DB_Point>> vec_each_lb_pts;
			vec_each_lb_pts.resize(nLabel + 1);
			for (int j = 0; j < vec_label_2d_3d_pts.size(); ++j)
			{
				if (vec_label_2d_3d_pts[j].size() == 0) continue;

				int val = vec_label_2d_3d_pts[j][0].clusterID;
				vec_each_lb_pts[val].push_back(vec_label_2d_3d_pts[j][0]);
			}

			for (int lb = 0; lb < vec_each_lb_pts.size(); ++lb)
			{
				if (vec_each_lb_pts[lb].size() == 0) continue;

				int val = vec_each_lb_pts[lb][0].clusterID;
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

		else if (Voxel_Labeling_Method == 2)
		{
			//====================================================================================================================
			// ===================================================================================================================
			//테스트!!!!!!
			vec_tmpLabel_2d_3d_pts.clear();
			vec_label_2d_3d_pts.clear();
			vec_tmpLabel_2d_3d_pts.resize(down_img.cols*down_img.rows);
			vec_label_2d_3d_pts.resize(down_img.cols*down_img.rows);
			for (int j = 0; j < vec_voxels.size(); ++j)
			{

				if (eachLabelPt[j].size() < 4)
					continue;

				float inMaxX = -100000, inMaxY = -100000, inMaxZ = -100000;
				float inMinX = 1000000, inMinY = 1000000, inMinZ = 1000000;
				DB_Voxels tmp;

				for (int k = 0; k < eachLabelPt[j].size(); ++k)
				{
					float x = eachLabelPt[j][k].x;
					float y = eachLabelPt[j][k].y;
					float z = eachLabelPt[j][k].z;

					if (inMaxX < x) inMaxX = x;
					if (inMaxY < y) inMaxY = y;
					if (inMaxZ < z) inMaxZ = z;

					if (inMinX > x) inMinX = x;
					if (inMinY > y) inMinY = y;
					if (inMinZ > z) inMinZ = z;

				}

				tmp.planes[0].x[0] = inMaxX; tmp.planes[0].y[0] = inMaxY; tmp.planes[0].z[0] = inMaxZ;
				tmp.planes[0].x[1] = inMaxX; tmp.planes[0].y[1] = inMinY; tmp.planes[0].z[1] = inMaxZ;
				tmp.planes[0].x[2] = inMinX; tmp.planes[0].y[2] = inMinY; tmp.planes[0].z[2] = inMaxZ;
				tmp.planes[0].x[3] = inMinX; tmp.planes[0].y[3] = inMaxY; tmp.planes[0].z[3] = inMaxZ;
				plane_normVec(tmp.planes[0]);

				tmp.planes[1].x[0] = inMaxX; tmp.planes[1].y[0] = inMaxY; tmp.planes[1].z[0] = inMinZ;
				tmp.planes[1].x[1] = inMaxX; tmp.planes[1].y[1] = inMinY; tmp.planes[1].z[1] = inMinZ;
				tmp.planes[1].x[2] = inMaxX; tmp.planes[1].y[2] = inMinY; tmp.planes[1].z[2] = inMaxZ;
				tmp.planes[1].x[3] = inMaxX; tmp.planes[1].y[3] = inMaxY; tmp.planes[1].z[3] = inMaxZ;
				plane_normVec(tmp.planes[1]);

				tmp.planes[2].x[0] = inMinX; tmp.planes[2].y[0] = inMaxY; tmp.planes[2].z[0] = inMinZ;
				tmp.planes[2].x[1] = inMinX; tmp.planes[2].y[1] = inMinY; tmp.planes[2].z[1] = inMinZ;
				tmp.planes[2].x[2] = inMaxX; tmp.planes[2].y[2] = inMinY; tmp.planes[2].z[2] = inMinZ;
				tmp.planes[2].x[3] = inMaxX; tmp.planes[2].y[3] = inMaxY; tmp.planes[2].z[3] = inMinZ;
				plane_normVec(tmp.planes[2]);

				tmp.planes[3].x[0] = inMinX; tmp.planes[3].y[0] = inMaxY; tmp.planes[3].z[0] = inMaxZ;
				tmp.planes[3].x[1] = inMinX; tmp.planes[3].y[1] = inMinY; tmp.planes[3].z[1] = inMaxZ;
				tmp.planes[3].x[2] = inMinX; tmp.planes[3].y[2] = inMinY; tmp.planes[3].z[2] = inMinZ;
				tmp.planes[3].x[3] = inMinX; tmp.planes[3].y[3] = inMaxY; tmp.planes[3].z[3] = inMinZ;
				plane_normVec(tmp.planes[3]);

				tmp.planes[4].x[0] = inMaxX; tmp.planes[4].y[0] = inMaxY; tmp.planes[4].z[0] = inMinZ;
				tmp.planes[4].x[1] = inMaxX; tmp.planes[4].y[1] = inMaxY; tmp.planes[4].z[1] = inMaxZ;
				tmp.planes[4].x[2] = inMinX; tmp.planes[4].y[2] = inMaxY; tmp.planes[4].z[2] = inMaxZ;
				tmp.planes[4].x[3] = inMinX; tmp.planes[4].y[3] = inMaxY; tmp.planes[4].z[3] = inMinZ;
				plane_normVec(tmp.planes[4]);

				tmp.planes[5].x[0] = inMaxX; tmp.planes[5].y[0] = inMinY; tmp.planes[5].z[0] = inMaxZ;
				tmp.planes[5].x[1] = inMaxX; tmp.planes[5].y[1] = inMinY; tmp.planes[5].z[1] = inMinZ;
				tmp.planes[5].x[2] = inMinX; tmp.planes[5].y[2] = inMinY; tmp.planes[5].z[2] = inMinZ;
				tmp.planes[5].x[3] = inMinX; tmp.planes[5].y[3] = inMinY; tmp.planes[5].z[3] = inMaxZ;
				plane_normVec(tmp.planes[5]);

				vector<Point> resized_contour;
				Scalar cvColor(color[j][0], color[j][1], color[j][2]);
				for (int k = 0; k < 6; ++k)
				{
					for (int l = 0; l < 4; ++l)
					{

						//GT R|t를 이용한 projection 좌표
						float x = camera_data[i].m[0][0] * tmp.planes[k].x[l]
							+ camera_data[i].m[0][1] * tmp.planes[k].y[l]
							+ camera_data[i].m[0][2] * tmp.planes[k].z[l];

						float y = camera_data[i].m[1][0] * tmp.planes[k].x[l]
							+ camera_data[i].m[1][1] * tmp.planes[k].y[l]
							+ camera_data[i].m[1][2] * tmp.planes[k].z[l];

						float z = camera_data[i].m[2][0] * tmp.planes[k].x[l]
							+ camera_data[i].m[2][1] * tmp.planes[k].y[l]
							+ camera_data[i].m[2][2] * tmp.planes[k].z[l];

						x = x + camera_data[i].t[0];
						y = y + camera_data[i].t[1];
						z = z + camera_data[i].t[2];

						x = focal_len * x;
						y = focal_len * y;

						float pnp_feature_x = x / z;
						float pnp_feature_y = y / z;

						float feature_x = pnp_feature_x + 0.5;
						float feature_y = pnp_feature_y + 0.5;

						if (feature_x < 0) feature_x = 0;
						if (feature_y < 0) feature_y = 0;
						if (feature_x >= reSize_W) feature_x = reSize_W - 1;
						if (feature_y >= reSize_H) feature_y = reSize_H - 1;

						resized_contour.push_back(Point(feature_x, feature_y));

					}
				}


				bool isClosed = true;

				vector<Point> hull(resized_contour.size());
				convexHull(resized_contour, hull);
				const Point *pts = (const Point*)Mat(hull).data;
				int npts = Mat(hull).rows;

				Mat tmp_voxel_label_img = Mat::zeros(down_img.rows, down_img.cols, CV_16U);
				ushort val = vec_voxels[j].clusterID;
				fillPoly(tmp_voxel_label_img, &pts, &npts, 1, Scalar(val));

				ushort *tmpVoxlb_data_ptr = tmp_voxel_label_img.ptr<ushort>(0);
				for (int yy = 0; yy < label_img.rows; ++yy)
				{
					for (int xx = 0; xx < label_img.cols; ++xx)
					{
						ushort val = tmpVoxlb_data_ptr[yy*label_img.cols + xx];
						if (val == 0) continue;

						int feature_x = xx;
						int feature_y = yy;

						DB_Point tmp;
						tmp.clusterID = val;   //배경이 0이라서... id는 1부터 시작함.
						tmp.x_2d = feature_x;
						tmp.y_2d = feature_y;

						cv::Mat Point2f_vec = cv::Mat(3, 1, CV_64FC1);
						Point2f_vec.at<double>(0) = feature_x; //focal_len = focal_len / scale;, scale = img.rows / reSize_H;												
						Point2f_vec.at<double>(1) = feature_y;
						Point2f_vec.at<double>(2) = 1;
						cv::Mat point_3d = AP_mat_inv * Point2f_vec;
						point_3d.at<double>(0) = point_3d.at<double>(0) / point_3d.at<double>(3);
						point_3d.at<double>(1) = point_3d.at<double>(1) / point_3d.at<double>(3);
						point_3d.at<double>(2) = point_3d.at<double>(2) / point_3d.at<double>(3);
						point_3d.at<double>(3) = point_3d.at<double>(3) / point_3d.at<double>(3);

						//직선과 평면의 교점(Pi) 구하기
						float pt1[3] = { t[0], t[1], t[2] };  //월드좌표계상에서 카메라 중심 좌표(카메라 위치)
						float pt2[3] = { point_3d.at<double>(0), point_3d.at<double>(1), point_3d.at<double>(2) };  //공간상의 한점 Pl 
						float vv[3] = { pt2[0] - pt1[0], pt2[1] - pt1[1], pt2[2] - pt1[2] };  //직선의 방정식의 방향벡터
						float mag = sqrt((vv[0] * vv[0]) + (vv[1] * vv[1]) + (vv[2] * vv[2]));
						float v[3] = { vv[0], vv[1], vv[2] };  //직선의 방정식의 방향 단위 벡터
						float n[3] = { coefs[tmp.clusterID - 1][0], coefs[tmp.clusterID - 1][1], coefs[tmp.clusterID - 1][2] };
						float v_dot_n = v[0] * n[0] + v[1] * n[1] + v[2] * n[2];
						if (v_dot_n == 0)
						{
							printf("내적이 0 이므로, 직선과 평면이 나란한 경우다!\n");
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

			float cameraPt[3] = { t[0], t[1], t[2] };  //월드좌표계상에서 카메라 중심 좌표(카메라 위치)
			for (int j = 0; j < vec_tmpLabel_2d_3d_pts.size(); ++j)
			{
				if (vec_tmpLabel_2d_3d_pts[j].size() == 0) continue;

				if (vec_tmpLabel_2d_3d_pts[j].size() > 1)
				{
					int bestId = -1;
					float lowDist = 9999;
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

			//여기 코딩 다시하자... 라벨별로 contour 따야함...!!! 
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
					ref_voxel_label_img.at<ushort>(Point(feature_x, feature_y)) = val;
					ref_voxel_fillContour_img.data[feature_y*ref_kp_label_img.cols + feature_x] = 255;
				}


				Canny(ref_kp_label_img, ref_kp_label_img, 50, 100);
				bitwise_or(ref_kp_label_img, ref_voxel_Contour_img, ref_voxel_Contour_img);
			}
			//for (int lb = 0; lb < vec_each_lb_pts.size(); ++lb)
			//{
			//	if (vec_each_lb_pts[lb].size() == 0) continue;

			//	vector<Point> contour;ref_voxel_Contour_img
			//	for (int k = 0; k < vec_each_lb_pts[lb].size(); ++k)
			//	{
			//		int feature_x = vec_each_lb_pts[lb][k].x_2d;
			//		int feature_y = vec_each_lb_pts[lb][k].y_2d;

			//		contour.push_back(Point(feature_x, feature_y));
			//	}
			//	

			//	bool isClosed = true;
			//	vector<Point> hull(contour.size());
			//	convexHull(contour, hull);
			//	const Point *pts = (const Point*)Mat(hull).data;
			//	int npts = Mat(hull).rows;
			//	polylines(ref_voxel_Contour_img, &pts, &npts, 1, isClosed, Scalar(255, 255, 255) , 1);

			//	ushort val = vec_each_lb_pts[lb][0].clusterID;
			//	fillPoly(ref_voxel_label_img, &pts, &npts, 1, Scalar(val));
			//}
			//====================================================================================================================
			//====================================================================================================================
		}

		//printf("333333333333333333333333333333333333333333333333333333333333333333333\n");

		//원본 voxel  data... keypoints랑 그에 의한 convexhull labeling...
		for (int j = 0; j < eachLabelPt.size(); ++j)
		{
			if (eachLabelPt[j].size() < 4)
				continue;

			DB_img.voxel_db_pt.push_back(eachLabelPt[j]);

			vector<Point> contour, resized_contour;
			Scalar cvColor(color[j][0], color[j][1], color[j][2]);
			for (int k = 0; k < eachLabelPt[j].size(); ++k)
			{
				int feature_x = eachLabelPt[j][k].x_2d + 0.5;
				int feature_y = eachLabelPt[j][k].y_2d + 0.5;

				circle(down_img, Point(feature_x, feature_y), 1, cvColor, -1);
				resized_contour.push_back(Point(feature_x, feature_y));

				kp_label_img.data[feature_y*kp_label_img.cols + feature_x] = 255;

			}
			bool isClosed = true;

			vector<Point> hull(resized_contour.size());
			convexHull(resized_contour, hull);
			const Point *pts = (const Point*)Mat(hull).data;
			int npts = Mat(hull).rows;
			polylines(down_img, &pts, &npts, 1, isClosed, cvColor, 1);

			ushort val = eachLabelPt[j][0].clusterID;
			fillPoly(label_img, &pts, &npts, 1, Scalar(val));

		}

		int TotalNumberOfPixels = voxel_label_img.rows*voxel_label_img.cols;
		int zeroPixels = TotalNumberOfPixels - countNonZero(voxel_label_img);
		printf("None voxelLabel! : %d, %d\n", zeroPixels, TotalNumberOfPixels);
		if (zeroPixels == TotalNumberOfPixels)
		{
			continue;
		}


		imshow("kp_label_img", kp_label_img);
		imshow("down_img", down_img);
		cv::Mat pic16bit;
		label_img.convertTo(pic16bit, CV_16U, 255); //convert to 16-bit by multiplying all values by 255
		cv::namedWindow("label_img");
		cv::imshow("label_img", pic16bit);

		cv::Mat pic16bit2;
		voxel_label_img.convertTo(pic16bit2, CV_16U, 255); //convert to 16-bit by multiplying all values by 255
		cv::namedWindow("voxel_label_img");
		cv::imshow("voxel_label_img", pic16bit2);

		cv::Mat pic16bit3;
		ref_voxel_label_img.convertTo(pic16bit3, CV_16U, 255); //convert to 16-bit by multiplying all values by 255
		cv::namedWindow("ref_voxel_label_img");
		cv::imshow("ref_voxel_label_img", pic16bit3);


		imshow("voxel_Contour_img", voxel_Contour_img);
		imshow("ref_voxel_Contour_img", ref_voxel_Contour_img);
		cv::waitKey(0);


		//printf("44444444444444444444444444444444444444444444444444444444444444444444444444\n");


		ushort *lb_data_ptr;
		uchar *contour_data_ptr;

		if (Voxel_Labeling_Method == 1)
		{
			lb_data_ptr = voxel_label_img.ptr<ushort>(0);
			contour_data_ptr = voxel_Contour_img.ptr<uchar>(0);
		}
		else if (Voxel_Labeling_Method == 2)
		{
			lb_data_ptr = ref_voxel_label_img.ptr<ushort>(0);
			contour_data_ptr = ref_voxel_Contour_img.ptr<uchar>(0);
		}

		//2d에서 3d로 바꾸는 실험 - 카메라 자세에 의한 방향벡터(직선)의 공간상의 한점 Pl구하기
		Mat voxel_3dPt_img = Mat::zeros(down_img.rows, down_img.cols, CV_32FC3); //3dpt를 이미지로 저장하자.
		//vector < vector<DB_Point> > vec_eachLabelPts;
		//vec_eachLabelPts.resize(ss);

		vector<Point2d> vec_features_2d;
		vector<Point3d> vec_features_3d;
		cv::Mat R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
		cv::Mat t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix

		for (int yy = 0; yy < label_img.rows; ++yy)
		{
			for (int xx = 0; xx < label_img.cols; ++xx)
			{
				ushort val2 = lb_data_ptr[yy*label_img.cols + xx];
				ushort val = voxel_label_img.at<ushort>(Point(xx, yy));
				if (val == 0) continue;

				//printf("%d, %d\n", (int)val, (int)val2);

				cv::Mat Point2f_vec = cv::Mat(3, 1, CV_64FC1);
				Point2f_vec.at<double>(0) = xx; //focal_len = focal_len / scale;, scale = img.rows / reSize_H;												
				Point2f_vec.at<double>(1) = yy;
				Point2f_vec.at<double>(2) = 1;
				cv::Mat point_3d = AP_mat_inv * Point2f_vec;
				point_3d.at<double>(0) = point_3d.at<double>(0) / point_3d.at<double>(3);
				point_3d.at<double>(1) = point_3d.at<double>(1) / point_3d.at<double>(3);
				point_3d.at<double>(2) = point_3d.at<double>(2) / point_3d.at<double>(3);
				point_3d.at<double>(3) = point_3d.at<double>(3) / point_3d.at<double>(3);

				//직선과 평면의 교점(Pi) 구하기
				float pt1[3] = { t[0], t[1], t[2] };  //월드좌표계상에서 카메라 중심 좌표(카메라 위치)
				float pt2[3] = { point_3d.at<double>(0), point_3d.at<double>(1), point_3d.at<double>(2) };  //공간상의 한점 Pl 
				float vv[3] = { pt2[0] - pt1[0], pt2[1] - pt1[1], pt2[2] - pt1[2] };  //직선의 방정식의 방향벡터
				float mag = sqrt((vv[0] * vv[0]) + (vv[1] * vv[1]) + (vv[2] * vv[2]));
				float v[3] = { vv[0], vv[1], vv[2] };  //직선의 방정식의 방향 단위 벡터
				float n[3] = { coefs[val - 1][0], coefs[val - 1][1], coefs[val - 1][2] };
				float v_dot_n = v[0] * n[0] + v[1] * n[1] + v[2] * n[2];
				if (v_dot_n == 0)
				{
					printf("내적이 0 이므로, 직선과 평면이 나란한 경우다!\n");
				}
				//float tt = (Pp_Pl[0] / v_dot_n)*n[0] + (Pp_Pl[1] / v_dot_n)*n[1] + (Pp_Pl[2] / v_dot_n)*n[2];
				float tt = (n[0] * pt1[0] + n[1] * pt1[1] + n[2] * pt1[2] + coefs[val - 1][3])
					/ (n[0] * (pt1[0] - pt2[0]) + n[1] * (pt1[1] - pt2[1]) + n[2] * (pt1[2] - pt2[2]));

				float Pi[3] = { pt1[0] + tt * v[0], pt1[1] + tt * v[1], pt1[2] + tt * v[2] };


				//구해진 교점 다시 재사영시켜서 좌표 맞는지 확인하기.
				cv::Mat Point3f_vec2 = cv::Mat(4, 1, CV_64FC1);
				Point3f_vec2.at<double>(0) = Pi[0];
				Point3f_vec2.at<double>(1) = Pi[1];
				Point3f_vec2.at<double>(2) = Pi[2];
				Point3f_vec2.at<double>(3) = 1;
				cv::Mat Point2f_vec2 = cv::Mat(3, 1, CV_64FC1);
				Point2f_vec2 = AP_mat * Point3f_vec2;

				cv::Point2f Point2f;
				Point2f.x = (float)(Point2f_vec2.at<double>(0) / Point2f_vec2.at<double>(2));
				Point2f.y = (float)(Point2f_vec2.at<double>(1) / Point2f_vec2.at<double>(2));
				int xxx = Point2f.x + 0.5;
				int yyy = Point2f.y + 0.5;

				DB_Point tmpPts;
				tmpPts.clusterID = val;
				tmpPts.x_2d = xxx;
				tmpPts.y_2d = yyy;
				tmpPts.x = Pi[0];
				tmpPts.y = Pi[1];
				tmpPts.z = Pi[2];
				tmpPts.pt_3d_id = -1;


				voxel_3dPt_img.at<Vec3f>(yy, xx)[0] = Pi[0];         //vec3d는 double형 벡터
				voxel_3dPt_img.at<Vec3f>(yy, xx)[1] = Pi[1];
				voxel_3dPt_img.at<Vec3f>(yy, xx)[2] = Pi[2];

				float x = tmpPts.x;
				float y = tmpPts.y;
				float z = tmpPts.z;

				vec_features_2d.push_back(Point2d(xx, yy));
				vec_features_3d.push_back(Point3d(x, y, z));

				if (abs(x) > 5000 || abs(y) > 5000 || abs(z) > 5000)
					printf("x:%d, y:%d, - x:%d, y:%d, -- X:%f, Y:%f, Z:%f\n", xx, yy, xxx, yyy, x, y, z);
				//printf("x:%d, y:%d, - x:%d, y:%d, -- X:%f, Y:%f, Z:%f\n", xx, yy, xxx, yyy, x, y, z);

				int debug = 0;
			}
		}


		bool correspondence = EstimatePoseByPnP(vec_features_2d, vec_features_3d,
			DB_img.focal_len, A_matrix, R_matrix, t_matrix);

		float t1 = t_matrix.at<double>(0);
		float t2 = t_matrix.at<double>(1);
		float t3 = t_matrix.at<double>(2);

		GetQuaternionRotationByPnP(R_matrix, t_matrix, q);

		float q2[4];
		camera_data[i].GetQuaternionRotation(q2);

		float dot_prod = q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3];
		if (dot_prod > 1) dot_prod = 1;
		if (dot_prod < -1) dot_prod = -1;
		float theta2 = 2 * acos(dot_prod) * 180 / M_PI;

		float error_x = sqrt(pow(t1 - camera_data[i].t[0], 2) + pow(t2 - camera_data[i].t[1], 2) + pow(t3 - camera_data[i].t[2], 2));
		printf("id:%d, Error XYZ (m):%f, theta:%f\n", i, error_x, theta2);
		vec_error_t.push_back(error_x);
		vec_error_r.push_back(theta2);

		//printf("555555555555555555555555555555555555555555555555555555555555555555555555555555555555\n");

		/*
			그 이미지에 해당하는 feature points 확인하기...
		*/
		//Mat gt_kp_label_img = Mat::zeros(down_img.rows, down_img.cols, CV_8U);
		//for (int j = 0; j < size; ++j)
		//{
		//	Scalar cvColor(255, 0, 0);
		//	float feature_x = features_2d[i][j].x + c_w + 0.5;
		//	float feature_y = features_2d[i][j].y + c_h + 0.5;

		//	feature_x = feature_x / (1920.0 / down_img.cols);
		//	feature_y = feature_y / (1080.0 / down_img.rows);

		//	//circle(kp_label_img, Point(feature_x, feature_y), 1, cvColor, -1);
		//	gt_kp_label_img.data[(int)feature_y*gt_kp_label_img.cols + (int)feature_x] = 255;
		//}
		//imshow("GT kp_label_img", gt_kp_label_img);
		//waitKey(0);



		//vector<string> line_str = split(names[i], '.');
		//vector<string> img_path_split = split(line_str[0], '_');
		//string save_path_rgb = absPath + img_path_split[0] + ".png";
		//string save_path = absPath + img_path_split[0] + "_label" + ".png";
		//string save_path2 = absPath + img_path_split[0] + "_kp" + ".png";
		//string save_path3 = absPath + img_path_split[0] + "_3dpts" + ".yml";
		//string save_path4 = absPath + img_path_split[0] + "_contour" + ".png";
		//string save_path5 = absPath + img_path_split[0] + "_fillcontour" + ".png";
		//cout << save_path << endl;

		//if (Voxel_Labeling_Method == 1)
		//{
		//	imwrite(save_path_rgb, down_img2);
		//	imwrite(save_path, voxel_label_img);
		//	imwrite(save_path2, kp_label_img);
		//	imwrite(save_path4, voxel_Contour_img);
		//	imwrite(save_path5, voxel_fillContour_img);
		//}
		//else if (Voxel_Labeling_Method == 2)
		//{
		//	imwrite(save_path_rgb, down_img2);
		//	imwrite(save_path, ref_voxel_label_img);
		//	imwrite(save_path2, kp_label_img);
		//	imwrite(save_path4, ref_voxel_Contour_img);
		//	imwrite(save_path5, ref_voxel_fillContour_img);
		//}
		//else
		//{
		//	imwrite(save_path_rgb, down_img2);
		//	imwrite(save_path, label_img);
		//	imwrite(save_path2, kp_label_img);
		//}


		//FileStorage fs_w(save_path3, FileStorage::WRITE);
		//fs_w << "voxel_3dPt_img" << voxel_3dPt_img;  //choose any key here, just be consistant with the one below
		//fs_w.release();

		////text파일로 저장하는 방법 - pt_2d_id는 저장 안함. -> 이미지 사이즈가 클 수록... 저장할 포인트가 많아지기 때문에.. 문제당...
		//oStream_dbData << DB_img.img_ID << " " << DB_img.img_path << " " << DB_img.quat[0] << " " << DB_img.quat[1]
		//	<< " " << DB_img.quat[2] << " " << DB_img.quat[3] << " " << DB_img.camCent[0] << " " << DB_img.camCent[1]
		//	<< " " << DB_img.camCent[2] << " " << DB_img.focal_len << " ";

		//int ref_pt_size_sum = 0;
		//for (int lb_id = 0; lb_id < DB_img.voxel_db_pt.size(); ++lb_id)
		//{
		//	int lb_pt_sz = DB_img.voxel_db_pt[lb_id].size();
		//	ref_pt_size_sum += lb_pt_sz;
		//}
		//oStream_dbData << ref_pt_size_sum << endl;

		//for (int lb_id = 0; lb_id < DB_img.voxel_db_pt.size(); ++lb_id)
		//{
		//	int lb_pt_sz = DB_img.voxel_db_pt[lb_id].size();
		//	for (int pt_id = 0; pt_id < lb_pt_sz; ++pt_id)
		//	{
		//		oStream_dbData << DB_img.voxel_db_pt[lb_id][pt_id].x_2d << " " << DB_img.voxel_db_pt[lb_id][pt_id].y_2d
		//			<< " " << DB_img.voxel_db_pt[lb_id][pt_id].x << " " << DB_img.voxel_db_pt[lb_id][pt_id].y << " " << DB_img.voxel_db_pt[lb_id][pt_id].z
		//			<< " " << DB_img.voxel_db_pt[lb_id][pt_id].clusterID << " " << DB_img.voxel_db_pt[lb_id][pt_id].pt_3d_id << endl;
		//	}

		//}



	}
	oStream_dbData.close();

	double sum = std::accumulate(vec_error_t.begin(), vec_error_t.end(), 0.0);
	double mean = sum / vec_error_t.size();

	double sum_q = std::accumulate(vec_error_r.begin(), vec_error_r.end(), 0.0);
	double mean_q = sum_q / vec_error_r.size();

	printf("vec_error_t mean:%f, mean_q:%f \n", mean, mean_q);

	for (int x = 0; x < NUM_COLOR; x++)
	{
		delete[] color[x];
	}
	delete[] color;

	for (int x = 0; x < nLabel; x++)
	{
		delete[] coefs[x];
	}
	delete[] coefs;

	printf("DBSCAN finish!\n");

	return true;
}


void SaveNVM(const char* filename, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc)
{
	std::cout << "Saving model to " << filename << "...\n";
	ofstream out(filename);

	out << "NVM_V3_R9T\n" << camera_data.size() << '\n' << std::setprecision(12);
	if (names.size() < camera_data.size()) names.resize(camera_data.size(), string("unknown"));
	if (ptc.size() < 3 * point_data.size()) ptc.resize(point_data.size() * 3, 0);

	////////////////////////////////////
	for (size_t i = 0; i < camera_data.size(); ++i)
	{
		CameraT& cam = camera_data[i];
		out << names[i] << ' ' << cam.GetFocalLength() << ' ';
		for (int j = 0; j < 9; ++j) out << cam.m[0][j] << ' ';
		out << cam.t[0] << ' ' << cam.t[1] << ' ' << cam.t[2] << ' '
			<< cam.GetNormalizedMeasurementDistortion() << " 0\n";
	}

	out << point_data.size() << '\n';

	for (size_t i = 0, j = 0; i < point_data.size(); ++i)
	{
		Point3D& pt = point_data[i];
		int * pc = &ptc[i * 3];
		out << pt.xyz[0] << ' ' << pt.xyz[1] << ' ' << pt.xyz[2] << ' '
			<< pc[0] << ' ' << pc[1] << ' ' << pc[2] << ' ';

		size_t je = j;
		while (je < ptidx.size() && ptidx[je] == (int)i) je++;

		out << (je - j) << ' ';

		for (; j < je; ++j)    out << camidx[j] << ' ' << " 0 " << measurements[j].x << ' ' << measurements[j].y << ' ';

		out << '\n';
	}
}



bool LoadModelFile(const char* name, vector<CameraT>& camera_data, vector<Point3D>& point_data,
	vector<Point2D>& measurements, vector<int>& ptidx, vector<int>& camidx,
	vector<string>& names, vector<int>& ptc, int cubeSize)
{
	if (name == NULL)return false;
	ifstream in(name);

	std::cout << "Loading cameras/points: " << name << "\n";
	if (!in.is_open()) return false;

	if (strstr(name, ".nvm"))return LoadNVM(in, camera_data, point_data, measurements, ptidx, camidx, names, ptc, cubeSize);
}


void plane_normVec(DB_Plane& plane)
{
	double x1 = plane.x[3] - plane.x[0];
	double y1 = plane.y[3] - plane.y[0];
	double z1 = plane.z[3] - plane.z[0];

	double x2 = plane.x[1] - plane.x[0];
	double y2 = plane.y[1] - plane.y[0];
	double z2 = plane.z[1] - plane.z[0];

	//surface noramal vector
	double x3 = y1 * z2 - z1 * y2;
	double y3 = z1 * x2 - x1 * z2;
	double z3 = x1 * y2 - y1 * x2;
	//plane.surfaceNomalX = x3;
	//plane.surfaceNomalY = y3;
	//plane.surfaceNomalZ = z3;

	double l = sqrt(x3*x3 + y3 * y3 + z3 * z3);
	plane.surfaceNomalX = x3 / l;
	plane.surfaceNomalY = y3 / l;
	plane.surfaceNomalZ = z3 / l;

	//nomalized vector
	plane.equation.a = x3 / l;
	plane.equation.b = y3 / l;
	plane.equation.c = z3 / l;
	plane.equation.d = -(plane.equation.a*plane.x[0] + plane.equation.b*plane.y[0] + plane.equation.c*plane.z[0]);
}