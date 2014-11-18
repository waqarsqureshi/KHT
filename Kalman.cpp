#pragma once
#include "Kalman.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
TKalmanFilter::TKalmanFilter(Point2f pt,float dt,float Accel_noise_mag)
{
	//���������� ������� (��� ������ ��� ��������, ��� "��������" ����)
	deltatime = dt; //0.2
	
	// ��������� �� �� �����, ������� ������� ��� � ���� ��������.
	// ���� �� ����� ������������, ����� �������� ��������� ����� ������ ������������� ������. 
	// ��� ��������. (����������� ���������� �������� ���������: �/�^2)
	// ����������, ��������� ������ ������ ����� ����������.
	//float Accel_noise_mag = 0.5; 

	//4 ���������� ���������, 2 ���������� ���������
	kalman = new KalmanFilter( 4, 2, 0 );  
	// ������� ��������
	kalman->transitionMatrix = (Mat_<float>(4, 4) << 1,0,deltatime,0,   0,1,0,deltatime,  0,0,1,0,  0,0,0,1);

	// init... 
	LastResult = pt;
	kalman->statePre.at<float>(0) = pt.x; // x
	kalman->statePre.at<float>(1) = pt.y; // y

	kalman->statePre.at<float>(2) = 0;
	kalman->statePre.at<float>(3) = 0;

	kalman->statePost.at<float>(0)=pt.x;
	kalman->statePost.at<float>(1)=pt.y;

	setIdentity(kalman->measurementMatrix);

	kalman->processNoiseCov=(Mat_<float>(4, 4) <<pow(deltatime,4.0)/4.0,0,pow(deltatime,3.0)/2.0,0,0,pow(deltatime,4.0)/4.0,0,pow(deltatime,3.0)/2.0,pow(deltatime,3.0)/2.0	,0						,pow(deltatime,2.0)			,0,
	0,pow(deltatime,3.0)/2.0,0,pow(deltatime,2.0));


	kalman->processNoiseCov*=Accel_noise_mag;

	setIdentity(kalman->measurementNoiseCov, Scalar::all(0.1));

	setIdentity(kalman->errorCovPost, Scalar::all(.1));

}
//---------------------------------------------------------------------------
TKalmanFilter::~TKalmanFilter()
{
	delete kalman;
}

//---------------------------------------------------------------------------
Point2f TKalmanFilter::GetPrediction()
{
	Mat prediction = kalman->predict();
	LastResult=Point2f(prediction.at<float>(0),prediction.at<float>(1)); 
	return LastResult;
}
//---------------------------------------------------------------------------
Point2f TKalmanFilter::Update(Point2f p, bool DataCorrect)
{
	Mat measurement(2,1,CV_32FC1);
	if(!DataCorrect)
	{
		measurement.at<float>(0) = LastResult.x;  //�������� ��������� ������������
		measurement.at<float>(1) = LastResult.y;
	}
	else
	{
		measurement.at<float>(0) = p.x;  //��������, ��������� ������ ���������
		measurement.at<float>(1) = p.y;
	}
	// ���������
		Mat estimated = kalman->correct(measurement);
		LastResult.x=estimated.at<float>(0);  //��������, ��������� ������ ���������
		LastResult.y=estimated.at<float>(1);
	return LastResult;
}
//---------------------------------------------------------------------------