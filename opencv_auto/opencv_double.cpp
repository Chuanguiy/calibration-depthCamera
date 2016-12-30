// opencv_double.cpp : �������̨Ӧ�ó������ڵ㡣
//
#include "stdafx.h"
#include"highgui.h"
#include"cv.h"
#include"iostream"
#include"fstream"
#include"cstdlib"
#include"opencv_calib.h"
int main()
{
	//������������������
	CvMat* depth_intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* depth_distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);
	//������ɫ�����������
	CvMat* color_intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* color_distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);

	//�궨
	calib_yang(depth_intrinsic_matrix, depth_distortion_coeffs,"depth");
	//calib_yang(color_intrinsic_matrix, color_distortion_coeffs,"color");
	
	//�ͷž���
	cvReleaseMat(&depth_intrinsic_matrix);
	cvReleaseMat(&depth_distortion_coeffs);

	cvReleaseMat(&color_intrinsic_matrix);
	cvReleaseMat(&color_distortion_coeffs);

	return 0;
}