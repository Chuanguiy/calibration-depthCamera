//function:auto calibration
#include "stdafx.h"
#include"highgui.h"
#include"cv.h"
#include"iostream"
#include"fstream"
#include"cstdlib"
#include"opencv_calib.h"
using namespace std;

void calib_yang(CvMat* cam_intrinsic_matrix, CvMat* cam_distortion_coeffs,char *camera)
{
	/*��ʼ������	ͼ������,�ǵ����,���̸���Ϣ*/
	CvMat*cam_object_points2;
	CvMat*cam_image_points2;
	bool def;
	int cam_board_n;
	int img_num=16, cam_board_w=9, cam_board_h=9,cam_Dx=50,cam_Dy=50;
	// ͼ��������������ȡ�ǵ�ĸ�����������ȡ�ǵ�ĸ�������Ҫ��ȡ�����ȡ��߶�
	cout<<"ʹ��Ĭ�ϲ�����"//\n\nͼ��ǰ׺����cam\n"
		<<"ͼƬ������16\n�����ڽǵ㣺9x9\n�ߴ磺50mmx50mm"
		<<endl;
	cin >>def;
	if(!def)
	{
		cout << "�����ͼ�������\n";
		cin >> img_num;
		cout << "����*��ʵ*���̸��##����##����Ľǵ����\n";
		cin >> cam_board_w;
		cout << "����*��ʵ*���̸��##����##����Ľǵ����\n";
		cin >> cam_board_h;
		cout << "����*��ʵ*���̸��##����##����ĳ���\n";
		cin >> cam_Dx;
		cout << "����*��ʵ*���̸��##����##����ĳ���"<<endl;
		cin >> cam_Dy;
	}
	cam_board_n = cam_board_w*cam_board_h;


	//�����ʼ��
	CvSize cam_board_sz = cvSize(cam_board_w, cam_board_h);
	CvMat*cam_image_points = cvCreateMat(cam_board_n*(img_num), 2, CV_32FC1);
	CvMat*cam_object_points = cvCreateMat(cam_board_n*(img_num), 3, CV_32FC1);
	CvMat*cam_point_counts = cvCreateMat((img_num), 1, CV_32SC1);
	CvPoint2D32f*cam_corners = new CvPoint2D32f[cam_board_n];//�洢������
	int cam_corner_count;
	int cam_step;

	CvSize cam_image_sz; // �ṹ�塪��ÿ�к�ÿ�еĽǵ����
	cvNamedWindow("calib", CV_WINDOW_AUTOSIZE);//��ʾ�����ʼ��

	//���ͼ��ߴ�
	char path[50];
	sprintf(path,"..\\cam_%s\\cam1.jpg",camera);
	IplImage *cam_image_temp = cvLoadImage(path, 0);//Ĭ��·���������ļ���cam
	cam_image_sz = cvGetSize(cam_image_temp);
	char failurebuf[20] = { 0 };

	//��ȡ�ǵ�

	fstream cam_data;//ʵ���ļ�����cam_data
	cam_data.open("..\\output\\TXT\\cam_corners.txt", ofstream::out);
	fstream cam_object_data;
	cam_object_data.open("..\\output\\TXT\\cam_object_data.txt", ofstream::out);
	//�ӹ�ͼ���Ա���ڴ���
  process:
	int successes = 0;
	for (int ii = 1; ii < img_num + 1; ii++)
	{
		sprintf(path, "..\\cam_%s\\cam%d.jpg",camera,ii);
		IplImage *cam_image = cvLoadImage(path, 0);//��ȡͼƬ
		//ȷ���Ƿ����̸�ͼƬ������¼�ǵ�λ��
		int cam_found = cvFindChessboardCorners(cam_image, cam_board_sz, cam_corners, &cam_corner_count,
			 CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		//��ȷѰ���ڽǵ�λ�ã�����0.01
		cvFindCornerSubPix(cam_image, cam_corners, cam_corner_count,
			cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001));
		//���Ƽ�⵽�����̽ǵ�
		cvDrawChessboardCorners(cam_image, cam_board_sz, cam_corners, cam_corner_count, cam_found);

		if (cam_corner_count != cam_board_n)//���нǵ㶼��⵽
			cout << "find cam"<<ii<<" corner failed!\n";
		
		//����ͼƬ����ɹ�������
		if ( cam_corner_count == cam_board_n) 
		{			
			cam_step = successes*cam_board_n;
			for (int i = cam_step, j = 0; j < cam_board_n; ++i, ++j) 
			{
				CV_MAT_ELEM(*cam_image_points, float, i, 0) = cam_corners[j].x;
				CV_MAT_ELEM(*cam_image_points, float, i, 1) = cam_corners[j].y;
				CV_MAT_ELEM(*cam_object_points, float, i, 0) = (j/cam_board_w)*cam_Dx;
				CV_MAT_ELEM(*cam_object_points, float, i, 1) = (j % cam_board_w)*cam_Dy;
				CV_MAT_ELEM(*cam_object_points, float, i, 2) = 0.0f;
				cam_data << cam_corners[j].x << "\t" << cam_corners[j].y << "\n";
				cam_object_data << (j/cam_board_w)*cam_Dx << "\t" << (j %cam_board_w)*cam_Dy << "\t0\n";
			}
			CV_MAT_ELEM(*cam_point_counts, int, successes, 0) = cam_board_n;
			successes++;
			cout << "success number" << successes << endl;
			cvShowImage("calib", cam_image);
			cvWaitKey(1000);
		}				
	}
	if (successes < 10)
	{
		cout<< "��ȷ�ϱ궨����Ϣ����������\n"
			<< "����*��ʵ*���̸��##����##����Ľǵ����\n";
		cin >> cam_board_w;
		cout << "����*��ʵ*���̸��##����##����Ľǵ����\n";
		cin >> cam_board_h;
		cout << "����*��ʵ*���̸��##����##����ĳ���\n";
		cin >> cam_Dx;
		cout << "����*��ʵ*���̸��##����##����ĳ���"<<endl;
		cin >> cam_Dy;
		goto process;//�ǵ���ȡʧ�ܣ����¿�ʼ
	}
	/*
	//��ԭ�ɹ����ĵ�
	*/
	cam_image_points2 = cvCreateMat(cam_board_n*(successes), 2, CV_32FC1);
	cam_object_points2 = cvCreateMat(cam_board_n*(successes), 3, CV_32FC1);
	CvMat*cam_point_counts2 = cvCreateMat((successes), 1, CV_32SC1);
	for (int i = 0; i < successes*cam_board_n; ++i)
	{
		CV_MAT_ELEM(*cam_image_points2, float, i, 0) = CV_MAT_ELEM(*cam_image_points, float, i, 0);
		CV_MAT_ELEM(*cam_image_points2, float, i, 1) = CV_MAT_ELEM(*cam_image_points, float, i, 1);
		CV_MAT_ELEM(*cam_object_points2, float, i, 0) = CV_MAT_ELEM(*cam_object_points, float, i, 0);
		CV_MAT_ELEM(*cam_object_points2, float, i, 1) = CV_MAT_ELEM(*cam_object_points, float, i, 1);
		CV_MAT_ELEM(*cam_object_points2, float, i, 2) = CV_MAT_ELEM(*cam_object_points, float, i, 2);

	}
	for (int i = 0; i < successes; ++i)
	{
		CV_MAT_ELEM(*cam_point_counts2, int, i, 0) = CV_MAT_ELEM(*cam_point_counts, int, i, 0);
	}
	cvSave("..\\output\\XML\\cam_corners.xml", cam_image_points2);

	cvReleaseMat(&cam_object_points);
	cvReleaseMat(&cam_image_points);
	cvReleaseMat(&cam_point_counts);

	
	/*
	//�궨���
	//
	*/
	CV_MAT_ELEM(*cam_intrinsic_matrix, float, 0, 0) = 1.0f;
	CV_MAT_ELEM(*cam_intrinsic_matrix, float, 1, 1) = 1.0f;
	CvMat* cam_rotation_all = cvCreateMat( successes, 3, CV_32FC1);
	CvMat* cam_translation_vector_all = cvCreateMat( successes,3, CV_32FC1);
	//���ñ궨��������������
	cvCalibrateCamera2(
		cam_object_points2,//��¼ÿ���ǵ���������ľ���
		cam_image_points2,//��¼�ǵ���������ľ���
		cam_point_counts2,
		cam_image_sz,
		cam_intrinsic_matrix,//ָ���ڲ��������ָ�룬
		cam_distortion_coeffs,//ָ�����ϵ�������ָ�룬˳�� k1,k2,p1,p2,k3
		cam_rotation_all,
		cam_translation_vector_all,
		0//CV_CALIB_FIX_ASPECT_RATIO  
		);
	cvSave("..\\output\\XML\\cam_intrinsic_matrix.xml", cam_intrinsic_matrix);
	cvSave("..\\output\\XML\\cam_distortion_coeffs.xml", cam_distortion_coeffs);
	//�궨
	cvSave("..\\output\\XML\\cam_rotation_all.xml", cam_rotation_all);
	cvSave("..\\output\\XML\\cam_translation_vector_all.xml", cam_translation_vector_all);
	char path1[100] = "..\\output\\result_data_no_optim.txt";
	save_result(cam_rotation_all, cam_translation_vector_all,
		cam_intrinsic_matrix, cam_distortion_coeffs,path1,successes);
	system("pause>nul");
	cvDestroyWindow("calib");
}
//����궨���
void save_result(CvMat*cam_rotation_all, CvMat*cam_translation_vector_all,
	CvMat*cam_intrinsic_matrix, CvMat*cam_distortion_coeffs,char*pathc,int sucesses)
{
	fstream Yeah_result; //�����ļ���
	Yeah_result.open(pathc, ofstream::out);
	Yeah_result << setprecision(12) << "fc[0] =" << CV_MAT_ELEM(*cam_intrinsic_matrix, float, 0, 0
		) << "; fc[1] =" << CV_MAT_ELEM(*cam_intrinsic_matrix, float, 1, 1) << "; //CAM�Ľ���\n";
	Yeah_result  << setprecision(12) << "cc[0] = " << CV_MAT_ELEM(*cam_intrinsic_matrix, float, 0, 2) 
		<< "; cc[1] = " << CV_MAT_ELEM(*cam_intrinsic_matrix, float, 1, 2) << ";//CAM���ĵ�\n";
	Yeah_result  << setprecision(12) << "���������\nkc[0](k1) =" << CV_MAT_ELEM(*cam_distortion_coeffs, float, 0, 0) <<
		";  kc[1](k2) =" << CV_MAT_ELEM(*cam_distortion_coeffs, float, 1, 0) <<
		" \nkc[2](p1) =" << CV_MAT_ELEM(*cam_distortion_coeffs, float, 2, 0) << 
		";  kc[3](p2) =" << CV_MAT_ELEM(*cam_distortion_coeffs, float, 3, 0) <<
		"\n kc[4](k3) =" << CV_MAT_ELEM(*cam_distortion_coeffs, float, 4, 0) <<
		"\n\n\n\n\n\n���������:\n"<<endl;
		for(int i=0;i<sucesses;++i)
		{
			Yeah_result<<"r:("<<setprecision(12) <<CV_MAT_ELEM(*cam_rotation_all, float, i, 0)<<"\t,"<<CV_MAT_ELEM(*cam_rotation_all, float, i, 1)<<"\t,"<<CV_MAT_ELEM(*cam_rotation_all, float, i, 2)<<")\n";
			Yeah_result<<"t:("<<setprecision(12) <<CV_MAT_ELEM(*cam_translation_vector_all, float, i, 0)<<"\t,"<<CV_MAT_ELEM(*cam_translation_vector_all, float, i, 1)<<"\t,"<<CV_MAT_ELEM(*cam_translation_vector_all, float, i, 2)<<")\n\n\n";
		}	
}