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
	/*初始化数据	图像组数,角点个数,棋盘格信息*/
	CvMat*cam_object_points2;
	CvMat*cam_image_points2;
	bool def;
	int cam_board_n;
	int img_num=16, cam_board_w=9, cam_board_h=9,cam_Dx=50,cam_Dy=50;
	// 图像组数，横轴提取角点的个数，纵轴提取角点的个数，需要提取区域宽度、高度
	cout<<"使用默认参数？"//\n\n图像前缀名：cam\n"
		<<"图片组数：16\n棋盘内角点：9x9\n尺寸：50mmx50mm"
		<<endl;
	cin >>def;
	if(!def)
	{
		cout << "输入的图像的组数\n";
		cin >> img_num;
		cout << "输入*真实*棋盘格的##横轴##方向的角点个数\n";
		cin >> cam_board_w;
		cout << "输入*真实*棋盘格的##纵轴##方向的角点个数\n";
		cin >> cam_board_h;
		cout << "输入*真实*棋盘格的##横轴##方向的长度\n";
		cin >> cam_Dx;
		cout << "输入*真实*棋盘格的##纵轴##方向的长度"<<endl;
		cin >> cam_Dy;
	}
	cam_board_n = cam_board_w*cam_board_h;


	//相机初始化
	CvSize cam_board_sz = cvSize(cam_board_w, cam_board_h);
	CvMat*cam_image_points = cvCreateMat(cam_board_n*(img_num), 2, CV_32FC1);
	CvMat*cam_object_points = cvCreateMat(cam_board_n*(img_num), 3, CV_32FC1);
	CvMat*cam_point_counts = cvCreateMat((img_num), 1, CV_32SC1);
	CvPoint2D32f*cam_corners = new CvPoint2D32f[cam_board_n];//存储点坐标
	int cam_corner_count;
	int cam_step;

	CvSize cam_image_sz; // 结构体——每行和每列的角点个数
	cvNamedWindow("calib", CV_WINDOW_AUTOSIZE);//显示窗体初始化

	//获得图像尺寸
	char path[50];
	sprintf(path,"..\\cam_%s\\cam1.jpg",camera);
	IplImage *cam_image_temp = cvLoadImage(path, 0);//默认路径工程子文件夹cam
	cam_image_sz = cvGetSize(cam_image_temp);
	char failurebuf[20] = { 0 };

	//提取角点

	fstream cam_data;//实例文件对象cam_data
	cam_data.open("..\\output\\TXT\\cam_corners.txt", ofstream::out);
	fstream cam_object_data;
	cam_object_data.open("..\\output\\TXT\\cam_object_data.txt", ofstream::out);
	//加工图像以便后期处理
  process:
	int successes = 0;
	for (int ii = 1; ii < img_num + 1; ii++)
	{
		sprintf(path, "..\\cam_%s\\cam%d.jpg",camera,ii);
		IplImage *cam_image = cvLoadImage(path, 0);//读取图片
		//确定是否棋盘格图片，并记录角点位置
		int cam_found = cvFindChessboardCorners(cam_image, cam_board_sz, cam_corners, &cam_corner_count,
			 CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		//精确寻找内角点位置，精度0.01
		cvFindCornerSubPix(cam_image, cam_corners, cam_corner_count,
			cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001));
		//绘制检测到的棋盘角点
		cvDrawChessboardCorners(cam_image, cam_board_sz, cam_corners, cam_corner_count, cam_found);

		if (cam_corner_count != cam_board_n)//所有角点都检测到
			cout << "find cam"<<ii<<" corner failed!\n";
		
		//所有图片处理成功保存结果
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
		cout<< "请确认标定板信息并重新输入\n"
			<< "输入*真实*棋盘格的##横轴##方向的角点个数\n";
		cin >> cam_board_w;
		cout << "输入*真实*棋盘格的##纵轴##方向的角点个数\n";
		cin >> cam_board_h;
		cout << "输入*真实*棋盘格的##横轴##方向的长度\n";
		cin >> cam_Dx;
		cout << "输入*真实*棋盘格的##纵轴##方向的长度"<<endl;
		cin >> cam_Dy;
		goto process;//角点提取失败，重新开始
	}
	/*
	//还原成功检测的点
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
	//标定相机
	//
	*/
	CV_MAT_ELEM(*cam_intrinsic_matrix, float, 0, 0) = 1.0f;
	CV_MAT_ELEM(*cam_intrinsic_matrix, float, 1, 1) = 1.0f;
	CvMat* cam_rotation_all = cvCreateMat( successes, 3, CV_32FC1);
	CvMat* cam_translation_vector_all = cvCreateMat( successes,3, CV_32FC1);
	//利用标定计算相机内外参数
	cvCalibrateCamera2(
		cam_object_points2,//记录每个角点物理坐标的矩阵
		cam_image_points2,//记录角点像素坐标的矩阵
		cam_point_counts2,
		cam_image_sz,
		cam_intrinsic_matrix,//指向内参数矩阵的指针，
		cam_distortion_coeffs,//指向畸变系数矩阵的指针，顺序 k1,k2,p1,p2,k3
		cam_rotation_all,
		cam_translation_vector_all,
		0//CV_CALIB_FIX_ASPECT_RATIO  
		);
	cvSave("..\\output\\XML\\cam_intrinsic_matrix.xml", cam_intrinsic_matrix);
	cvSave("..\\output\\XML\\cam_distortion_coeffs.xml", cam_distortion_coeffs);
	//标定
	cvSave("..\\output\\XML\\cam_rotation_all.xml", cam_rotation_all);
	cvSave("..\\output\\XML\\cam_translation_vector_all.xml", cam_translation_vector_all);
	char path1[100] = "..\\output\\result_data_no_optim.txt";
	save_result(cam_rotation_all, cam_translation_vector_all,
		cam_intrinsic_matrix, cam_distortion_coeffs,path1,successes);
	system("pause>nul");
	cvDestroyWindow("calib");
}
//保存标定结果
void save_result(CvMat*cam_rotation_all, CvMat*cam_translation_vector_all,
	CvMat*cam_intrinsic_matrix, CvMat*cam_distortion_coeffs,char*pathc,int sucesses)
{
	fstream Yeah_result; //定义文件流
	Yeah_result.open(pathc, ofstream::out);
	Yeah_result << setprecision(12) << "fc[0] =" << CV_MAT_ELEM(*cam_intrinsic_matrix, float, 0, 0
		) << "; fc[1] =" << CV_MAT_ELEM(*cam_intrinsic_matrix, float, 1, 1) << "; //CAM的焦距\n";
	Yeah_result  << setprecision(12) << "cc[0] = " << CV_MAT_ELEM(*cam_intrinsic_matrix, float, 0, 2) 
		<< "; cc[1] = " << CV_MAT_ELEM(*cam_intrinsic_matrix, float, 1, 2) << ";//CAM中心点\n";
	Yeah_result  << setprecision(12) << "畸变参数：\nkc[0](k1) =" << CV_MAT_ELEM(*cam_distortion_coeffs, float, 0, 0) <<
		";  kc[1](k2) =" << CV_MAT_ELEM(*cam_distortion_coeffs, float, 1, 0) <<
		" \nkc[2](p1) =" << CV_MAT_ELEM(*cam_distortion_coeffs, float, 2, 0) << 
		";  kc[3](p2) =" << CV_MAT_ELEM(*cam_distortion_coeffs, float, 3, 0) <<
		"\n kc[4](k3) =" << CV_MAT_ELEM(*cam_distortion_coeffs, float, 4, 0) <<
		"\n\n\n\n\n\n外参数矩阵:\n"<<endl;
		for(int i=0;i<sucesses;++i)
		{
			Yeah_result<<"r:("<<setprecision(12) <<CV_MAT_ELEM(*cam_rotation_all, float, i, 0)<<"\t,"<<CV_MAT_ELEM(*cam_rotation_all, float, i, 1)<<"\t,"<<CV_MAT_ELEM(*cam_rotation_all, float, i, 2)<<")\n";
			Yeah_result<<"t:("<<setprecision(12) <<CV_MAT_ELEM(*cam_translation_vector_all, float, i, 0)<<"\t,"<<CV_MAT_ELEM(*cam_translation_vector_all, float, i, 1)<<"\t,"<<CV_MAT_ELEM(*cam_translation_vector_all, float, i, 2)<<")\n\n\n";
		}	
}