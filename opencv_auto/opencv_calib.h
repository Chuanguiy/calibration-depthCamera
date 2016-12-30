#ifndef CALIB
#define CALIB
void save_result(CvMat*cam_rotation_all, CvMat*cam_translation_vector_all,
	CvMat*cam_intrinsic_matrix, CvMat*cam_distortion_coeffs,char*pathc,int sucesses);
void calib_yang(CvMat* cam_intrinsic_matrix, CvMat* cam_distortion_coeffs,char *camera);
#endif