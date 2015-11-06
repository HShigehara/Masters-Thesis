/*
* @file KinectMethod.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git 
* @brief kinect�����̃N���X�̃w�b�_
* @date 2014.12.10
* @author H.Shigehara
*/

/* �C���N���[�h�K�[�h */
#ifndef __KINECTMETHOD_HPP__
#define __KINECTMETHOD_HPP__

/* �C���N���[�h */
#include "3DPathTrackingUsingtheKINECT.hpp"

/*! @brief �G���[�`�F�b�N */
#define ERROR_CHECK(ret)\
if (ret != S_OK){ \
stringstream ss; \
ss << "faild" #ret " " << hex << ret << endl; \
throw runtime_error(ss.str().c_str()); \
}

/*! Kinect�̉𑜓x�̐ݒ� */
const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

/*!
* @class Kinect
* @brief Kinect����p�̃N���X
*/
class Kinect
{
private:
	INuiSensor* kinect; //!<INuiSensor�^�̃|�C���^
	HANDLE imageStreamHandle; //!<RGB�J�����̃X�g���[���f�[�^���������߂̃n���h��
	HANDLE depthStreamHandle; //!<Depth�J�����̃X�g���[���f�[�^���������߂̃n���h��
	DWORD width; //!<��
	DWORD height; //!<����

	int countKinect; //!<�ڑ�����Ă���Kinect�̐����J�E���g����ϐ�

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //!<�_�Q�f�[�^�ۑ��p(c57)

public:
	Kinect(); //!<�R���X�g���N�^
	~Kinect(); //!<�f�X�g���N�^
	
	HANDLE streamEvent; //!<RGB,Depth�J�����̃t���[���X�V�C�x���g��҂��߂̃C�x���g�n���h��
	int key; //!<�E�B���h�E�\���̃E�F�C�g�^�C���i�[�ϐ�

	int actualExtractedNum; //���ۂɋ��������o���ꂽ��(0�ȊO��������)(c31)

	void initialize(); //!<Kinect�̏�����
	void createInstance(); //!<�C���X�^���X�̐���

	Mat drawRGBImage(Mat& image); //!<RGB�J�����̏���
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud(Mat& Mat_image); //!<Depth�J�����̏���(c57)
	Point3ius getAverageCoordinate(Mat& image); //!<Depth�J�����̏���
	
	int getDistance(Mat& image); //!<�������擾(c49)


	Vector4 getLocalPosition(Point3ius averageCoordinate); //!<(x,y,z)����3�������W��(X,Y,Z)���v�Z(c10)
	
	void showDepthImage(); //!<�����摜��\��
};

/* �C���N���[�h�K�[�h�̏I�� */
#endif /* __KINECTMETHOD_HPP__ */