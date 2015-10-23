/*
* @file ImageProcessingMethod.hpp 
* @link https://github.com/HShigehara/Masters-Thesis.git 
* @brief �摜�����֘A�̃N���X�̃w�b�_
* @date 2014.12.10
* @author H.Shigehara
*/

/* �C���N���[�h�K�[�h */
#ifndef __IMAGEPROCESSINGMETHOD_HPP__
#define __IMAGEPROCESSINGMETHOD_HPP__

/* �C���N���[�h */
#include "3DPathTrackingUsingtheKINECT.hpp"

/*!
* @class ImageProcessing
* @brief �摜�����p�̃N���X
*/
class ImageProcessing
{
private:
	//Mat hsvceimg; //!<�F���o���HSV�摜
	/* (c26) */
	Mat hue;
	Mat mask;
	Mat hist;
	Mat backproj;
	int _vmin = vmin;
	int _vmax = vmax;

public:
	ImageProcessing(); //!<�R���X�g���N�^
	~ImageProcessing(); //!<�f�X�g���N�^
	void showImage(char* windowName, Mat& input_image); //!<�E�C���h�E�̖��O�������ɒǉ�(c31)�DMat�̕\��(c17)
	//�����̉摜��1�̃E�C���h�E�ɂ܂Ƃ߂ĕ\������D�I�[�o�[���[�h(c36)
	void showTogetherImage(Mat& image1, Mat& image2); //!<2�̉摜���ꏏ�ɕ\��(c36)
	void showTogetherImage(Mat& image1, Mat& image2, Mat& image3); //!<3�̉摜���ꏏ�ɕ\��(c36)
	Mat convertRGB2HSV(Mat& input_image); //!<RGB�摜image��HSV�ɕϊ����郁�\�b�h
	Mat extractColor(Mat& input_hsvimg); //!<�ϊ�����HSV�摜�������̐F�𒊏o���郁�\�b�h
	Mat getCoordinate(Mat& input_binimg); //!<��l�摜������W�𒊏o���郁�\�b�h(c40)
	/*�I�[�v�j���O�����p�ɂƂ��Ă����D(c31)*/
	//Mat extractBlack(Mat& opening_img); //!<�I�[�v�j���O������̉摜������W�𒊏o����(c24)
	Mat grayscaleImage(Mat& hsv_ceimg); //!<�O���[�X�P�[���ɕϊ�(c19)
	Mat binarizationImage(Mat& gray_img); //!<���o�����摜�̓�l��(c19)
	Mat OpeningImage(Mat& bin_img); //!<���o���ē�l�������摜�ɑ΂��ăI�[�v�j���O����(�����������k�����c��)���s��(c19)
	void trackingObject(Mat& hsv_img); //!<�Ώۂ�ǐՂ���(c26)


	void drawCenterPoint(Mat& image, Point3ius averageCoordinate/*, const string* mainWindowName*/); //!<�v�Z�������ύ��W����ʏ�ɕ\������(c45)

	Mat mTrim_img;
};

/* �C���N���[�h�K�[�h�̏I�� */
#endif /* __IMAGEpROCESSINGMETHOD_HPP__ */