/*
 * @file ImageProcessingMethod.cpp
 * @link https://github.com/HShigehara/3DPathTrackingUsingtheKINECT.git
 * @brief �摜�������s�����߂̃��\�b�h�Q
 * @date 2014.10.17
 * @author H.Shigehara
 */

/* �w�b�_�t�@�C���̃C���N���[�h */
#include "stdafx.h"
#include "3DPathTrackingUsingtheKINECT.hpp" //�w�b�_�t�@�C���̃C���N���[�h
#include "ImageProcessingMethod.hpp"

/*!
 * @brief ���\�b�hImageProcessing::ImageProcessing().�R���X�g���N�^
 * @param �Ȃ�
 * @return �Ȃ�
 */
ImageProcessing::ImageProcessing()
{
	//�R���X�g���N�^�͍��̂Ƃ���Ȃ�
}

/*!
 * @brief ���\�b�hImageProcessing::~ImageProcessing().�f�X�g���N�^
 * @param �Ȃ�
 * @return �Ȃ�
 */
ImageProcessing::~ImageProcessing()
{
	//�f�X�g���N�^�͍��̂Ƃ���Ȃ�
}

/*!
* @brief ���\�b�hImageProcessing::showImage().cv::Mat��\��
* @param windowName,input_image
* @return �Ȃ�
*/
void ImageProcessing::showImage(char* windowName, Mat& input_image)
{
	namedWindow(windowName, CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
	imshow(windowName, input_image);
	return;
}

/*!
* @brief ���\�b�hImageProcessing::showTogetherImage().2��cv::Mat��1�̃E�C���h�E�ɕ\��
* @param image1,image2
* @return �Ȃ�
*/
void ImageProcessing::showTogetherImage(Mat& image1, Mat& image2)
{
	//�E�C���h�E���ƍ����摜���`
	char* windowName = "��������";
	int winWidth = image1.cols + image2.cols;
	int winHeight = max(image1.rows, image2.rows); //�E�C���h�E�̍����͍������ɍ��킹��
	Mat all_img(Size(winWidth, winHeight), CV_8UC3);

	//�摜������
	Mat mRoi1(all_img, Rect(0, 0, image1.cols, image1.rows));
	Mat mRoi2(all_img, Rect(image1.cols, 0, image2.cols, image2.rows));
	image1.copyTo(mRoi1);
	image2.copyTo(mRoi2);

	showImage(windowName, all_img); //���������摜��\��

	return;
}

/*!
* @brief ���\�b�hImageProcessing::showTogetherImage().3��cv::Mat��1�̃E�C���h�E�ɕ\��
* @param image1,image2,image3
* @return �Ȃ�
*/
void ImageProcessing::showTogetherImage(Mat& image1, Mat& image2, Mat& image3)
{
	//�E�C���h�E���ƍ����摜���`
	char* windowName = "��������";
	int winWidth = image1.cols + image2.cols + image3.cols;
	int winHeight = max({ image1.rows, image2.rows, image3.rows }); //�E�C���h�E�̍����͍������ɍ��킹��
	Mat mAll_img(Size(winWidth, winHeight), CV_8UC3);

	//�摜������
	Mat mRoi1(mAll_img, Rect(0, 0, image1.cols, image1.rows));
	Mat mRoi2(mAll_img, Rect(image1.cols, 0, image2.cols, image2.rows));
	Mat mRoi3(mAll_img, Rect(image1.cols + image2.cols, 0, image3.cols, image3.rows));
	image1.copyTo(mRoi1);
	image2.copyTo(mRoi2);
	image3.copyTo(mRoi3);

	showImage(windowName, mAll_img); //���������摜��\��

	return;
}

/*!
 * @brief ���\�b�hImageProcessing::convertRGB2HSV().RGB�摜��HSV�ɕϊ�����
 * @param input_image
 * @return hsvimg
 */
Mat ImageProcessing::convertRGB2HSV(Mat& input_image)
{
	Mat smoothimg; //!<�t�B���^�������ɗ��p����ϐ�(c17)
	Mat hsvimg; //!<HSV�摜�i�[�p�ϐ�(c17)
	medianBlur(input_image, smoothimg, 7); //���f�B�A���t�B���^�ŕ������D(�����l7)
	cvtColor(smoothimg, hsvimg, CV_BGR2HSV); //HSV�ɕϊ�
	return (hsvimg);
}

/*!
* @brief ���\�b�hImageProcessing::colorExtraction().HSV�摜�������̐F(��)�𒊏o���āC�����̉摜�𐶐�����
* @param input_hsvimg
* @return hsvceimg
*/
Mat ImageProcessing::extractColor(Mat& input_hsvimg)
{
	Mat hsvceimg; //!<�F���o��̉摜

	hsvceimg = Mat(Size(input_hsvimg.cols, input_hsvimg.rows), CV_8UC3); //HSV���o��p�̒�`.image�Ɠ������ƍ����ɂ�,3�`�����l���ɂ���
	//hsv_ceimg = Scalar(0, 0, 0); //�w�i�����ŏ�����
	hsvceimg = Scalar(255, 255, 255); //�w�i�𔒂ŏ�����

	//(c3)
	for (int y = 1; y < input_hsvimg.rows; y++)
	{
		for (int x = 1; x < input_hsvimg.cols; x++){
			int px = input_hsvimg.step*y + (x * 3);
			//if (input_hsvimg.data[px] >= 110 && input_hsvimg.data[px] <= 123 && input_hsvimg.data[px + 1] >= 80 && input_hsvimg.data[px + 2] >= 40){ //���\��������(���{�[��)
			if (input_hsvimg.data[px] >= 110 && input_hsvimg.data[px] <= 120 && input_hsvimg.data[px + 1] >= 80 && input_hsvimg.data[px + 2] >= 45){ //���\��������(���{�[��)
				//���o���ꂽ�F���킩��₷���F�ɕύX����(�I�����W��[px]=0;[px+1]=150;)
				hsvceimg.data[px] = 0;
				hsvceimg.data[px + 1] = 0;
				hsvceimg.data[px + 2] = 255;
			}
		}
	}

	return (hsvceimg);
}

/*!
* @brief ���\�b�hImageProcessing::getCoordinate().���W�𒊏o���郁�\�b�h
* @param input_binimg
* @return extblackimage
*/
Mat ImageProcessing::getCoordinate(Mat& input_binimg)
{
	Mat extblackimg;
	//(c33)
	for (int i = 0; i < /*ALLPIXEL*/extractedNum; i++){
		extCoordinate[i].x = 0;
		extCoordinate[i].y = 0;
		extCoordinate[i].z = 0;
		extractedPointOneDim[i] = 0;
	}
	extractedNum = 0; //(c6)1�t���[�����Ƃɒ��o���ꂽ�s�N�Z�����J�E���g���邽�߂�extractedNum��������

	extblackimg = Mat(Size(input_binimg.cols, input_binimg.rows), CV_8UC1); //
	//extblackimg = Scalar(0, 0, 0); //�w�i�����ŏ�����
	extblackimg = Scalar(255, 255, 255); //�w�i�𔒂ŏ�����

	//(c3)
	//�擾������W�f�[�^���Ԉ����Dfor(��,��<��,��++)��for(��,��<��,��+=�����ɂ���)(c48)
	for (int y = 1; y < input_binimg.rows; y+=1)
	{
		for (int x = 1; x < input_binimg.cols; x+=1){
			int px = input_binimg.step * y + x;
			if (input_binimg.data[px] ==  0){ //���\��������(���{�[��)
				//���o���ꂽ�F���킩��₷���F�ɕύX����(�I�����W��[px]=0;[px+1]=150;)
				extblackimg.data[px] = 0;

				//(c6)�D���o�����摜�ɑ΂��č��W���擾����ꍇ�ɂ͗��p����D(c24)�ł̓I�[�v�j���O�����̌�ɍ��W���擾�������̂ŃR�����g�A�E�g
				extractedPointOneDim[extractedNum] = (trackWindow.x + x - 1) + WIDTH * (trackWindow.y + y - 1) - WIDTH; //���o���ꂽ(x,y)��1�����̒l�ɕϊ�
				extCoordinate[extractedNum].x = trackWindow.x + x - 1; //���o���ꂽx���W(c8)
				extCoordinate[extractedNum].y = trackWindow.y + y - 1; //���o���ꂽy���W(c8)
				//cout << extCoordinate[extractedNum].x << " , " << extCoordinate[extractedNum].y << endl;
				extractedNum++;
			}
		}
	}
	return (extblackimg);
}

/*!
* @brief ���\�b�hImageProcessing::grayscaleImage().���o�����摜���O���[�X�P�[���ɕϊ�����
* @param cv::Mat& hsv_ceimg
* @return grayimg
*/
Mat ImageProcessing::grayscaleImage(Mat& hsv_ceimg)
{
	Mat grayimg;
	cvtColor(hsv_ceimg, grayimg, CV_RGB2GRAY); //���o�����摜���O���[�X�P�[���ɂ���
	//showImage("GrayImage", grayimg); //�m�F�p(c40)
	return (grayimg);
}

/*!
* @brief ���\�b�hImageProcessing::binarizationImage().����̐F�𒊏o�����摜�ɑ΂��ē�l���������s��
* @param gray_img
* @return binimg
*/
Mat ImageProcessing::binarizationImage(Mat& gray_img)
{
	Mat binimg;
	cv::threshold(gray_img, binimg, 0, 255, THRESH_BINARY | THRESH_OTSU); //�O���[�X�P�[���̉摜���l������
	//showImage("BinarizationImage", binimg); //�m�F�p(c40)
	return (binimg);
}

/*!
* @brief ���\�b�hImageProcessing::OpeningImage().����̐F�𒊏o���ē�l�������摜�ɑ΂��ăI�[�v�j���O�������s��
* @param bin_img
* @return openingimg
*/
Mat ImageProcessing::OpeningImage(Mat& bin_img)
{
	Mat openingimg;
	morphologyEx(bin_img, openingimg, MORPH_OPEN, Mat(), Point(-1, -1), 2); //�I�[�v�j���O(�k�����c��)����
	//showImage("MorphologyEX", openingimg); //�m�F�p(c40)
	return (openingimg);
}

/*!
* @brief ���\�b�hImageProcessing::trackObject().�Ώۂ�ǐՂ��郁�\�b�h(c26)
* @param Mat& hsv_img
* @return �Ȃ�
*/
void ImageProcessing::trackingObject(Mat& hsv_img)
{
	if (trackObject)
	{
		inRange(hsv_img, Scalar(0, smin, MIN(_vmin, _vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);

		//Hue�̒��o
		int ch[] = { 0, 0 };
		hue.create(hsv_img.size(), hsv_img.depth());
		mixChannels(&hsv_img, 1, &hue, 1, ch, 1);

		//��`�̈�̑I��
		if (trackObject < 0)
		{
			//�g���b�L���O�E�C���h�E��ݒ�
			trackWindow = selection;
			trackObject = 1;

			//�q�X�g�O�������v�Z
			Mat roi(hue, selection), maskroi(mask, selection);
			calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
			normalize(hist, hist, 0, 255, CV_MINMAX);
		}

		//�o�b�N�v���W�F�N�V����
		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
		backproj &= mask;
		
		//CamShift�A���S���Y��
		RotatedRect trackBox = CamShift(backproj, trackWindow, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));

		if (trackWindow.area() <= 1)
		{
			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
			trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,trackWindow.x + r, trackWindow.y + r) & Rect(0, 0, cols, rows);
		}

		if (backprojMode)
		{
			cvtColor(backproj, image, CV_GRAY2BGR);
		}

		//�摜��؂���ۂɁC�]�T�������Đ؂���悤�ɂ���(c33).�������C���̏ꍇ�C(x,y)<(10,10)���Ɛ؂�����W���}�C�i�X�ɂȂ�OpenCV�̃G���[���N���邽�߁C�ꍇ����������
		if (trackWindow.x - 10 > 0 && trackWindow.y - 10 > 0){ //(x,y) > (10,10)�ł���΁C�㉺���E�]�T�������Đ؂��邱�Ƃ��ł���
			Mat trim(image, Rect(trackWindow.x - 10, trackWindow.y - 10, trackWindow.width + 20, trackWindow.height + 20)); //�摜��؂���ہC�͈͂��L���Ă���(c33)
			mTrim_img = trim.clone(); //�]�T�������Đ؂������摜�����S�ɃR�s�[����(c33)
		}
		else{ //(x,y) < (10,10)�ł���΁C�؂���ۂɃG���[���N���邽�߁C���Ə�͂��̂܂ܐ؂���D�E�Ɖ��͗]�T�������Đ؂���D(c33)
			Mat trim(image, Rect(trackWindow.x, trackWindow.y, trackWindow.width + 20, trackWindow.height + 20)); //
			mTrim_img = trim.clone(); //�؂������摜�����S�ɃR�s�[����(c33)
		}

		//�ǐՂ���~�̕\��
		ellipse(image, trackBox, Scalar(0, 0, 255), 2, CV_AA);
	}

	//�I��̈��\��
	if (selectObject && selection.width > 0 && selection.height > 0)
	{
		Mat roi(image, selection);
		bitwise_not(roi, roi);
	}

	return;
}

/*!
* @brief ���\�b�hImageProcessing::drawCenterPoint().���S���W��`�悷�郁�\�b�h(c26)
* @param �Ȃ�
* @return �Ȃ�
*/
void ImageProcessing::drawCenterPoint(Mat& image, Point3ius averageCoordinate, char* mainWindowName)
{
	//Mat mAveragePoint; //���ύ��W��`�悷��ϐ�(c45)

	//mAveragePoint = image.clone();

	//circle(mAveragePoint, Point2i(averageCoordinate.x, averageCoordinate.y), 1, Scalar(0, 0, 255), 2, CV_AA);
	circle(image, Point2i(averageCoordinate.x, averageCoordinate.y), 1, Scalar(0, 0, 255), 2, CV_AA);
	//imshow(mainWindowName, mAveragePoint);
	//imshow(mainWindowName, image);

	return;
}