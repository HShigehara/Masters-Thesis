/*
 * @file KinectMethod.cpp
 * @link https://github.com/HShigehara/Masters-Thesis.git
 * @brief Kinect���������߂̃��\�b�h�Q
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* �w�b�_�t�@�C���̃C���N���[�h */
#include "3DPathTrackingUsingtheKINECT.hpp" //�w�b�_�t�@�C���̃C���N���[�h
#include "KinectMethod.hpp"

/*!
 * @brief ���\�b�hKinect::Kinect().�R���X�g���N�^
 * @param �Ȃ�
 * @return �Ȃ�
 */
Kinect::Kinect()
{
	countKinect = 0; //Kinect�̐���������
}

/*!
 * @brief ���\�b�hKinect::~Kinect().�f�X�g���N�^
 * @param �Ȃ�
 * @return �Ȃ�
 */
Kinect::~Kinect()
{
	//�I������
	if (kinect != 0){
		kinect->NuiShutdown();
		kinect->Release();
	}
}

/*!
 * @brief ���\�b�hKinect::createInstance().�C���X�^���X�̐���
 * @param �Ȃ�
 * @return �Ȃ�
 */
void Kinect::createInstance()
{
	//�ڑ�����Ă���Kinect�̐����擾����
	ERROR_CHECK(::NuiGetSensorCount(&countKinect));
	if (countKinect == 0){
		throw runtime_error("Please Connect the Kinect.");
	}

	ERROR_CHECK(::NuiCreateSensorByIndex(0, &kinect)); //�ŏ���Kinect�̃C���X�^���X�𐶐�����

	//Kinect�̏�Ԃ��擾����
	HRESULT status = kinect->NuiStatus();
	if (status != S_OK){
		throw runtime_error("You Cannot Use the Kinect.");

	}

	return;
}

/*!
 * @brief ���\�b�hKinect::initialize().Kinect�̏�����
 * @param �Ȃ�
 * @return �Ȃ�
 */
void Kinect::initialize()
{
	createInstance(); //createInstance()�̏����ֈȍ~

	ERROR_CHECK(kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH)); //Kinect�̐ݒ��������
	ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, CAMERA_RESOLUTION, 0, 2, 0, &imageStreamHandle)); //RGB�J������������
	ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, CAMERA_RESOLUTION, 0, 2, 0, &depthStreamHandle)); //Depth�J������������
	ERROR_CHECK(kinect->NuiImageStreamSetImageFrameFlags(depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE)); //Near���[�h

	//�t���[���X�V�̃C�x���g�n���h�����쐬
	streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
	ERROR_CHECK(kinect->NuiSetFrameEndEvent(streamEvent, 0));

	::NuiImageResolutionToSize(CAMERA_RESOLUTION, width, height); //�w�肵���𑜓x�̉�ʃT�C�Y���擾����

	return;
}

/*!
 * @brief ���\�b�hKinect::drawRGBImage(Mat& image).RGB�J�����̏���
 * @param image
 * @return image
 */
Mat Kinect::drawRGBImage(Mat& image)
{
	try{
		//RGB�J�����̃t���[���f�[�^���擾����
		NUI_IMAGE_FRAME imageFrame = { 0 };
		ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(imageStreamHandle, 0, &imageFrame));

		//�摜�f�[�^�̎擾
		NUI_LOCKED_RECT colorData;
		imageFrame.pFrameTexture->LockRect(0, &colorData, 0, 0);

		//�摜�f�[�^�̃R�s�[
		image = Mat(height, width, CV_8UC4, colorData.pBits);

		//�t���[���f�[�^�̉��
		ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(imageStreamHandle, &imageFrame));
	}
	catch (exception& ex){ //��O����(c57)
		cout << ex.what() << endl;
	}
	return (image); //RGB�J��������摜���擾���Ԃ�(c30)
}

/*!
 * @brief ���\�b�hKinect::getAverageCoordinate(Mat& image).Depth�J�����̏���
 * @param cv::Mat& image
 * @return �Ȃ�
 */
Point3ius Kinect::getAverageCoordinate(Mat& image) //(c31)
{
	Vector4 sumCoordinate; //1�t���[���̊e���W�̑��a�����߂�ϐ��̏�����(c31)
	Point3ius avgCoordinate; //!<���ύ��W���i�[����N���X���̃��[�J���ϐ�(c38)

	//�ϐ��̏�����(c31)
	actualExtractedNum = 0;

	sumCoordinate.x = 0;
	sumCoordinate.y = 0;
	sumCoordinate.z = 0;

	avgCoordinate.x = 0;
	avgCoordinate.y = 0;
	avgCoordinate.z = 0;

	//Depth�J�����̃t���[���f�[�^���擾����
	NUI_IMAGE_FRAME depthFrame = { 0 };
	ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(depthStreamHandle, INFINITE, &depthFrame));

	//Depth�f�[�^���擾
	NUI_LOCKED_RECT depthData = { 0 };
	depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

	USHORT* depth = (USHORT*)depthData.pBits; //depth�f�[�^���i�[

				//cout << actualExtractedNum << endl;
	if (extractedNum > 0){ //���o�������W��1�ȏ゠��Έȉ��̏��������s
		for (int i = 0; i < extractedNum; i++){ //���o���ꂽ���������W�𑫂����킹�Ă���(c37)
			USHORT distance = ::NuiDepthPixelToDepth(depth[extractedPointOneDim[i]]); //distance�̒P�ʂ�mm
			if (distance != 0){ //������0�ȊO�ł���΁C�����ƍ��W�𑫂����킹�C�����ꂽ�����J�E���g(c31)
				sumCoordinate.x += extCoordinate[i].x; //���o���ꂽx���W�𑫂����킹�Ă���(c37)
				sumCoordinate.y += extCoordinate[i].y; //���o���ꂽy���W�𑫂����킹�Ă���(c37)
				sumCoordinate.z += distance; //���o���ꂽz(����)�𑫂����킹�Ă���(c37)

				//cout << distance << endl; (���m�F�p)

				Vector4 worldCoordinate = NuiTransformDepthImageToSkeleton((long)extCoordinate[i].x, (long)extCoordinate[i].y, (USHORT)distance, NUI_IMAGE_RESOLUTION_640x480);
				XYZCoordinate[actualExtractedNum].x = worldCoordinate.x * 1000.0f;
				XYZCoordinate[actualExtractedNum].y = worldCoordinate.y * 1000.0f;
				XYZCoordinate[actualExtractedNum].z = worldCoordinate.z * 1000.0f;

				//cout << XYZCoordinate[actualExtractedNum].z << endl; (���m�F�p)
				actualExtractedNum++; //���ۂɑ������킳�ꂽ�����J�E���g����(c37)
			}
		}
		avgCoordinate.x = /*(float)*/(int)(sumCoordinate.x / actualExtractedNum);
		avgCoordinate.y = /*(float)*/(int)(sumCoordinate.y / actualExtractedNum);
		avgCoordinate.z = /*(USHORT)*/(USHORT)(sumCoordinate.z / actualExtractedNum); //(c11)

		avgFlag = true;
	}
	else
	{
		avgFlag = false; //���ύ��W���v�Z���Ȃ��悤�ɂ���
	}

	ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame));

	return (avgCoordinate);
}

/*!
* @brief ���\�b�hKinect::coordinateTransform.���W�̕ϊ����s�����\�b�h(c10)
* @param �Ȃ�
* @return �Ȃ�
*/
/*void*/Vector4 Kinect::getLocalPosition(Point3ius averageCoordinate)
{
	Vector4 rp; //realPoint�i�[�p�̕ϐ�(c38)
	Vector4 worldCoordinate; //���[���h���W�n(c38)

	//�ϐ��̏�����(c33)
	rp.x = 0.0;
	rp.y = 0.0;
	rp.z = 0.0;
	worldCoordinate.x = 0.0;
	worldCoordinate.y = 0.0;
	worldCoordinate.z = 0.0;

	worldCoordinate = NuiTransformDepthImageToSkeleton((long)averageCoordinate.x, (long)averageCoordinate.y, (USHORT)averageCoordinate.z, NUI_IMAGE_RESOLUTION_640x480);
	rp.x = (float)(worldCoordinate.x *1000.0f);
	rp.y = (float)(worldCoordinate.y *1000.0f);
	rp.z = (float)(worldCoordinate.z *1000.0f);

	return rp;
}

/*
 * @brief ���\�b�hKinect::setDepthImage(Mat& Mat_image).�f�v�X�摜���擾���郁�\�b�h(c57)
 * @param cv::Mat& Mat_image
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr points
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect::getPointCloud(Mat& Mat_image)
{
	try{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>()); //�|�C���g�N���E�h�ۑ��p(c57)
		points->width = width;
		points->height = height;

		Mat_image = Mat(height, width, CV_8UC1, Scalar(0)); //�����摜�̏���

		//�����J�����̃t���[���f�[�^���擾
		NUI_IMAGE_FRAME depthFrame = { 0 };
		ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame));

		//�����f�[�^���擾����
		NUI_LOCKED_RECT depthData = { 0 };
		depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

		USHORT* depth = (USHORT*)depthData.pBits;
		for (int i = 0; i < (depthData.size / sizeof(USHORT)); ++i){
			USHORT distance = ::NuiDepthPixelToDepth(depth[i]);
			LONG depthX = i % width;
			LONG depthY = i / width;
			LONG colorX = depthX;
			LONG colorY = depthY;

			// �����J�����̍��W���ARGB�J�����̍��W�ɕϊ�����
			kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, depthX, depthY, 0, &colorX, &colorY);

			// �����摜�쐬
			Mat_image.at<UCHAR>(colorY, colorX) = distance / 8192.0 * 255.0;

			// �|�C���g�N���E�h
			Vector4 real = NuiTransformDepthImageToSkeleton(depthX, depthY, distance, CAMERA_RESOLUTION);
			pcl::PointXYZRGB point;
			point.x = real.x;
			point.y = -real.y;
			point.z = real.z;
			
			point.r = 255;
			point.g = 255;
			point.b = 255;

			points->push_back(point);
		}
		cloud = points;

		//�t���[���f�[�^���J������(c58)
		ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame));
	}
	catch (exception& ex){
		cout << ex.what() << endl;
	}
	return cloud;
}