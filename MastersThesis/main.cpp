/*
 * @file main.cpp
 * @link https://github.com/HShigehara/Masters-Thesis.git 
 * @brief main�֐�
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* �w�b�_�t�@�C���̃C���N���[�h */
#include "3DPathTrackingUsingtheKINECT.hpp"
#include "KinectMethod.hpp"
#include "ImageProcessingMethod.hpp"
#include "RouteDrawingMethod.hpp"
#include "System.hpp"
#include "LeastSquareMethod.hpp" //(c49)

/* �O���[�o���ϐ� */
//�摜�f�[�^
Mat image; //!<RGB�摜�i�[�p�̕ϐ�

char directoryName[NOC]; //!<�t�H���_��

//(c6)
int extractedPointOneDim[ALLPIXEL]; //!<���o���ꂽ���W��1�����̒l
int extractedNum; //(c6)colorExtraction()�ŉ��s�N�Z�����o���ꂽ���J�E���g����ϐ�

Point3ius extCoordinate[ALLPIXEL]; //���o���ꂽ���W(c37)

Vector4 XYZCoordinate[ALLPIXEL]; //3�����ɕϊ����ꂽ(X,Y,Z)(c49)

//(c25)
Rect trackWindow; //!<�ǐՃE�C���h�E
int hsize = 16;
float hranges[] = { 0, 180 }; //!<H�͈̔�
const float* phranges = hranges;
bool backprojMode = false; //!<�o�b�N�v���W�F�N�g���[�h
bool selectObject = false; //!<�I�u�W�F�N�g�I��
int trackObject = 0; //!<�ǐՂ���I�u�W�F�N�g
Point origin; //!<�I���W�i���̍��W
Rect selection; //!<�I��
int vmin = 10, vmax = 256, smin = 30; //!<HSV�͈͎̔w��
void onMouse(int event, int x, int y, int flags, void* param); //!<�}�E�X����

//(c26)
/*int*/bool avgFlag; //!<���ς��v�Z�����Ƃ��p�̃t���O(c30)
/*int*/bool mouseFlag; //!<�}�E�X����m�F�p�̃t���O(c26)

/*!
 * @brief �֐�main()
 * @param �Ȃ�
 * @return �Ȃ�
 */
int main()
{
RETRY: //goto��.�v������肭�����Ȃ������烊�Z�b�g����p
	//�ϐ��̐錾
	int checkNum; //!<�v���O�����I�����Ƀf�[�^��ۑ����邩�m�F���邽�߂̕ϐ�(c38)
	int countDataNum; //�o�͂��ꂽ�f�[�^�����J�E���g����(c39)
	outputData outputData[OUTPUTDATA_MAX]; //!<�o�͂���f�[�^��錾�D�ő�10000��(c41)

	//�t�@�C�����̒�`(c39)
	char* outputDataName = "3d.dat"; //�v���f�[�^���o�͂����t�@�C����(c39)
	char* cenerofgravityDataName = "cog.dat"; //�d�S���W���o�͂����t�@�C����(c52)

	//�摜�֌W
	Mat mHSVforObjectTracking_img; //!<�q�X�g�O�����쐬�̂��߂�HSV�ϊ���̃f�[�^�ۑ��p
	Mat mHSVforTrim_img; //!<�؂����̉摜�i�[�p(c31)
	Mat mHSVColorExtraction_img; //!<(ce:color extraction).HSV�ɕϊ�����p�̕ϐ�

	//�I�[�v�j���O�����p�ɂƂ��Ă���
	Mat mGray_img; //!<�O���[�X�P�[���p�̕ϐ�(c19)
	Mat mBin_img; //!<���o������ɓ�l�������摜��ۑ�����ϐ�(c21)
	Mat mOpening_img; //!<�I�[�v�j���O���s�����摜���狗���𒊏o����(c24).�I�[�v�j���O�������s���ɂ͕K�v*/
	Mat mExtractedBlack_img; //!<�I�[�v�j���O��̓�l�摜���璊�o���ꂽ�������W���i�[���Ă���ϐ�(c40)

	//�C���X�^���X�̐���
	System sys; //!<�V�X�e���I�ȃ��\�b�h���܂Ƃ߂Ă���N���X
	RouteDrawing routedraw; //!<RouteDrawing�N���X�̃C���X�^���X�𐶐�
	LeastSquareMethod lsm; //!<�ŏ����@���s���N���X�̃C���X�^���X�𐶐�(c49)

	//���C���̏���
	try{
		sys.startMessage(); //�v���O�����J�n���̃��b�Z�[�W��\��

		Kinect kinect; //Kinect�N���X�̃C���X�^���X�𐶐�
		ImageProcessing imgproc; //Imageprocessing�N���X�̃C���X�^���X�𐶐�
		

		//���W�֌W�̕ϐ��̒�`
		Vector4 realPoint; //!<�ϊ���̐��E���W�n�̒l���i�[
		Point3ius averageCoordinate; //!<���ύ��W���i�[����N���X���̃��[�J���ϐ�(c38)

		//�ϐ��̐錾
		double sumTime; //���v�̎��Ԃ��J�E���g����ϐ�
		double time; //1�t���[��������̎���(c39)
		double fps; //�t���[�����[�g(c39)

		//�E�C���h�E���ƃt�@�C�����̒�`
		char* mainWindowName = "���摜"; //���C���E�C���h�E�̖��O�����Ă����D(c31)
		char* outputVideoName = "video.avi"; //�v�����̓���t�@�C����(c39)

		//xml�t�@�C���̓ǂݍ���
		char* cameraParameterName = "cameraParam.xml"; //�J�����L�����u���[�V�����ɂ���ē���ꂽ�t�@�C����(c54)
		sys.loadInternalCameraParameter(cameraParameterName); //�J�����p�����[�^��ǂݍ���(c54)
		Mat undistort_img;

		//�^�C�}�[�p�ϐ�
		int64 end; //�I�����̃^�C�}�[

		//�t���O

		//�ϐ��̏�����
		countDataNum = 0;
		sumTime = 0.0; //���v�̎��Ԃ��J�E���g����ϐ�
		time = 0.0; //1�t���[���̎���
		realPoint.x = 0.0; //3�������W��x���W
		realPoint.y = 0.0; //3�������W��y���W
		realPoint.z = 0.0; //3�������W��x���W
		avgFlag = /*0*/false; //�Čv���̂��߂ɕ��ύ��W���v�Z�������`�F�b�N����t���O�ϐ���������
		mouseFlag = /*0*/false; //�Čv���̂��߂Ƀ}�E�X���N���b�N���������`�F�b�N����t���O�ϐ���������

		sys.makeDirectory(); //�N���������t�H���_���ɂ��ăt�H���_���쐬

		//������o��(c40)
		//char outputVideoPath[NOC]; //!<����o�͎��̃p�X(c38)
		//sprintf_s(outputVideoPath, "%s/%s", directoryName, outputVideoName); //(c38)
		//VideoWriter writer(outputVideoPath, /*CV_FOURCC('D','I','B',' ')*/-1/*CV_FOURCC('X','V','I','D')*//*CV_FOURCC('P','M','I','1')*/, 20, /*Size(WIDTH, HEIGHT)*/Size(640, 480), true); //����ɏo��.�^�悪�K�v�ȂƂ��̓R�����g�A�E�g(c35)
		//if (!writer.isOpened()){ //�I�[�v���G���[����(c40)
		//	cerr << outputVideoPath << " is Not Opened." << endl;
		//}


		kinect.initialize(); //Kinect�̏�����

		namedWindow(mainWindowName, CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO); //���摜�p�̃E�C���h�E��\��(c31)
		namedWindow("�c�ݕ␳��", CV_WINDOW_AUTOSIZE || CV_WINDOW_FREERATIO);

		while (1){ //(c3).���C�����[�v�D1�t���[�����Ƃ̏������J��Ԃ��s���D
			//(c25)
			setMouseCallback(mainWindowName, onMouse, 0); //�}�E�X�R�[���o�b�N�֐����Z�b�g(c31)

			//�f�[�^�̍X�V��҂�
			DWORD ret = ::WaitForSingleObject(kinect.streamEvent, INFINITE); //�t���[���X�V���C�x���g�Ƃ��đ҂�
			::ResetEvent(kinect.streamEvent); //�C�x���g�����������玟�̃C�x���g�ɔ����ă��Z�b�g

			//�^�C�}�[�𓱓��D�X�^�[�g�n�_(c18)
			double f = 1000.0 / getTickFrequency();
			int64 start = getTickCount(); //�X�^�[�g

			//Kinect�����E�摜����
			kinect.drawRGBImage(image); //RGB�J�����̏���
			
			//�c�ݕ␳��̉摜�ɑ΂��ď������s���悤�ɂ���
			undistort(image, undistort_img, sys.internalCameraParam, sys.distortionCoefficients, Mat()); //�c�ݕ␳��̉摜�ŏ㏑��(c54)
			imshow("�c�ݕ␳��", undistort_img);
			//image = undistort_img.clone();

			//�摜��\��
			imshow(mainWindowName, image);

			//���C���̏���(c26)(c30)
			if (mouseFlag == true){ //mouseFlag��true�ł����=�}�E�X�̃{�^������ɏオ������
				//writer << image; //�����ۑ�(c35)
				mHSVforObjectTracking_img = imgproc.convertRGB2HSV(image); //CamShift�ŗ��p����q�X�g�O�������쐬���邽�߂�HSV�֕ϊ�(c29)
				imgproc.trackingObject(mHSVforObjectTracking_img); //�摜��ǐՂ��郁�\�b�h�ֈڍs(c26)
				mHSVforTrim_img = imgproc.convertRGB2HSV(imgproc.mTrim_img); //�]�T�������Đ؂���ꂽ�摜��HSV�ɕϊ�����(c33)
				mHSVColorExtraction_img = imgproc.extractColor(mHSVforTrim_img); //�]�T�������Đ؂������摜�������̐F�𒊏o����(c33)

				//�I�[�v�j���O���s���Ȃ�K�v�ȏ���(c30)
				//�O���[�X�P�[���ϊ�(c19)
				mGray_img = imgproc.grayscaleImage(mHSVColorExtraction_img);
				//���o�����s�N�Z�����炳��ɐ��x�����߂邽�߂ɖc���E���k���s��(c19)
				mBin_img = imgproc.binarizationImage(mGray_img); //���o�����摜�̓�l��
				mOpening_img = imgproc.OpeningImage(mBin_img); //�I�[�v�j���O����(c19)

				mExtractedBlack_img = imgproc.getCoordinate(mOpening_img);
				//�m�F�p
				namedWindow("�v���Ώۂ̓_", CV_WINDOW_KEEPRATIO);
				//namedWindow("�v���Ώۂ̓_", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
				imshow("�v���Ώۂ̓_", mExtractedBlack_img);

				//�ŏ����@���s��(��)(c49)
				//lsm.getSphereData(kinect.actualExtractedNum); //�ŏ����@�̏����ֈڍs(c49)

				//imgproc.showImage("extractedCoordinate", mExtractedBlack_img);

				averageCoordinate = kinect.getAverageCoordinate(image); //Depth�J�����̏���(c5)

				imgproc.drawCenterPoint(image, averageCoordinate, mainWindowName); //���ύ��W��\�����郁�\�b�h(c45)

				if (avgFlag == true){ //���ύ��W�����o����Ă�����
					realPoint = kinect.getLocalPosition(averageCoordinate); //1�t���[��������̕��ύ��W��3�������W���擾����(c10)(c28)
					end = getTickCount();
					time = (end - start) * f;
					sumTime += time;
					fps = 1000.0 / time; //�t���[�����[�g���v�Z

					putText(image, to_string((int)fps) + "fps", Point(WIDTH - 100, HEIGHT - 28), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2, CV_AA); //�����Ƀt���[�����[�g���o��

					outputData[countDataNum].totalTime = sumTime; //�v�����ꂽ�g�[�^���̎���(c41)
					outputData[countDataNum].x = realPoint.x; //Kinect���W�n�ɕϊ����ꂽx(c41)
					outputData[countDataNum].y = realPoint.y; //Kinect���W�n�ɕϊ����ꂽy(c41)
					outputData[countDataNum].z = realPoint.z; //Kinect���W�n�ɕϊ����ꂽz(c41)

					//���o���ꂽ�f�[�^��\��
					//cout << "X => " << realPoint.x << "\tY => " << realPoint.y << "\tZ => " << realPoint.z << endl;
					//routedraw.plot3DRealTime(countDataNum, outputData); //�ʒu�����A���^�C���Ńv���b�g����(c43)
					countDataNum++; //�o�͂��ꂽ�f�[�^�����J�E���g����(c41)
				}
				else{ //���W�����o����Ă��Ȃ��Ƃ��D�I�[�v�j���O�������s���Ă���Ƃ��͔������̓�l�摜�Ȃ̂ł����ɂ��邱�Ƃ͂Ȃ�(c41)
					end = getTickCount(); //�^�C�}�[�I��(c41)
				}
				//imgproc.showTogetherImage(mHSVforTrim_img, mHSVColorExtraction_img); //2�̉摜��1�̃E�C���h�E�ɕ\���D�m�F�p(c36)
			}
			else{ //�}�E�X���N���b�N����Ă��Ȃ��Ƃ��̃^�C�}�[�I��(c41)
				end = getTickCount(); //�^�C�}�[�I��
			}

			//�I���̂��߂̃L�[���̓`�F�b�N���\���̂��߂̃E�F�C�g�^�C��
			kinect.key = waitKey(1);
			if (kinect.key == 'q'){ //�v���I��
				sys.outputAllData(outputDataName, outputData, countDataNum);
				//routedraw.plot3D(outputDataName); //(c4)
				destroyAllWindows();
				break;
			}
			else if (kinect.key == 'r'){ //�Čv������Ƃ��ɑO�̃t�@�C�����폜���Ă���(c31)
				cv::destroyAllWindows(); //OpenCV�ō쐬�����E�C���h�E��S�č폜����(c35)
				sys.removeDirectory();
				cout << "Data Removed." << endl;
				goto RETRY;
			}
		}
	}

	catch (exception& ex){ //��O����
		cout << ex.what() << endl;
		destroyAllWindows(); //OpenCV�ō쐬�����E�C���h�E��S�č폜����(c35)
		//�ُ�I���������̓f�[�^��ۑ�����K�v���Ȃ��̂ō폜
		sys.removeDirectory();
		cout << "Data Removed." << endl;
		return -1;
	}

	//�f�[�^��ۑ����邩�̊m�F(c27)
	cout << "Save Data?" << endl;
	checkNum = sys.alternatives(); //'1'�Ȃ�ۑ��C'0'�Ȃ�폜


	if (checkNum == 0){ //�폜����Ƃ�(c55)
		sys.removeDirectory(/*checkNum*/); //�f�B���N�g�����폜���邩�ǂ���
	}
	else{ //�ۑ�����Ƃ�
		routedraw.gnuplotScript(/*checkNum, */outputDataName); //���3D���W���v���b�g����p��gnuplot�X�N���v�g����邩�ǂ���
		routedraw.gnuplotScriptCoG(/*checkNum, */cenerofgravityDataName); //��ŏd�S���W���v���b�g����p��gnuplot�X�N���v�g����邩�ǂ���(c52)
	}

	sys.endMessage(checkNum);

	return 0;
}