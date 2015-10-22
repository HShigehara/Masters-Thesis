/*
* @file System.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief kinect�����̃N���X�̃w�b�_
* @date 2014.12.19
* @author H.Shigehara
*/

/* */
#include "3DPathTrackingUsingtheKINECT.hpp"
#include "System.hpp"

/*!
* @brief System::System().�R���X�g���N�^
* @param �Ȃ�
* @return �Ȃ�
*/
System::System()
{
	//�R���X�g���N�^�̏����͂Ȃ�
}

/*!
* @brief System::~System().�f�X�g���N�^
* @param �Ȃ�
* @return �Ȃ�
*/
System::~System()
{
	//�f�X�g���N�^�̏����͂Ȃ�
}

/*!
* @brief System::startMessage().�v���O�����N�����̃��b�Z�[�W
* @param �Ȃ�
* @return �Ȃ�
*/
void System::startMessage()
{
	cout << "==================================================================" << endl;
	cout << " Starting the Program...." << endl;
	cout << " Please Enclose the Object You Want to Track." << endl;
	cout << " If You Enter a 'q' Key, the Program Terminates. (On the Window.)" << endl;
	cout << " If You Enter a 'r' Key, the Program Restart. (On the Window.)" << endl;
	//cout << " To Initialize Tracking, Re-Select the Object with Mouse." << endl;
	cout << "==================================================================\n" << endl;

	return;
}

/*!
* @brief System::endMessage().�v���O�����I�����̃��b�Z�[�W
* @param �Ȃ�
* @return �Ȃ�
*/
void System::endMessage(int cNum)
{
	cout << "==================================================================" << endl;
	cout << "Closing the Program...." << endl;
	if (cNum == 1){
		cout << "Data Has Been Output to \"" << directoryName << "\"." << endl;
		openDirectory();
	}
	cout << "==================================================================" << endl;
	
	return;
}


/*!
* @brief System::makeDirectory().�f�B���N�g�����쐬
* @param �Ȃ�
* @return �Ȃ�
*/
void System::makeDirectory()
{
	//�t�H���_������ʂ��邽�߂Ɏ������擾���Ă���(c4)
	SYSTEMTIME st;

	GetLocalTime(&st);
	sprintf_s(directoryName, "[%4d%02d%02d]%02d_%02d_%02d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
	_mkdir(directoryName);

	return;
}

/*!
* @brief System::removeDirectory().�f�B���N�g�����폜(c21)
* @param �Ȃ� 
* @return �Ȃ�
*/
void System::removeDirectory()
{
	char rmdirCommand[NOC]; //�f�B���N�g�����폜����R�}���h(c21).�ϐ�����ύX&���̃��\�b�h�݂̂ŗL���ȕϐ�(c30)
	//�f�B���N�g�����폜����
	sprintf_s(rmdirCommand, "rmdir /s /q %s", directoryName);
	system(rmdirCommand);
	cout << "Not Save.\n" << endl;

	return;
}

/*!
* @brief System::alternatives().Yes/No��2���̃`�F�b�N(c21)
* @param �Ȃ�
* @return checkNum
*/
int System::alternatives()
{
	char checkNum; //!<0��1���`�F�b�N���邽�߂̕ϐ�(c27).���̃��\�b�h�݂̂ŗL���ȕϐ�(c30)

	cout << "1. Yes" << endl;
	cout << "0. No" << endl;
	cin >> checkNum;
	cout << "\n";

	while (1)
	{
		if (checkNum == '0'){
			return (atoi(&checkNum));
		}
		else if (checkNum == '1'){
			return (atoi(&checkNum));
		}
		else{
			cout << "Please Input 1 or 0." << endl;
			cout << "1. Yes" << endl;
			cout << "0. No" << endl;
			cin >> checkNum;
		}
	}
}

/*!
* @brief System::openDireectory().�o�͂����f�B���N�g�����J��(c39)
* @param �Ȃ�
* @return �Ȃ�
*/
void System::openDirectory()
{
	cout << "Opening This Directory." << endl;
	char openDirectoryCommand[NOC]; //�����ŊJ���E�C���h�E�̃p�X
	sprintf_s(openDirectoryCommand, "explorer %s", directoryName); //�f�B���N�g�����J���R�}���h
	system(openDirectoryCommand); //�f�B���N�g�����J���R�}���h�����s

	return;
}

/*!
* @brief System::outputData().�f�[�^���t�@�C���ɏ����o�����\�b�h(c41)
* @param outputDataName, outputData, countDataNum
* @return �Ȃ�
*/
void System::outputAllData(char* outputDataName, outputData* outputData, int countDataNum)
{
	//�t�@�C���|�C���^
	FILE *extractedCoordinate; //!<���o�������W�̋���(c7)
	extractedCoordinate = NULL; //!<�t�@�C���|�C���^�̏�����(c7)

	//(X,Y,Z)�f�[�^�i�[�p�̃t�@�C��
	char outputDataPath[NOC]; //!<�f�[�^�t�@�C���o�͂̍ۂ̃p�X(c37)
	
	sprintf_s(outputDataPath, "%s/%s", directoryName, outputDataName); //(c7)
	fopen_s(&extractedCoordinate, outputDataPath, "w"); //(c7)
	if (outputDataPath == NULL){ //�t�@�C���I�[�v���G���[����(c40)
		cerr << outputDataPath << " is Not Opened.";
		exit(1);
	}

	//�t�@�C���ɏo�͂��鏈��(c42)
	for (int i = 2; i < countDataNum - 15; i++){ //�ŏ��ƍŌ�̂������̃f�[�^���t�@�C���ɏo�͂��Ȃ�(c41)
		fprintf_s(extractedCoordinate, "%f %f %f %f\n", outputData[i].totalTime, outputData[i].x, outputData[i].y, outputData[i].z);
	}
	fclose(extractedCoordinate); //(c8)

	return;
}

/*!
* @brief System::loadInternalCameraParam().�J�����L�����u���[�V�����ɂ���ē���ꂽ�J�����p�����[�^��K�p���郁�\�b�h(c54)
* @param cameraParamFile
* @return �Ȃ�
*/
void System::loadInternalCameraParameter(char* cameraParamFile)
{
	cout << "Loading Camera Parameter" << endl;
	//xml�t�@�C���̓ǂݍ���
	FileStorage fs(cameraParamFile, FileStorage::READ); //�ǂݍ��݃��[�h
	//�����p�����[�^�̓ǂݍ���
	fs["camera_matrix"] >> internalCameraParam; //�����p�����[�^��ǂݍ���
	fs["distortion_coefficients"] >> distortionCoefficients; //�c�݌W����ǂݍ���

	return;
}