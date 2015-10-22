/*
* @file System.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief kinect�����̃N���X�̃w�b�_
* @date 2014.12.19
* @author H.Shigehara
*/


/* �C���N���[�h�K�[�h */
#ifndef __SYSTEM_HPP__
#define __SYSTEM_HPP__

/* �w�b�_�t�@�C���̃C���N���[�h */
#include "3DPathTrackingUsingtheKINECT.hpp"

class System
{
private:
	char fullPathName[NOC]; //!<�t���p�X���擾(c38)

public:
	System();
	~System();
	void startMessage(); //!<�v���O�����J�n���̃��b�Z�[�W��\��(c26)
	void endMessage(int cNum); //!<�v���O�����I�����̃��b�Z�[�W��\��(c38)
	void makeDirectory(); //�f�B���N�g���̍쐬
	void removeDirectory(/*int cNum*/); //!<�擾�����f�[�^���s�v�������ꍇ�f�B���N�g�����폜����
	int alternatives(); //!<�����̓��͂��`�F�b�N����
	void openDirectory(); //!<�f�B���N�g�����J��(c38)
	void outputAllData(char* outputDataName, outputData* outputData, int countDataNum); //!<�f�[�^���t�@�C���ɏ����o�����\�b�h(c41)
	void loadInternalCameraParameter(char* cameraParamFile); //!<�J�����L�����u���[�V�����ɂ���ē���ꂽ�p�����[�^��K�p����(c54)

	Mat internalCameraParam; //!<�J�����L�����u���[�V�����ɂ���ē���ꂽ�����p�����[�^�s��(c54)
	Mat distortionCoefficients; //!<�J�����L�����u���[�V�����ɂ���ē���ꂽ�c�݌W���s��(c54)
};

#endif /* __SYSTEM_HPP__ */