/*
 * @file 3DPathTrackingUsingtheKINECT.h
 * @link https://HShigehara@bitbucket.org/HShigehara/3dpathtrackingusingthekinect.git 
 * @brief �����f�[�^���擾���摜�Ƃ��ĕ\������v���O�����̃w�b�_�t�@�C��
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* �C���N���[�h�K�[�h */
#ifndef __3DPATHTRACKINGUSINGTHEKINECT_HPP__
#define __3DPATHTRACKINGUSINGTHEKINECT_HPP__

/* �w�b�_�t�@�C���̃C���N���[�h */
#include<iostream> //!<�W�����o�̓X�g���[��
#include<sstream> //!<�X�g�����O�X�g���[��
#include<fstream> //!<�t�@�C�����o�̓X�g���[��
using namespace std; //!<���O���

#include<stdio.h> //!<C�̕W�����o�̓X�g���[��
#include<stdlib.h> //!<�W�����C�u����
#include<direct.h> //!<�f�B���N�g�����쐬���邽�ߗp����
#include<math.h> //!<���w�֐��p�̃��C�u����(c12)
#include<ctype.h> //!<�����̎�ނ̔���╶���̕ϊ����s��(c25)

/* NuiApi.h�̑O��Windows.h���C���N���[�h���� */
#include<Windows.h>
#include<NuiApi.h>

/* OpenCV�֘A�̃C���N���[�h */
#include<opencv2\opencv.hpp>
#include<opencv2\core\core.hpp>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<opencv2\video\tracking.hpp> //!<����̃g���b�L���O���s�����߂̃��C�u����(c25)
using namespace cv; //!<���O���

/* ���x�����O�������s���w�b�_��ǉ�(c21) */
//#include "Labeling.hpp"

/* ��` */
//�}�N��
#define WIDTH 640 //!<�摜�̕�
#define HEIGHT 480 //!<�摜�̍���
#define ALLPIXEL WIDTH*HEIGHT //!<1�t���[���̑S�s�N�Z����
#define NOC 32 //!<Number of Characters�D(�t�@�C���̖��O��t����Ƃ��̕���������)
#define OUTPUTDATA_MAX 10000 //!<�o�͂���f�[�^�̏��

//�\����
typedef struct Point3{ //���o���ꂽ���W��ۑ�����\����(c37)
	int x;
	int y;
	USHORT z;
}Point3ius;

typedef struct outputData{ //�t�@�C���ɏo�͂���f�[�^�Q(c41)
	double totalTime;
	float x;
	float y;
	float z;
}outputData;

/* �O���[�o���ϐ� */
//�摜�֌W
extern Mat image; //!<RGB�摜�i�[�p�̕ϐ�

//�t�@�C�����֌W
extern char directoryName[NOC]; //!<�t�H���_��

//(c6)
extern int extractedPointOneDim[ALLPIXEL]; //!<���o���ꂽ���W��1�����̒l
extern int extractedNum; //!<(c6)colorExtraction()�ŉ��s�N�Z�����o���ꂽ���J�E���g����ϐ�

//(c8)
extern Point3ius extCoordinate[ALLPIXEL]; //!<���o���ꂽ���W��ۑ�����ϐ�(c37)

//(c49)
extern Vector4 XYZCoordinate[ALLPIXEL]; //!<3�����ɕϊ����ꂽ(X,Y,Z)�̃f�[�^(c49)

/* CamShift�p�ϐ�(c25) */
extern bool backprojMode; //!<�o�b�N�v���W�F�N�g���[�h
extern bool selectObject; //!<�I�u�W�F�N�g�I��
extern int trackObject; //!<�ǐՂ���I�u�W�F�N�g
extern Point origin; //!<�I���W�i���̍��W
extern Rect selection; //!<�I��
extern int vmin, vmax, smin; //!<HSV�͈͎̔w��
extern void onMouse(int event, int x, int y, int, void*); //!<�}�E�X����
/* (c25) */
extern Rect trackWindow; //!<�ǐՃE�C���h�E
extern int hsize;
extern float hranges[];//!<H�͈̔�
extern const float* phranges;

/* (c26) */
extern /*int*/bool avgFlag; //!<���ς��v�Z�����Ƃ��p�̃t���O(c30)
extern /*int*/bool mouseFlag; //!<�}�E�X����m�F�p�̃t���O(c26)

/* �C���N���[�h�K�[�h�̏I�� */
#endif /* __3DPATHTRACKINGUSINGTHEKINECT_HPP__ */