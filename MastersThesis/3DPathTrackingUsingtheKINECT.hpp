/*
 * @file 3DPathTrackingUsingtheKINECT.h
 * @link https://github.com/HShigehara/Masters-Thesis.git
 * @brief �����f�[�^���擾���摜�Ƃ��ĕ\������v���O�����̃w�b�_�t�@�C��
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* �C���N���[�h�K�[�h */
#ifndef __3DPATHTRACKINGUSINGTHEKINECT_HPP__
#define __3DPATHTRACKINGUSINGTHEKINECT_HPP__

//�悭�g���w�b�_��stdafx.h��
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
extern Mat depth_image; //!<�����摜�i�[�p�̕ϐ�(c58)

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