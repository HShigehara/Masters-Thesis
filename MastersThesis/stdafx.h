// stdafx.h : �W���̃V�X�e�� �C���N���[�h �t�@�C���̃C���N���[�h �t�@�C���A�܂���
// �Q�Ɖ񐔂������A�����܂�ύX����Ȃ��A�v���W�F�N�g��p�̃C���N���[�h �t�@�C��
// ���L�q���܂��B
//

#pragma once

#define _CRT_SECURE_NO_WARNINGS //fopen���̂̊֐��̌x�����\���ɂ���

/* �悭���p����w�b�_�[���v���R���p�C������ */
/* �w�b�_�t�@�C���̃C���N���[�h */
#include <iostream> //!<�W�����o�̓X�g���[��
#include <sstream> //!<�X�g�����O�X�g���[��
#include <fstream> //!<�t�@�C�����o�̓X�g���[��
#include <string> //!<������
using namespace std; //!<���O���

#include <stdio.h> //!<C�̕W�����o�̓X�g���[��
#include <stdlib.h> //!<�W�����C�u����
#include <direct.h> //!<�f�B���N�g�����쐬���邽�ߗp����
#include <math.h> //!<���w�֐��p�̃��C�u����(c12)
#include <ctype.h> //!<�����̎�ނ̔���╶���̕ϊ����s��(c25)

/* NuiApi.h�̑O��Windows.h���C���N���[�h���� */
#include <Windows.h>
#include <NuiApi.h>

/* OpenCV�֘A�̃C���N���[�h */
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\video\tracking.hpp> //!<����̃g���b�L���O���s�����߂̃��C�u����(c25)
#include <opencv2\flann\flann.hpp>
using namespace cv; //!<���O���

/* ���x�����O�������s���w�b�_��ǉ�(c21) */
//#include "Labeling.hpp"

/* PCL�֘A�̃C���N���[�h */
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\io\io.h>
#include <pcl\io\pcd_io.h> //.pcd�o�͗p
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\filters\passthrough.h> //Kinect����擾���������̊O��l���폜
#include <pcl\filters\statistical_outlier_removal.h> //�O��l�t�B���^�[�p
#include <pcl\filters\radius_outlier_removal.h> //�O��l�t�B���^�[�p(c60)
#include <pcl\kdtree\kdtree_flann.h> //�X���[�W���O�p
#include <pcl\surface\mls.h> //�X���[�W���O�p
#include <pcl\filters\voxel_grid.h> //�_�E���T���v�����O�p
#include <pcl\ModelCoefficients.h>
#include <pcl\sample_consensus\method_types.h>
#include <pcl\sample_consensus\model_types.h>
#include <pcl\segmentation\sac_segmentation.h>
#include <pcl\PCLPointField.h>

/* ��` */
//�}�N��
#define WIDTH 640 //!<�摜�̕�
#define HEIGHT 480 //!<�摜�̍���
#define ALLPIXEL WIDTH*HEIGHT //!<1�t���[���̑S�s�N�Z����
#define NOC 32 //!<Number of Characters�D(�t�@�C���̖��O��t����Ƃ��̕���������)
#define OUTPUTDATA_MAX 10000 //!<�o�͂���f�[�^�̏��
// TODO: �v���O�����ɕK�v�Ȓǉ��w�b�_�[�������ŎQ�Ƃ��Ă��������B
