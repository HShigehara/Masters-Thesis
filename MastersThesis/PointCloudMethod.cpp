/*
* @file PointCloudMethod.cpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief PCL�֘A�̏������s���N���X
* @date 2015.10.30
* @author H.Shigehara
*/

/* �w�b�_�t�@�C���̃C���N���[�h */
#include "3DPathTrackingUsingtheKINECT.hpp"
#include "PointCloudMethod.hpp"

/*!
* @brief ���\�b�hPointCloudMethod::PointCloudMethod().�R���X�g���N�^
* @param �Ȃ�
* @return �Ȃ�
*/
PointCloudMethod::PointCloudMethod()
{
	//�R���X�g���N�^
	//cloudViewerName = "Point Cloud"; //�N���E�h�r���[���[�̖��O���w��
}

/*!
* @brief ���\�b�hPointCloudMethod::~PointCloudMethod().�f�X�g���N�^
* @param �Ȃ�
* @return �Ȃ�
*/
PointCloudMethod::~PointCloudMethod()
{
	//�f�X�g���N�^
}

void PointCloudMethod::initializePointCloudViewer(string cloudViewerName)
{
	viewer = new pcl::visualization::CloudViewer(cloudViewerName);
	return;
}