/*
* @file PointCloudMethod.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief PCL�֘A�̏������s���N���X�̃w�b�_
* @date 2015.10.30
* @author H.Shigehara
*/

/* �C���N���[�h�K�[�h */
#ifndef __POINTCLOUDMETHOD_HPP__
#define __POINTCLOUDMETHOD_HPP__

/* �C���N���[�h */
#include "3DPathTrackingUsingtheKINECT.hpp"

/*!
* @class PointCloudMethod
* @brief �_�Q�������s���N���X
*/
class PointCloudMethod{
private:
	string cloudViewerName; //�N���E�h�r���[���[�̃E�C���h�E��


public:
	PointCloudMethod(); //�R���X�g���N�^
	~PointCloudMethod(); //�f�X�g���N�^

	void initializePointCloudViewer(string cloudViewerName);

	//�N���E�h�r���[���[�p
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::visualization::CloudViewer *viewer;


};

/* �C���N���[�h�K�[�h�̏I�� */
#endif /* __POINTCLOUDMETHOD_HPP__ */