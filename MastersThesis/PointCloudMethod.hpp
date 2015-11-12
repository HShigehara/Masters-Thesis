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

public:
	PointCloudMethod(); //�R���X�g���N�^
	PointCloudMethod(bool flag_RO, bool flag_DS, bool flag_MLS, bool flag_EP); //�R���X�g���N�^(c64)
	~PointCloudMethod(); //�f�X�g���N�^

	void initializePointCloudViewer(string cloudViewerName);
	void flagChecker(); //�t���O�𔻒肷�郁�\�b�h(c64)


	//�O��l����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	
	//�_�E���T���v�����O
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, float leafSizeX, float leafSizeY, float leafSizeZ);

	//�X���[�W���O
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool compute_normals, bool polynomial_fit, double radius);

	//���ʌ��o
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool optimize, double threshold, bool negative);

	//�N���E�h�r���[���[�p
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::visualization::CloudViewer *viewer;


	//�e�_�Q�������s�����ۂ��̃t���O�ϐ�(c64)
	bool flag_removeOutlier;
	bool flag_downsampling;
	bool flag_MLS;
	bool flag_extractPlane;
};

/* �C���N���[�h�K�[�h�̏I�� */
#endif /* __POINTCLOUDMETHOD_HPP__ */