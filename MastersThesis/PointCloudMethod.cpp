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


/*
* @brief ���\�b�hPointCloudMethod::initializePointCloudViewer().�|�C���g�N���E�h�r���[�A�[�����������郁�\�b�h(c57)
* @param string cloudViewerName
* @return �Ȃ�
*/
void PointCloudMethod::initializePointCloudViewer(string cloudViewerName)
{
	viewer = new pcl::visualization::CloudViewer(cloudViewerName);
	return;
}

/*
* @brief ���\�b�hPointCloudMethod::downSamplingUsingVoxelGridFilter()�D�_�E���T���v�����O�������s�����\�b�h(c59)
* @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud
*/
pcl::PointCloud<pcl::PointXYZ> PointCloudMethod::downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>()); //�t�B���^�����O��p�̃|�C���g�N���E�h��錾
	pcl::VoxelGrid<pcl::PointXYZ> sor;

	sor.setInputCloud(inputPointCloud);
	//sor.setLeafSize()�Ń_�E���T���v�����O�̒��x��ύX
	sor.setLeafSize(0.003f, 0.003f, 0.003f); //Default
	//sor.setLeafSize(0.03f, 0.03f, 0.03f); //���Ȃ�
	sor.filter(*filtered);

	//�|�C���g�N���E�h����������ێ��ł��Ă��邩�T�C�Y���m�F
	//cout << "inputSIZE => " << inputPointCloud->size() << endl; //���͂����|�C���g�N���E�h�̃T�C�Y
	//cout << "outputSIZE => " << filtered->size() << endl; //�o�͂����|�C���g�N���E�h�̃T�C�Y

	return *filtered;
}