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
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudMethod::downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud)
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
	cout << "after Voxel Grid Filter => " << filtered->size() << endl; //�o�͂����|�C���g�N���E�h�̃T�C�Y

	return filtered;
}

/*
 * @brief ���\�b�hPointCloudMethod::removeOutlier()�D�O��l���������郁�\�b�h(c59)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudMethod::removeOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>()); //�t�B���^�����O��p�̃|�C���g�N���E�h��錾
	
	
	//cout << "after Remove Outlier => " << inputPointCloud->size() << endl;

	return filtered;
}

/*
* @brief ���\�b�hPointCloudMethod::smoothingUsingMovingLeastSquare()�D�X���[�W���O���s�����\�b�h(c60)
* @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudMethod::smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>()); //�t�B���^�����O������p�̃|�C���g�N���E�h

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>()); //KdTree�̍쐬

	//pcl::PointCloud<pcl::PointXYZ> mls_points; //�o�͂���_�Q�̕ۑ��ꏊ
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; //�X���[�W���O����

	mls.setComputeNormals(true); //�@���̌v�Z
	//�e�p�����[�^�̐ݒ�
	mls.setInputCloud(inputPointCloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	//mls.setSearchRadius(0.003); //Default
	mls.setSearchRadius(0.0025);
	//mls.process(mls_points); //�o��
	mls.process(*filtered); // �o��

	//cout << "after MLS => " << mls_points.size() << endl;
	cout << "after MLS => " << filtered->size() << endl;
	//cout << "after MLS(Point Cloud Data) => " << filtered << endl;

	//return mls_points;
	return filtered;
}
