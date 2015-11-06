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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	//cout << "before PassThroughFilter => " << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //�t�B���^�[��p�̃|�C���g�N���E�h
	pcl::PassThrough<pcl::PointXYZRGB> pt;
	pt.setInputCloud(inputPointCloud);
	pt.setFilterFieldName("z");
	pt.setFilterLimits(0, 30);
	pt.filter(*filtered);

	
	//cout << "after PassThroughFilter => " << filtered->size() << endl;

	return filtered;
}

/*
 * @brief ���\�b�hPointCloudMethod::removeOutlier()�D�O��l���������郁�\�b�h(c59)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "before Remove Outlier => " << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //�t�B���^�����O��p�̃|�C���g�N���E�h��錾
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> fl;
	
	fl.setInputCloud(inputPointCloud);
	fl.setMeanK(50);
	fl.setStddevMulThresh(0.1);
	fl.setNegative(false);
	cout << "TEST" << endl;
	fl.filter(*filtered);

	cout << "after Remove Outlier => " << filtered->size() << endl;

	return filtered;
}

/*
* @brief ���\�b�hPointCloudMethod::radiusOutlierRemoval()�D�O��l���������郁�\�b�h(c60)
* @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "before RadiusOutlierRemoval => " << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //�t�B���^�����O��p�̃|�C���g�N���E�h��錾
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;

	ror.setInputCloud(inputPointCloud);
	cout << "before RadiusOutlierRemoval => " << inputPointCloud->size() << endl;

	ror.setRadiusSearch(0.8);
	ror.setMinNeighborsInRadius(2);
	ror.filter(*filtered);

	cout << "after RadiusOutlierRemoval => " << filtered->size() << endl;
	return filtered;
}

/*
* @brief ���\�b�hPointCloudMethod::downSamplingUsingVoxelGridFilter()�D�_�E���T���v�����O�������s�����\�b�h(c59)
* @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, float leafSizeX, float leafSizeY, float leafSizeZ)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //�t�B���^�����O��p�̃|�C���g�N���E�h��錾
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;

	vg.setInputCloud(inputPointCloud);
	//sor.setLeafSize()�Ń_�E���T���v�����O�̒��x��ύX
	vg.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
	//vg.setLeafSize(0.003f, 0.003f, 0.003f); //Default
	//sor.setLeafSize(0.03f, 0.03f, 0.03f); //���Ȃ�
	vg.filter(*filtered);

	//�|�C���g�N���E�h����������ێ��ł��Ă��邩�T�C�Y���m�F
	//cout << "inputSIZE => " << inputPointCloud->size() << endl; //���͂����|�C���g�N���E�h�̃T�C�Y
	cout << "after Voxel Grid Filter => " << filtered->size() << endl; //�o�͂����|�C���g�N���E�h�̃T�C�Y

	return filtered;
}

/*
* @brief ���\�b�hPointCloudMethod::smoothingUsingMovingLeastSquare()�D�X���[�W���O���s�����\�b�h(c60)
* @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool compute_normals, bool polynomial_fit, double radius)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //�t�B���^�����O������p�̃|�C���g�N���E�h

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>()); //KdTree�̍쐬

	//pcl::PointCloud<pcl::PointXYZ> mls_points; //�o�͂���_�Q�̕ۑ��ꏊ
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls; //�X���[�W���O����

	mls.setComputeNormals(compute_normals); //�@���̌v�Z
	//�e�p�����[�^�̐ݒ�
	mls.setInputCloud(inputPointCloud);
	mls.setPolynomialFit(polynomial_fit);
	mls.setSearchMethod(tree);
	//mls.setSearchRadius(0.003); //Default
	mls.setSearchRadius(radius);
	//mls.process(mls_points); //�o��
	mls.process(*filtered); // �o��

	//cout << "after MLS => " << mls_points.size() << endl;
	cout << "after MLS => " << filtered->size() << endl;
	//cout << "after MLS(Point Cloud Data) => " << filtered << endl;

	//return mls_points;
	return filtered;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	//�Z�O�����e�[�V�����I�u�W�F�N�g�̐���
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	
	//�I�v�V����
	seg.setOptimizeCoefficients(true);

	//Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(inputPointCloud->makeShared());
	seg.segment(*inliers, *coefficients);



	//if (inliers->indices.size() == 0){
	//	PCL_ERROR("Cloud not estimate a planar model for the given dataset.");
	//	exit(1);
	//}
	//cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << endl;
	//cerr << "Model inliers: " << inliers->indices.size() << endl;
	for (size_t i = 0; i < inliers->indices.size(); ++i){
		//cerr << inliers->indices[i] << " "
		//	<< inputPointCloud->points[inliers->indices[i]].x << " "
		//	<< inputPointCloud->points[inliers->indices[i]].y << " "
		//	<< inputPointCloud->points[inliers->indices[i]].z << " "
		//	<< endl;

		inputPointCloud->points[inliers->indices[i]].r = 255;
		inputPointCloud->points[inliers->indices[i]].g = 0;
		inputPointCloud->points[inliers->indices[i]].b = 0;

	}

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(inputPointCloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*filtered);

	//filtered = inputPointCloud->makeShared();
	//cout << "TEST => " << filtered->size() << endl;
	return filtered;
}