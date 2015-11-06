/*
* @file PointCloudMethod.cpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief PCL関連の処理を行うクラス
* @date 2015.10.30
* @author H.Shigehara
*/

/* ヘッダファイルのインクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"
#include "PointCloudMethod.hpp"

/*!
* @brief メソッドPointCloudMethod::PointCloudMethod().コンストラクタ
* @param なし
* @return なし
*/
PointCloudMethod::PointCloudMethod()
{
	//コンストラクタ
}

/*!
* @brief メソッドPointCloudMethod::~PointCloudMethod().デストラクタ
* @param なし
* @return なし
*/
PointCloudMethod::~PointCloudMethod()
{
	//デストラクタ
}


/*
* @brief メソッドPointCloudMethod::initializePointCloudViewer().ポイントクラウドビューアーを初期化するメソッド(c57)
* @param string cloudViewerName
* @return なし
*/
void PointCloudMethod::initializePointCloudViewer(string cloudViewerName)
{
	viewer = new pcl::visualization::CloudViewer(cloudViewerName);
	return;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	//cout << "before PassThroughFilter => " << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルター後用のポイントクラウド
	pcl::PassThrough<pcl::PointXYZRGB> pt;
	pt.setInputCloud(inputPointCloud);
	pt.setFilterFieldName("z");
	pt.setFilterLimits(0, 30);
	pt.filter(*filtered);

	
	//cout << "after PassThroughFilter => " << filtered->size() << endl;

	return filtered;
}

/*
 * @brief メソッドPointCloudMethod::removeOutlier()．外れ値を除去するメソッド(c59)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "before Remove Outlier => " << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング後用のポイントクラウドを宣言
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
* @brief メソッドPointCloudMethod::radiusOutlierRemoval()．外れ値を除去するメソッド(c60)
* @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud)
{
	cout << "before RadiusOutlierRemoval => " << inputPointCloud->size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング後用のポイントクラウドを宣言
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
* @brief メソッドPointCloudMethod::downSamplingUsingVoxelGridFilter()．ダウンサンプリング処理を行うメソッド(c59)
* @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, float leafSizeX, float leafSizeY, float leafSizeZ)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング後用のポイントクラウドを宣言
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;

	vg.setInputCloud(inputPointCloud);
	//sor.setLeafSize()でダウンサンプリングの程度を変更
	vg.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
	//vg.setLeafSize(0.003f, 0.003f, 0.003f); //Default
	//sor.setLeafSize(0.03f, 0.03f, 0.03f); //少ない
	vg.filter(*filtered);

	//ポイントクラウドをしっかり保持できているかサイズを確認
	//cout << "inputSIZE => " << inputPointCloud->size() << endl; //入力したポイントクラウドのサイズ
	cout << "after Voxel Grid Filter => " << filtered->size() << endl; //出力されるポイントクラウドのサイズ

	return filtered;
}

/*
* @brief メソッドPointCloudMethod::smoothingUsingMovingLeastSquare()．スムージングを行うメソッド(c60)
* @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMethod::smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool compute_normals, bool polynomial_fit, double radius)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>()); //フィルタリング処理後用のポイントクラウド

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>()); //KdTreeの作成

	//pcl::PointCloud<pcl::PointXYZ> mls_points; //出力する点群の保存場所
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls; //スムージング処理

	mls.setComputeNormals(compute_normals); //法線の計算
	//各パラメータの設定
	mls.setInputCloud(inputPointCloud);
	mls.setPolynomialFit(polynomial_fit);
	mls.setSearchMethod(tree);
	//mls.setSearchRadius(0.003); //Default
	mls.setSearchRadius(radius);
	//mls.process(mls_points); //出力
	mls.process(*filtered); // 出力

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

	//セグメンテーションオブジェクトの生成
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	
	//オプション
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