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

/*
* @brief メソッドPointCloudMethod::downSamplingUsingVoxelGridFilter()．ダウンサンプリング処理を行うメソッド(c59)
* @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudMethod::downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>()); //フィルタリング後用のポイントクラウドを宣言
	pcl::VoxelGrid<pcl::PointXYZ> sor;

	sor.setInputCloud(inputPointCloud);
	//sor.setLeafSize()でダウンサンプリングの程度を変更
	sor.setLeafSize(0.003f, 0.003f, 0.003f); //Default
	//sor.setLeafSize(0.03f, 0.03f, 0.03f); //少ない
	sor.filter(*filtered);

	//ポイントクラウドをしっかり保持できているかサイズを確認
	//cout << "inputSIZE => " << inputPointCloud->size() << endl; //入力したポイントクラウドのサイズ
	cout << "after Voxel Grid Filter => " << filtered->size() << endl; //出力されるポイントクラウドのサイズ

	return filtered;
}

/*
 * @brief メソッドPointCloudMethod::removeOutlier()．外れ値を除去するメソッド(c59)
 * @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr filtered
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudMethod::removeOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>()); //フィルタリング後用のポイントクラウドを宣言
	
	
	//cout << "after Remove Outlier => " << inputPointCloud->size() << endl;

	return filtered;
}

/*
* @brief メソッドPointCloudMethod::smoothingUsingMovingLeastSquare()．スムージングを行うメソッド(c60)
* @param pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudMethod::smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>()); //フィルタリング処理後用のポイントクラウド

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>()); //KdTreeの作成

	//pcl::PointCloud<pcl::PointXYZ> mls_points; //出力する点群の保存場所
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; //スムージング処理

	mls.setComputeNormals(true); //法線の計算
	//各パラメータの設定
	mls.setInputCloud(inputPointCloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	//mls.setSearchRadius(0.003); //Default
	mls.setSearchRadius(0.0025);
	//mls.process(mls_points); //出力
	mls.process(*filtered); // 出力

	//cout << "after MLS => " << mls_points.size() << endl;
	cout << "after MLS => " << filtered->size() << endl;
	//cout << "after MLS(Point Cloud Data) => " << filtered << endl;

	//return mls_points;
	return filtered;
}
