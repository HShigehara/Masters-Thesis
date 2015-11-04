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
* @return pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointCloud
*/
pcl::PointCloud<pcl::PointXYZ> PointCloudMethod::downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud)
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
	//cout << "outputSIZE => " << filtered->size() << endl; //出力されるポイントクラウドのサイズ

	return *filtered;
}