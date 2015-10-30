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
	//cloudViewerName = "Point Cloud"; //クラウドビューワーの名前を指定
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

void PointCloudMethod::initializePointCloudViewer(string cloudViewerName)
{
	viewer = new pcl::visualization::CloudViewer(cloudViewerName);
	return;
}