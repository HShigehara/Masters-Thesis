/*
* @file PointCloudMethod.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief PCL関連の処理を行うクラスのヘッダ
* @date 2015.10.30
* @author H.Shigehara
*/

/* インクルードガード */
#ifndef __POINTCLOUDMETHOD_HPP__
#define __POINTCLOUDMETHOD_HPP__

/* インクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"

/*!
* @class PointCloudMethod
* @brief 点群処理を行うクラス
*/
class PointCloudMethod{
private:
	//string cloudViewerName; //クラウドビューワーのウインドウ名


public:
	PointCloudMethod(); //コンストラクタ
	~PointCloudMethod(); //デストラクタ

	void initializePointCloudViewer(string cloudViewerName);
	pcl::PointCloud<pcl::PointXYZ>::Ptr downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPointCloud);

	//クラウドビューワー用
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::visualization::CloudViewer *viewer;


};

/* インクルードガードの終了 */
#endif /* __POINTCLOUDMETHOD_HPP__ */