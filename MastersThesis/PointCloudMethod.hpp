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

public:
	PointCloudMethod(); //コンストラクタ
	PointCloudMethod(bool flag_RO, bool flag_DS, bool flag_MLS, bool flag_EP); //コンストラクタ(c64)
	~PointCloudMethod(); //デストラクタ

	void initializePointCloudViewer(string cloudViewerName);
	void flagChecker(); //フラグを判定するメソッド(c64)


	//外れ値除去
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud);
	
	//ダウンサンプリング
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingUsingVoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, float leafSizeX, float leafSizeY, float leafSizeZ);

	//スムージング
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothingUsingMovingLeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool compute_normals, bool polynomial_fit, double radius);

	//平面検出
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputPointCloud, bool optimize, double threshold, bool negative);

	//クラウドビューワー用
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::visualization::CloudViewer *viewer;

	//各点群処理を行うか否かのフラグ変数(c64)
	bool FlagRemoveOutlier;
	bool FlagDownsampling;
	bool FlagMLS;
	bool FlagExtractPlane;
};

/* インクルードガードの終了 */
#endif /* __POINTCLOUDMETHOD_HPP__ */