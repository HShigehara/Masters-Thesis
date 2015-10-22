/*
* @file System.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief kinect処理のクラスのヘッダ
* @date 2014.12.19
* @author H.Shigehara
*/


/* インクルードガード */
#ifndef __SYSTEM_HPP__
#define __SYSTEM_HPP__

/* ヘッダファイルのインクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"

class System
{
private:
	char fullPathName[NOC]; //!<フルパスを取得(c38)

public:
	System();
	~System();
	void startMessage(); //!<プログラム開始時のメッセージを表示(c26)
	void endMessage(int cNum); //!<プログラム終了時のメッセージを表示(c38)
	void makeDirectory(); //ディレクトリの作成
	void removeDirectory(/*int cNum*/); //!<取得したデータが不要だった場合ディレクトリを削除する
	int alternatives(); //!<数字の入力をチェックする
	void openDirectory(); //!<ディレクトリを開く(c38)
	void outputAllData(char* outputDataName, outputData* outputData, int countDataNum); //!<データをファイルに書き出すメソッド(c41)
	void loadInternalCameraParameter(char* cameraParamFile); //!<カメラキャリブレーションによって得られたパラメータを適用する(c54)

	Mat internalCameraParam; //!<カメラキャリブレーションによって得られた内部パラメータ行列(c54)
	Mat distortionCoefficients; //!<カメラキャリブレーションによって得られた歪み係数行列(c54)
};

#endif /* __SYSTEM_HPP__ */