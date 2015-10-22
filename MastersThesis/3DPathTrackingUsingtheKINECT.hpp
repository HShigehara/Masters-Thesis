/*
 * @file 3DPathTrackingUsingtheKINECT.h
 * @link https://HShigehara@bitbucket.org/HShigehara/3dpathtrackingusingthekinect.git 
 * @brief 距離データを取得し画像として表示するプログラムのヘッダファイル
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* インクルードガード */
#ifndef __3DPATHTRACKINGUSINGTHEKINECT_HPP__
#define __3DPATHTRACKINGUSINGTHEKINECT_HPP__

/* ヘッダファイルのインクルード */
#include<iostream> //!<標準入出力ストリーム
#include<sstream> //!<ストリングストリーム
#include<fstream> //!<ファイル入出力ストリーム
using namespace std; //!<名前空間

#include<stdio.h> //!<Cの標準入出力ストリーム
#include<stdlib.h> //!<標準ライブラリ
#include<direct.h> //!<ディレクトリを作成するため用いる
#include<math.h> //!<数学関数用のライブラリ(c12)
#include<ctype.h> //!<文字の種類の判定や文字の変換を行う(c25)

/* NuiApi.hの前にWindows.hをインクルードする */
#include<Windows.h>
#include<NuiApi.h>

/* OpenCV関連のインクルード */
#include<opencv2\opencv.hpp>
#include<opencv2\core\core.hpp>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<opencv2\video\tracking.hpp> //!<動画のトラッキングを行うためのライブラリ(c25)
using namespace cv; //!<名前空間

/* ラベリング処理を行うヘッダを追加(c21) */
//#include "Labeling.hpp"

/* 定義 */
//マクロ
#define WIDTH 640 //!<画像の幅
#define HEIGHT 480 //!<画像の高さ
#define ALLPIXEL WIDTH*HEIGHT //!<1フレームの全ピクセル数
#define NOC 32 //!<Number of Characters．(ファイルの名前を付けるときの文字数制限)
#define OUTPUTDATA_MAX 10000 //!<出力するデータの上限

//構造体
typedef struct Point3{ //抽出された座標を保存する構造体(c37)
	int x;
	int y;
	USHORT z;
}Point3ius;

typedef struct outputData{ //ファイルに出力するデータ群(c41)
	double totalTime;
	float x;
	float y;
	float z;
}outputData;

/* グローバル変数 */
//画像関係
extern Mat image; //!<RGB画像格納用の変数

//ファイル名関係
extern char directoryName[NOC]; //!<フォルダ名

//(c6)
extern int extractedPointOneDim[ALLPIXEL]; //!<抽出された座標の1次元の値
extern int extractedNum; //!<(c6)colorExtraction()で何ピクセル抽出されたかカウントする変数

//(c8)
extern Point3ius extCoordinate[ALLPIXEL]; //!<抽出された座標を保存する変数(c37)

//(c49)
extern Vector4 XYZCoordinate[ALLPIXEL]; //!<3次元に変換された(X,Y,Z)のデータ(c49)

/* CamShift用変数(c25) */
extern bool backprojMode; //!<バックプロジェクトモード
extern bool selectObject; //!<オブジェクト選択
extern int trackObject; //!<追跡するオブジェクト
extern Point origin; //!<オリジナルの座標
extern Rect selection; //!<選択
extern int vmin, vmax, smin; //!<HSVの範囲指定
extern void onMouse(int event, int x, int y, int, void*); //!<マウス操作
/* (c25) */
extern Rect trackWindow; //!<追跡ウインドウ
extern int hsize;
extern float hranges[];//!<Hの範囲
extern const float* phranges;

/* (c26) */
extern /*int*/bool avgFlag; //!<平均を計算したとき用のフラグ(c30)
extern /*int*/bool mouseFlag; //!<マウス操作確認用のフラグ(c26)

/* インクルードガードの終了 */
#endif /* __3DPATHTRACKINGUSINGTHEKINECT_HPP__ */