/*
 * @file main.cpp
 * @link https://github.com/HShigehara/Masters-Thesis.git 
 * @brief main関数
 * @date 2014.10.15
 * @author H.Shigehara
 */

/* ヘッダファイルのインクルード */
#include "3DPathTrackingUsingtheKINECT.hpp"
#include "KinectMethod.hpp"
#include "ImageProcessingMethod.hpp"
#include "RouteDrawingMethod.hpp"
#include "System.hpp"
#include "LeastSquareMethod.hpp" //(c49)

/* グローバル変数 */
//画像データ
Mat image; //!<RGB画像格納用の変数

char directoryName[NOC]; //!<フォルダ名

//(c6)
int extractedPointOneDim[ALLPIXEL]; //!<抽出された座標の1次元の値
int extractedNum; //(c6)colorExtraction()で何ピクセル抽出されたかカウントする変数

Point3ius extCoordinate[ALLPIXEL]; //抽出された座標(c37)

Vector4 XYZCoordinate[ALLPIXEL]; //3次元に変換された(X,Y,Z)(c49)

//(c25)
Rect trackWindow; //!<追跡ウインドウ
int hsize = 16;
float hranges[] = { 0, 180 }; //!<Hの範囲
const float* phranges = hranges;
bool backprojMode = false; //!<バックプロジェクトモード
bool selectObject = false; //!<オブジェクト選択
int trackObject = 0; //!<追跡するオブジェクト
Point origin; //!<オリジナルの座標
Rect selection; //!<選択
int vmin = 10, vmax = 256, smin = 30; //!<HSVの範囲指定
void onMouse(int event, int x, int y, int flags, void* param); //!<マウス操作

//(c26)
/*int*/bool avgFlag; //!<平均を計算したとき用のフラグ(c30)
/*int*/bool mouseFlag; //!<マウス操作確認用のフラグ(c26)

/*!
 * @brief 関数main()
 * @param なし
 * @return なし
 */
int main()
{
RETRY: //goto文.計測が上手くいかなかったらリセットする用
	//変数の宣言
	int checkNum; //!<プログラム終了時にデータを保存するか確認するための変数(c38)
	int countDataNum; //出力されたデータ数をカウントする(c39)
	outputData outputData[OUTPUTDATA_MAX]; //!<出力するデータを宣言．最大10000個(c41)

	//ファイル名の定義(c39)
	char* outputDataName = "3d.dat"; //計測データが出力されるファイル名(c39)
	char* cenerofgravityDataName = "cog.dat"; //重心座標が出力されるファイル名(c52)

	//画像関係
	Mat mHSVforObjectTracking_img; //!<ヒストグラム作成のためのHSV変換後のデータ保存用
	Mat mHSVforTrim_img; //!<切り取り後の画像格納用(c31)
	Mat mHSVColorExtraction_img; //!<(ce:color extraction).HSVに変換する用の変数

	//オープニング処理用にとっておく
	Mat mGray_img; //!<グレースケール用の変数(c19)
	Mat mBin_img; //!<抽出した後に二値化した画像を保存する変数(c21)
	Mat mOpening_img; //!<オープニングを行った画像から距離を抽出する(c24).オープニング処理を行うには必要*/
	Mat mExtractedBlack_img; //!<オープニング後の二値画像から抽出された黒い座標を格納している変数(c40)

	//インスタンスの生成
	System sys; //!<システム的なメソッドをまとめているクラス
	RouteDrawing routedraw; //!<RouteDrawingクラスのインスタンスを生成
	LeastSquareMethod lsm; //!<最小二乗法を行うクラスのインスタンスを生成(c49)

	//メインの処理
	try{
		sys.startMessage(); //プログラム開始時のメッセージを表示

		Kinect kinect; //Kinectクラスのインスタンスを生成
		ImageProcessing imgproc; //Imageprocessingクラスのインスタンスを生成
		

		//座標関係の変数の定義
		Vector4 realPoint; //!<変換後の世界座標系の値を格納
		Point3ius averageCoordinate; //!<平均座標を格納するクラス内のローカル変数(c38)

		//変数の宣言
		double sumTime; //合計の時間をカウントする変数
		double time; //1フレームあたりの時間(c39)
		double fps; //フレームレート(c39)

		//ウインドウ名とファイル名の定義
		char* mainWindowName = "動画像"; //メインウインドウの名前をつけておく．(c31)
		char* outputVideoName = "video.avi"; //計測中の動画ファイル名(c39)

		//xmlファイルの読み込み
		char* cameraParameterName = "cameraParam.xml"; //カメラキャリブレーションによって得られたファイル名(c54)
		sys.loadInternalCameraParameter(cameraParameterName); //カメラパラメータを読み込む(c54)
		Mat undistort_img;

		//タイマー用変数
		int64 end; //終了時のタイマー

		//フラグ

		//変数の初期化
		countDataNum = 0;
		sumTime = 0.0; //合計の時間をカウントする変数
		time = 0.0; //1フレームの時間
		realPoint.x = 0.0; //3次元座標のx座標
		realPoint.y = 0.0; //3次元座標のy座標
		realPoint.z = 0.0; //3次元座標のx座標
		avgFlag = /*0*/false; //再計測のために平均座標を計算したかチェックするフラグ変数を初期化
		mouseFlag = /*0*/false; //再計測のためにマウスをクリックしたかをチェックするフラグ変数を初期化

		sys.makeDirectory(); //起動時刻をフォルダ名にしてフォルダを作成

		//動画を出力(c40)
		//char outputVideoPath[NOC]; //!<動画出力時のパス(c38)
		//sprintf_s(outputVideoPath, "%s/%s", directoryName, outputVideoName); //(c38)
		//VideoWriter writer(outputVideoPath, /*CV_FOURCC('D','I','B',' ')*/-1/*CV_FOURCC('X','V','I','D')*//*CV_FOURCC('P','M','I','1')*/, 20, /*Size(WIDTH, HEIGHT)*/Size(640, 480), true); //動画に出力.録画が必要なときはコメントアウト(c35)
		//if (!writer.isOpened()){ //オープンエラー処理(c40)
		//	cerr << outputVideoPath << " is Not Opened." << endl;
		//}


		kinect.initialize(); //Kinectの初期化

		namedWindow(mainWindowName, CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO); //動画像用のウインドウを表示(c31)
		namedWindow("歪み補正後", CV_WINDOW_AUTOSIZE || CV_WINDOW_FREERATIO);

		while (1){ //(c3).メインループ．1フレームごとの処理を繰り返し行う．
			//(c25)
			setMouseCallback(mainWindowName, onMouse, 0); //マウスコールバック関数をセット(c31)

			//データの更新を待つ
			DWORD ret = ::WaitForSingleObject(kinect.streamEvent, INFINITE); //フレーム更新をイベントとして待つ
			::ResetEvent(kinect.streamEvent); //イベントが発生したら次のイベントに備えてリセット

			//タイマーを導入．スタート地点(c18)
			double f = 1000.0 / getTickFrequency();
			int64 start = getTickCount(); //スタート

			//Kinect処理・画像処理
			kinect.drawRGBImage(image); //RGBカメラの処理
			
			//歪み補正後の画像に対して処理を行うようにする
			undistort(image, undistort_img, sys.internalCameraParam, sys.distortionCoefficients, Mat()); //歪み補正後の画像で上書き(c54)
			imshow("歪み補正後", undistort_img);
			//image = undistort_img.clone();

			//画像を表示
			imshow(mainWindowName, image);

			//メインの処理(c26)(c30)
			if (mouseFlag == true){ //mouseFlagがtrueであれば=マウスのボタンが上に上がったら
				//writer << image; //動画を保存(c35)
				mHSVforObjectTracking_img = imgproc.convertRGB2HSV(image); //CamShiftで利用するヒストグラムを作成するためにHSVへ変換(c29)
				imgproc.trackingObject(mHSVforObjectTracking_img); //画像を追跡するメソッドへ移行(c26)
				mHSVforTrim_img = imgproc.convertRGB2HSV(imgproc.mTrim_img); //余裕を持って切り取られた画像をHSVに変換する(c33)
				mHSVColorExtraction_img = imgproc.extractColor(mHSVforTrim_img); //余裕を持って切り取った画像から特定の色を抽出する(c33)

				//オープニングを行うなら必要な処理(c30)
				//グレースケール変換(c19)
				mGray_img = imgproc.grayscaleImage(mHSVColorExtraction_img);
				//抽出したピクセルからさらに精度を高めるために膨張・収縮を行う(c19)
				mBin_img = imgproc.binarizationImage(mGray_img); //抽出した画像の二値化
				mOpening_img = imgproc.OpeningImage(mBin_img); //オープニング処理(c19)

				mExtractedBlack_img = imgproc.getCoordinate(mOpening_img);
				//確認用
				namedWindow("計測対象の点", CV_WINDOW_KEEPRATIO);
				//namedWindow("計測対象の点", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
				imshow("計測対象の点", mExtractedBlack_img);

				//最小二乗法を行う(仮)(c49)
				//lsm.getSphereData(kinect.actualExtractedNum); //最小二乗法の処理へ移行(c49)

				//imgproc.showImage("extractedCoordinate", mExtractedBlack_img);

				averageCoordinate = kinect.getAverageCoordinate(image); //Depthカメラの処理(c5)

				imgproc.drawCenterPoint(image, averageCoordinate, mainWindowName); //平均座標を表示するメソッド(c45)

				if (avgFlag == true){ //平均座標が抽出されていたら
					realPoint = kinect.getLocalPosition(averageCoordinate); //1フレームあたりの平均座標の3次元座標を取得する(c10)(c28)
					end = getTickCount();
					time = (end - start) * f;
					sumTime += time;
					fps = 1000.0 / time; //フレームレートを計算

					putText(image, to_string((int)fps) + "fps", Point(WIDTH - 100, HEIGHT - 28), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2, CV_AA); //動画上にフレームレートを出力

					outputData[countDataNum].totalTime = sumTime; //計測されたトータルの時間(c41)
					outputData[countDataNum].x = realPoint.x; //Kinect座標系に変換されたx(c41)
					outputData[countDataNum].y = realPoint.y; //Kinect座標系に変換されたy(c41)
					outputData[countDataNum].z = realPoint.z; //Kinect座標系に変換されたz(c41)

					//抽出されたデータを表示
					//cout << "X => " << realPoint.x << "\tY => " << realPoint.y << "\tZ => " << realPoint.z << endl;
					//routedraw.plot3DRealTime(countDataNum, outputData); //位置をリアルタイムでプロットする(c43)
					countDataNum++; //出力されたデータ数をカウントする(c41)
				}
				else{ //座標が抽出されていないとき．オープニング処理を行っているときは白か黒の二値画像なのでここにくることはない(c41)
					end = getTickCount(); //タイマー終了(c41)
				}
				//imgproc.showTogetherImage(mHSVforTrim_img, mHSVColorExtraction_img); //2つの画像を1つのウインドウに表示．確認用(c36)
			}
			else{ //マウスがクリックされていないときのタイマー終了(c41)
				end = getTickCount(); //タイマー終了
			}

			//終了のためのキー入力チェック兼表示のためのウェイトタイム
			kinect.key = waitKey(1);
			if (kinect.key == 'q'){ //計測終了
				sys.outputAllData(outputDataName, outputData, countDataNum);
				//routedraw.plot3D(outputDataName); //(c4)
				destroyAllWindows();
				break;
			}
			else if (kinect.key == 'r'){ //再計測するときに前のファイルを削除しておく(c31)
				cv::destroyAllWindows(); //OpenCVで作成したウインドウを全て削除する(c35)
				sys.removeDirectory();
				cout << "Data Removed." << endl;
				goto RETRY;
			}
		}
	}

	catch (exception& ex){ //例外処理
		cout << ex.what() << endl;
		destroyAllWindows(); //OpenCVで作成したウインドウを全て削除する(c35)
		//異常終了した時はデータを保存する必要がないので削除
		sys.removeDirectory();
		cout << "Data Removed." << endl;
		return -1;
	}

	//データを保存するかの確認(c27)
	cout << "Save Data?" << endl;
	checkNum = sys.alternatives(); //'1'なら保存，'0'なら削除


	if (checkNum == 0){ //削除するとき(c55)
		sys.removeDirectory(/*checkNum*/); //ディレクトリを削除するかどうか
	}
	else{ //保存するとき
		routedraw.gnuplotScript(/*checkNum, */outputDataName); //後で3D座標をプロットする用のgnuplotスクリプトを作るかどうか
		routedraw.gnuplotScriptCoG(/*checkNum, */cenerofgravityDataName); //後で重心座標をプロットする用のgnuplotスクリプトを作るかどうか(c52)
	}

	sys.endMessage(checkNum);

	return 0;
}