/*
* @file System.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief kinect処理のクラスのヘッダ
* @date 2014.12.19
* @author H.Shigehara
*/

/* */
#include "3DPathTrackingUsingtheKINECT.hpp"
#include "System.hpp"

/*!
* @brief System::System().コンストラクタ
* @param なし
* @return なし
*/
System::System()
{
	//コンストラクタの処理はなし
}

/*!
* @brief System::~System().デストラクタ
* @param なし
* @return なし
*/
System::~System()
{
	//デストラクタの処理はなし
}

/*!
* @brief System::startMessage().プログラム起動時のメッセージ
* @param なし
* @return なし
*/
void System::startMessage()
{
	cout << "==================================================================" << endl;
	cout << " Starting the Program...." << endl;
	cout << " Please Enclose the Object You Want to Track." << endl;
	cout << " If You Enter a 'q' Key, the Program Terminates. (On the Window.)" << endl;
	cout << " If You Enter a 'r' Key, the Program Restart. (On the Window.)" << endl;
	//cout << " To Initialize Tracking, Re-Select the Object with Mouse." << endl;
	cout << "==================================================================\n" << endl;

	return;
}

/*!
* @brief System::endMessage().プログラム終了時のメッセージ
* @param なし
* @return なし
*/
void System::endMessage(int cNum)
{
	cout << "==================================================================" << endl;
	cout << "Closing the Program...." << endl;
	if (cNum == 1){
		cout << "Data Has Been Output to \"" << directoryName << "\"." << endl;
		openDirectory();
	}
	cout << "==================================================================" << endl;
	
	return;
}


/*!
* @brief System::makeDirectory().ディレクトリを作成
* @param なし
* @return なし
*/
void System::makeDirectory()
{
	//フォルダ名を区別するために時刻を取得しておく(c4)
	SYSTEMTIME st;

	GetLocalTime(&st);
	sprintf_s(directoryName, "[%4d%02d%02d]%02d_%02d_%02d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
	_mkdir(directoryName);

	return;
}

/*!
* @brief System::removeDirectory().ディレクトリを削除(c21)
* @param なし 
* @return なし
*/
void System::removeDirectory()
{
	char rmdirCommand[NOC]; //ディレクトリを削除するコマンド(c21).変数名を変更&このメソッドのみで有効な変数(c30)
	//ディレクトリを削除する
	sprintf_s(rmdirCommand, "rmdir /s /q %s", directoryName);
	system(rmdirCommand);
	cout << "Not Save.\n" << endl;

	return;
}

/*!
* @brief System::alternatives().Yes/Noの2択のチェック(c21)
* @param なし
* @return checkNum
*/
int System::alternatives()
{
	char checkNum; //!<0か1をチェックするための変数(c27).このメソッドのみで有効な変数(c30)

	cout << "1. Yes" << endl;
	cout << "0. No" << endl;
	cin >> checkNum;
	cout << "\n";

	while (1)
	{
		if (checkNum == '0'){
			return (atoi(&checkNum));
		}
		else if (checkNum == '1'){
			return (atoi(&checkNum));
		}
		else{
			cout << "Please Input 1 or 0." << endl;
			cout << "1. Yes" << endl;
			cout << "0. No" << endl;
			cin >> checkNum;
		}
	}
}

/*!
* @brief System::openDireectory().出力したディレクトリを開く(c39)
* @param なし
* @return なし
*/
void System::openDirectory()
{
	cout << "Opening This Directory." << endl;
	char openDirectoryCommand[NOC]; //自動で開くウインドウのパス
	sprintf_s(openDirectoryCommand, "explorer %s", directoryName); //ディレクトリを開くコマンド
	system(openDirectoryCommand); //ディレクトリを開くコマンドを実行

	return;
}

/*!
* @brief System::outputData().データをファイルに書き出すメソッド(c41)
* @param outputDataName, outputData, countDataNum
* @return なし
*/
void System::outputAllData(char* outputDataName, outputData* outputData, int countDataNum)
{
	//ファイルポインタ
	FILE *extractedCoordinate; //!<抽出した座標の距離(c7)
	extractedCoordinate = NULL; //!<ファイルポインタの初期化(c7)

	//(X,Y,Z)データ格納用のファイル
	char outputDataPath[NOC]; //!<データファイル出力の際のパス(c37)
	
	sprintf_s(outputDataPath, "%s/%s", directoryName, outputDataName); //(c7)
	fopen_s(&extractedCoordinate, outputDataPath, "w"); //(c7)
	if (outputDataPath == NULL){ //ファイルオープンエラー処理(c40)
		cerr << outputDataPath << " is Not Opened.";
		exit(1);
	}

	//ファイルに出力する処理(c42)
	for (int i = 2; i < countDataNum - 15; i++){ //最初と最後のいくつかのデータをファイルに出力しない(c41)
		fprintf_s(extractedCoordinate, "%f %f %f %f\n", outputData[i].totalTime, outputData[i].x, outputData[i].y, outputData[i].z);
	}
	fclose(extractedCoordinate); //(c8)

	return;
}

/*!
* @brief System::loadInternalCameraParam().カメラキャリブレーションによって得られたカメラパラメータを適用するメソッド(c54)
* @param cameraParamFile
* @return なし
*/
void System::loadInternalCameraParameter(char* cameraParamFile)
{
	cout << "Loading Camera Parameter" << endl;
	//xmlファイルの読み込み
	FileStorage fs(cameraParamFile, FileStorage::READ); //読み込みモード
	//内部パラメータの読み込み
	fs["camera_matrix"] >> internalCameraParam; //内部パラメータを読み込む
	fs["distortion_coefficients"] >> distortionCoefficients; //歪み係数を読み込む

	return;
}