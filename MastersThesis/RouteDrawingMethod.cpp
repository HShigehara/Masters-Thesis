/*
 * @file RouteDrawingMethod.cpp
 * @link https://github.com/HShigehara/3DPathTrackingUsingtheKINECT.git 
 * @brief �o�H���v���b�g���邽�߂̃��\�b�h�Q
 * @date 2014.10.24
 * @author H.Shigehara
 */

/* �w�b�_�t�@�C���̃C���N���[�h */
//#include "stdafx.h"
#include "3DPathTrackingUsingtheKINECT.hpp" //�w�b�_�t�@�C���̃C���N���[�h
#include "RouteDrawingMethod.hpp"

/*!
* @brief ���\�b�hRouteDrawing::RouteDrawing().�R���X�g���N�^
* @param �Ȃ�
* @return �Ȃ�
*/
RouteDrawing::RouteDrawing()
{
	if ((gnuplot = _popen("gnuplot", "w")) == NULL){
		cout << "gnuplot���J���܂���D\"gnuplot/binary\"�փp�X���ʂ��Ă��邩�m�F���ĉ�����" << endl; //gnuplot/binary/gnuplot.exe���J��.��wgnuplot.exe�͋N�����邪�������i�܂�,gnuplot���N�������܂܂ɂȂ�
	}

	//(c43)
	if ((gpr = _popen("gnuplot", "w")) == NULL){
		cout << "gnuplot���J���܂���D\"gnuplot/binary\"�փp�X���ʂ��Ă��邩�m�F���ĉ�����" << endl; //gnuplot/binary/gnuplot.exe���J��.��wgnuplot.exe�͋N�����邪�������i�܂�,gnuplot���N�������܂܂ɂȂ�
	}
}

/*!
* @brief ���\�b�hRouteDrawing::RouteDrawing().�f�X�g���N�^
* @param �Ȃ�
* @return �Ȃ�
*/
RouteDrawing::~RouteDrawing()
{
	_pclose(gnuplot);

	//(c43)
	//fprintf_s(gpr, "unset multiplot\n");
	fprintf_s(gpr, "quit\n");
	_pclose(gpr);
}

/*!
 * @brief ���\�b�hRouteDrawing::plot3D().3D���W���v���b�g���郁�\�b�h(��)
 * @param outputDataName 
 * @return �Ȃ�
 */
void RouteDrawing::plot3D(char* outputDataName)
{
	//�o�̓t�@�C�����̒�`
	char* plotImageName = "plot.jpeg"; //plot�����Ƃ��̉摜�t�@�C����(c39)
	char* plotXYImageName = "plot_X-Y.jpeg"; //X-Y���ʂŃv���b�g�����摜�t�@�C���̖��O(c39)
	char* plotXZImageName = "plot_X-Z.jpeg"; //X-Z���ʂŃv���b�g�����摜�t�@�C���̖��O(c39)
	char* plotYZImageName = "plot_Y-Z.jpeg"; //Y-Z���ʂŃv���b�g�����摜�t�@�C���̖��O(c39)

	//fprintf_s(gnuplot,"�����w��q",�ϐ�);�ŏ������w�肵�āCgnuplot�֏o�͂ł���
	//fputs("�R�}���h",gnuplot);�ŃR�}���h�����̂܂܏o�͂ł���
	fputs("set xlabel \"X-axis\"\n", gnuplot); //X���̃��x��
	fputs("set ylabel \"Y-axis\"\n", gnuplot); //Y���̃��x��
	fputs("set zlabel \"Z-axis\"\n", gnuplot); //Z���̃��x��

	fprintf_s(gnuplot, "splot \"%s/%s\" using 2:3:4 with lp\n", directoryName, outputDataName); //�f�[�^���v���b�g
	fputs("set title \"Path\"\n", gnuplot); //�O���t�̃^�C�g��
	fputs("set term jpeg size 1280,720\n", gnuplot); //jpeg�ŕۑ����邽��
	fprintf_s(gnuplot,"set output \"%s/%s\"\n", directoryName, plotImageName); //���O�����ĕۑ�
	fputs("rep\n", gnuplot); //�摜�̕ۑ��̔��f

	//fputs("set view 1,360,1,1\n", gnuplot); //(X,Y)����
	fprintf_s(gnuplot, "plot \"%s/%s\" u 2:3 with lp\n", directoryName, outputDataName); //�f�[�^���v���b�g
	fputs("set title \"Path X-Y\"\n", gnuplot); //�O���t�̃^�C�g��
	fputs("set xlabel \"X-axis\"\n", gnuplot); //�O���t��X�����x��
	fputs("set ylabel \"Y-axis\"\n", gnuplot); //�O���t��Y�����x��
	fprintf_s(gnuplot,"set output \"%s/%s\"\n", directoryName, plotXYImageName); //���O�����ĕۑ�
	fputs("rep\n", gnuplot); //�摜�̕ۑ��̔��f
	
	//fputs("set view 90,360,1,1\n", gnuplot); //(X,Z)����
	fprintf_s(gnuplot, "plot \"%s/%s\" u 2:4 with lp\n", directoryName, outputDataName); //�f�[�^���v���b�g
	fputs("set title \"Path X-Z\"\n", gnuplot); //�O���t�̃^�C�g��
	fputs("set xlabel \"X-axis\"\n", gnuplot); //�O���t��X�����x��
	fputs("set ylabel \"Z-axis\"\n", gnuplot); //�O���t��Y�����x��
	fprintf_s(gnuplot,"set output \"%s/%s\"\n", directoryName, plotXZImageName); //���O�����ĕۑ�
	fputs("rep\n", gnuplot); //�摜�̕ۑ��̔��f

	//fputs("set view 90,90,1,1\n", gnuplot); //(Y,Z)����
	fprintf_s(gnuplot, "plot \"%s/%s\" u 3:4 with lp\n", directoryName,outputDataName); //�f�[�^���v���b�g
	fputs("set title \"Path Y-Z\"\n", gnuplot); //�O���t�̃^�C�g��
	fputs("set xlabel \"Y-axis\"\n", gnuplot); //�O���t��X�����x��
	fputs("set ylabel \"Z-axis\"\n", gnuplot); //�O���t��Y�����x��
	fprintf_s(gnuplot,"set output \"%s/%s\"\n", directoryName, plotYZImageName); //���O�����ĕۑ�
	fputs("rep\n", gnuplot); //�摜�̕ۑ��̔��f

	//�m�F�p
	//fprintf_s(gnuplot, "splot \"%s/%s\" with points\n", directoryName, outputDataName); //�f�[�^���v���b�g
	//fputs("pause 3\n", gnuplot); //�v���b�g�������ʂ����΂炭�\��
	
	fputs("quit\n", gnuplot); //gnuplot���I��
	fflush(gnuplot); //�R�}���h��gnuplot�Ŏ��s

	return;
}

/*!
* @brief ���\�b�hRouteDrawing::gnuplotScript().��ŁC3D���W���v���b�g���Ċm�F���邽�߂�gnuplot�̃X�N���v�g���o�͂���
* @param checkNum dataFileName
* @return �Ȃ�
*/
void RouteDrawing::gnuplotScript(/*int checkNum,*/char* dataFileName)
{
	//if (checkNum == 1){ //�f�[�^��ۑ�����ꍇ�ɃX�N���v�g�𐶐�����(c32)
		FILE *gp; //!<gnuplot�X�N���v�g�p�̃|�C���^
		char filePath[NOC]; //!<�t�@�C���̃p�X���i�[����ϐ�
		char* scriptFileName = "splot.plt"; //�o�͂���X�N���v�g�t�@�C���̖��O(c39)

		sprintf_s(filePath, "%s/%s", directoryName, scriptFileName); //�o�̓p�X���Œ�
		fopen_s(&gp, filePath, "w"); //�t�@�C�����������݃��[�h�ŊJ��

		//���ʐݒ�
		fprintf_s(gp, "set multiplot layout 2,2\n"); //1�̃E�C���h�E�ɕ����̃v���b�g��\��
		fprintf_s(gp, "set grid xtics ytics ztics\n");

		//�͈͐ݒ�(�����k��)
		if (plotMode == 4){
			fprintf_s(gp, "set autoscale\n"); //�����k��(c47)
		}

		//Time-X
		fprintf_s(gp, "set title \"Time-X\"\n");
		fprintf_s(gp, "set xlabel \"Time(ms)\"\n");
		fprintf_s(gp, "set ylabel \"X-axis(mm)\"\n");
		if (plotMode == 1){ //(1)���S�Œ�p
			fprintf_s(gp, "set yrange [-5:5]\n"); //���S�Œ�p
		}
		else if (plotMode == 2){ //(2)�c�ړ��p
			fprintf_s(gp, "set yrange [-15:15]\n"); //�c�ړ��p
		}
		else if(plotMode == 3){ //(3)���ړ��p
			fprintf_s(gp, "set yrange [-60:60]\n"); //���ړ��p
		}
		fprintf_s(gp, "plot \"%s\" using 1:2 with linespoints\n", dataFileName);

		//Time-Y
		fprintf_s(gp, "set title \"Time-Y\"\n");
		fprintf_s(gp, "set xlabel \"Time(ms)\"\n");
		fprintf_s(gp, "set ylabel \"Y-axis(mm)\"\n");
		if (plotMode == 1){ //(1)���S�Œ�p
			fprintf_s(gp, "set yrange [-10:0]\n"); //���S�Œ�p
		}
		else if (plotMode == 2){ //(2)�c�ړ��p
			fprintf_s(gp, "set yrange [-15:15]\n"); //�c�ړ��p
		}
		else if(plotMode == 3){ //(3)���ړ��p
			fprintf_s(gp, "set yrange [-15:15]\n"); //���ړ��p
		}
		fprintf_s(gp, "plot \"%s\" using 1:3 with linespoints\n", dataFileName);

		//Time-Z
		fprintf_s(gp, "set title \"Time-Z\"\n");
		fprintf_s(gp, "set xlabel \"Time(ms)\"\n");
		fprintf_s(gp, "set ylabel \"Z-axis(mm)\"\n");
		if (plotMode == 1){ //(1)���S�Œ�p
			fprintf_s(gp, "set yrange [70:150]\n"); //���S�Œ�p
		}
		else if (plotMode == 2){ //(2)�c�ړ��p
			fprintf_s(gp, "set yrange [150:250]\n"); //�c�ړ��p
		}
		else if(plotMode == 3){ //(3)���ړ��p
			fprintf_s(gp, "set yrange [50:250]\n"); //���ړ��p
		}
		fprintf_s(gp, "plot \"%s\" using 1:4 with linespoints\n", dataFileName);

		fprintf_s(gp, "set xlabel \"X-axis(mm)\"\n");
		fprintf_s(gp, "set ylabel \"Y-axis(mm)\"\n");
		fprintf_s(gp, "set zlabel \"Z-axis(mm)\"\n");
		fprintf_s(gp, "set title \"Path\"\n");
		if (plotMode == 1){ //(1)���S�Œ�p
			fprintf_s(gp, "set xrange [-10:10]\n");
			fprintf_s(gp, "set yrange [-10:0]\n");
			fprintf_s(gp, "set zrange [80:120]\n");
		}
		else if (plotMode == 2){ //(2)�c�ړ��p
			fprintf_s(gp, "set xrange [-10:10]\n");
			fprintf_s(gp, "set yrange [-10:10]\n");
			fprintf_s(gp, "set zrange [50:300]\n");
		}
		else if(plotMode == 3){ //(3)���ړ��p
			fprintf_s(gp, "set xrange [-60:60]\n");
			fprintf_s(gp, "set yrange [-10:10]\n");
			fprintf_s(gp, "set zrange [150:250]\n");
		}
		fprintf_s(gp, "set view equal xy\n");
		//fprintf_s(gp, "splot \"%s\" every 4 using 2:3:4 with linespoints\n", dataFileName); //�_���Ԉ����ăv���b�g����Ƃ�(c40)
		fprintf_s(gp, "splot \"%s\" using 2:3:4 with linespoints\n", dataFileName); //�f�[�^�����̂܂܃v���b�g(c40)

		fprintf_s(gp, "unset multiplot\n");

		fclose(gp);
	//}
	return;
}

void RouteDrawing::plot3DRealTime(int countDataNum, outputData outputData[OUTPUTDATA_MAX])
{
	FILE *realtimeplot; //!<���A���^�C���Ɉʒu���v���b�g���邽�߂ɗ��p(c43)
	char pathName[NOC]; //!<�t�@�C���܂ł̃p�X��(c43)
	char* fileName = "realplot.dat"; //!<���A���^�C���v���b�g�p�̃f�[�^�t�@�C��(c43)

	sprintf_s(pathName, "%s/%s", directoryName, fileName); //�f�[�^�t�@�C���̕ۑ�����w��
	fopen_s(&realtimeplot, pathName, "a"); //�t�@�C����ǋL���[�h�ŊJ��(c43)
	fprintf_s(realtimeplot, "%f %f %f %f\n", outputData[countDataNum].totalTime, outputData[countDataNum].x, outputData[countDataNum].y, outputData[countDataNum].z); //�t�@�C���֏o��(c43)
	fclose(realtimeplot); //�t�@�C�������(c43)

	//�v���b�g�J�n
	if (first == true){ //1��ڂ͕`��̏����ݒ������
		fprintf_s(gpr, "set grid\n"); //�O���b�h��`��
		//fprintf_s(gpr, "unset key\n"); //�}�������
		fprintf_s(gpr, "set xlabel \"X-axis\"\n"); //X���̃��x����ݒ�
		fprintf_s(gpr, "set ylabel \"Y-axis\"\n"); //Y���̃��x����ݒ�
		fprintf_s(gpr, "set zlabel \"Z-axis\"\n"); //Z���̃��x����ݒ�

		if (plotMode == 1){ //(1)���S�Œ�p
			fprintf_s(gpr, "set xrange [-10:10]\n");
			fprintf_s(gpr, "set yrange [-20:0]\n");
			fprintf_s(gpr, "set zrange [50:200]\n");
		}
		else if (plotMode == 2){ //(2)�c�ړ��p
			fprintf_s(gpr, "set xrange [-10:10]\n");
			fprintf_s(gpr, "set yrange [-10:10]\n");
			fprintf_s(gpr, "set zrange [50:300]\n");
		}
		else if(plotMode == 3){ //(3)���ړ��p
			fprintf_s(gpr, "set xrange [-60:60]\n");
			fprintf_s(gpr, "set yrange [-10:10]\n");
			fprintf_s(gpr, "set zrange [150:250]\n");
		}
		else{ //(4)�����k��
			fprintf_s(gpr, "set autoscale\n"); //�����k��(c47)
		}

		first = false;
	}
	fprintf_s(gpr, "splot \"%s\" using 2:3:4 with linespoints\n", pathName);
	fflush(gpr);

	return;
}

/*!
* @brief ���\�b�hRouteDrawing::gnuplotScript().��ŁC���̏d�S���W���v���b�g���邽�߂�gnuplot�̃X�N���v�g���o�͂���
* @param checkNum dataFileName
* @return �Ȃ�
*/
void RouteDrawing::gnuplotScriptCoG(/*int checkNum, */char* cogFileName)
{
	//if (checkNum == 1){ //�f�[�^��ۑ�����ꍇ�ɃX�N���v�g�𐶐�����(c32)
		FILE *gp; //!<gnuplot�X�N���v�g�p�̃|�C���^
		char filePath[NOC]; //!<�t�@�C���̃p�X���i�[����ϐ�
		char* scriptFileName = "cog.plt"; //�o�͂���X�N���v�g�t�@�C���̖��O(c39)

		sprintf_s(filePath, "%s/%s", directoryName, scriptFileName); //�o�̓p�X���Œ�
		fopen_s(&gp, filePath, "w"); //�t�@�C�����������݃��[�h�ŊJ��

		fprintf_s(gp, "set xlabel \"X-axis\"\n");
		fprintf_s(gp, "set ylabel \"Y-axis\"\n");
		fprintf_s(gp, "set zlabel \"Z-axis\"\n");
		fprintf_s(gp, "set title \"Path\"\n");
		if (plotMode == 1){ //(1)���S�Œ�p
			fprintf_s(gp, "set xrange [-60:60]\n");
			fprintf_s(gp, "set yrange [-30:30]\n");
			fprintf_s(gp, "set zrange [50:300]\n");
		}
		else if (plotMode == 2){ //(2)�c�ړ��p
			fprintf_s(gp, "set xrange [-10:10]\n");
			fprintf_s(gp, "set yrange [-10:10]\n");
			fprintf_s(gp, "set zrange [50:300]\n");
		}
		else if (plotMode == 3){ //(3)���ړ��p
			fprintf_s(gp, "set xrange [-60:60]\n");
			fprintf_s(gp, "set yrange [-10:10]\n");
			fprintf_s(gp, "set zrange [150:250]\n");
		}
		else{
			fprintf_s(gp, "set autoscale\n"); //�����k��
		}
		fprintf_s(gp, "set view equal xy\n");
		fprintf_s(gp, "splot \"%s\" using 2:3:4 with linespoints\n", cogFileName); //�f�[�^�����̂܂܃v���b�g(c40)
		fclose(gp);
	//}
	return;
}