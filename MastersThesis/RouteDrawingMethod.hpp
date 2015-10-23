/*
* @file RouteDrawingMethod.hpp 
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief gnuplot�����֘A�̃N���X�̃w�b�_
* @date 2014.12.10
* @author H.Shigehara
*/

/* �C���N���[�h�K�[�h */
#ifndef __ROUTEDRAWINGMETHOD_HPP__
#define __ROUTEDRAWINGMETHOD_HPP__

/* �C���N���[�h */
#include "3DPathTrackingUsingtheKINECT.hpp"

/*!
* @class RouteDrawing
* @brief �o�H�`��p�̃N���X
*/
class RouteDrawing
{
private:
	FILE *gnuplot; //�v���b�g�p
	FILE *gpr; //���A���^�C���o�͗p�̃v���b�g(c43)
	bool first = true; //
	int plotMode = 1; //!<gnuplot�̃X�N���v�g���t�@�C���ɏo�͂���ۂɁC�v������Ώۂ̏󋵂ɉ����Ēl�͈̔͂�ύX����.���S�Œ�p(1)�C�c�ړ��p(2)�C���ړ��p(3)(c46)�C�����k��(4)(c47)

public:
	RouteDrawing(); //!<�R���X�g���N�^
	~RouteDrawing(); //!<�f�X�g���N�^
	void plot3D(const string* outputDataName); //!<3D���W�t�@�C�����v���b�g���郁�\�b�h
	void gnuplotScript(const string* dataFileName); //
	void plot3DRealTime(int countDataNum, outputData outputData[OUTPUTDATA_MAX]); //���A���^�C����gnuplot�ɏo��(c43)
	void gnuplotScriptCoG(const string* cogFileName); //!<���̏d�S���W���v���b�g���郁�\�b�h(c52)
};

/* �C���N���[�h�K�[�h�̏I�� */
#endif /* __ROUTEDRAWINGMETHOD_HPP__ */