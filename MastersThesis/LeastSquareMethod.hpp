/*
* @file LeastSquareMethod.hpp
* @link https://github.com/HShigehara/Masters-Thesis.git
* @brief �ŏ����@���s���N���X�̃w�b�_
* @date 2015.07.16
* @author H.Shigehara
*/

/* �C���N���[�h�K�[�h */
#ifndef __LEASTSQUAREMETHOD_HPP__
#define __LEASTSQUAREMETHOD_HPP__

/* �C���N���[�h */
#include "3DPathTrackingUsingtheKINECT.hpp"

/*!
* @class LeastSquareMethod
* @brief �ŏ����@���s���N���X
*/
class LeastSquareMethod
{
private:


public:
	LeastSquareMethod(); //!<�R���X�g���N�^
	~LeastSquareMethod(); //!<�f�X�g���N�^

	void getSphereData(int actualExtractedNum); //!<�ŏ����@�ɂ���ċ��߂����̏����擾���郁�\�b�h
};

/* �C���N���[�h�K�[�h�̏I�� */
#endif /* __LEASTSQUAREMETHOD_HPP__ */