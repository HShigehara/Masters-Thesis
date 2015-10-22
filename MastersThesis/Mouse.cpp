/*
* @file main.cpp
* @link https://github.com/HShigehara/3DPathTrackingUsingtheKINECT.git
* @brief Mouse���������߂̃��\�b�h�Q
* @date 2015.05.11
* @author H.Shigehara
*/

#include "stdafx.h"
//#include "3DPathTrackingUsingtheKINECT.hpp" //�w�b�_�t�@�C���̃C���N���[�h

void onMouse(int event, int x, int y, int flags, void* param)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = abs(x - origin.x);
		selection.height = abs(y - origin.y);
		selection &= Rect(0, 0, image.cols, image.rows);
	}

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		break;
	case CV_EVENT_LBUTTONUP:
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
		{
			trackObject = -1;
			mouseFlag = true;
		}
		break;
	}

}