#ifndef _MOTION_PLAN_H_
#define _MOTION_PLAN_H_

#include <iostream>
#include <fstream>
#include <Windows.h>
#include <vector>
#include <algorithm>
#include "Eigen/Dense"
#include "HLrobotconfig.h"

using namespace std;

struct PosStruct
{
	// PosStruct() : x(0), y(0), z(0), yaw(0), pitch(0), roll(0) {}
	// PosStruct(double ix, double iy, double iz, double iyaw, double ipitch, double iroll) : x(ix), y(iy), z(iz), yaw(iyaw), pitch(ipitch), roll(iroll) {}
	double x;	  // x坐标，单位mm
	double y;	  // y坐标，单位mm
	double z;	  // z坐标，单位mm
	double yaw;	  // yaw坐标，单位度
	double pitch; // pitch坐标，单位度
	double roll;  // roll坐标，单位度
};

class CHLMotionPlan
{
private:
	double mJointAngleBegin[6];	 //起始点位的关节角度,单位度
	double mJointAngleEnd[6];	 //结束点位的关节角度，单位度
	double mStartMatrixData[16]; //起始点位的转换矩阵数组
	double mEndMatrixData[16];	 //结束点位的转换矩阵数组
	double mSampleTime;			 //采样点位，单位S
	double mVel;				 //速度，单位m/s
	double mAcc;				 //加速度，单位m/s/s
	double mDec;				 //减速度，单位m / s / s
	bool mConfig[3];			 //机器人姿态
	PosStruct mStartPos;
	PosStruct mEndPos;

public:
	CHLMotionPlan();
	virtual ~CHLMotionPlan();

	void SetSampleTime(double sampleTime);					  //设置采样时间
	void SetPlanPoints(PosStruct startPos, PosStruct endPos); //输入起始点位和结束点位的笛卡尔坐标
	void SetProfile(double vel, double acc, double dec);	  //设置运动参数，速度、加速度和减速度
	void GetPlanPoints(const string &kFilePath);			  //关节空间梯形速度规划
	void GetPlanPoints_line(const string &kFilePath);		  //笛卡尔空间直线轨迹梯形速度规划
};

#endif
