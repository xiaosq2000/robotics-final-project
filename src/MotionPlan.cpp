
#include "MotionPlan.h"

using namespace std;
using namespace Eigen;


/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/

CHLMotionPlan::CHLMotionPlan()
{
	for (int i = 0; i < 6; i++)
	{
		mJointAngleBegin[i] = 0;
		mJointAngleEnd[i] = 0;
	}

	for (int i = 0; i < 16; i++)
	{
		mStartMatrixData[i] = 0;
		mEndMatrixData[i] = 0;
	}

	mSampleTime = 0.001;
	mVel = 0;
	mAcc = 0;
	mDec = 0;
}

/********************************************************************
ABSTRACT:	析构函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
CHLMotionPlan::~CHLMotionPlan()
{
}

/********************************************************************
ABSTRACT:	设置采样时间

INPUTS:		sampleTime			采样时间，单位S

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetSampleTime(double sampleTime)
{
	if (sampleTime < 0.001)
	{
		mSampleTime = 0.001;
	}
	else
	{
		mSampleTime = sampleTime;
	}
}

/********************************************************************
ABSTRACT:	设置运动参数

INPUTS:		vel			速度，单位m/s
			acc			加速度，单位m/s/s
			dec			减速度，单位m / s / s

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetProfile(double vel, double acc, double dec)
{
	mVel = vel;
	mAcc = acc;
	mDec = dec;
}

/********************************************************************
ABSTRACT:	设置规划的起始单位和技术点位

INPUTS:		startPos			起始点位笛卡尔坐标
			endPos				结束点位笛卡尔坐标

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetPlanPoints(PosStruct startPos, PosStruct endPos)
{
	mStartPos.x = startPos.x;
	mStartPos.y = startPos.y;
	mStartPos.z = startPos.z;
	mStartPos.yaw = startPos.yaw;
	mStartPos.pitch = startPos.pitch;
	mStartPos.roll = startPos.roll;

	mEndPos.x = endPos.x;
	mEndPos.y = endPos.y;
	mEndPos.z = endPos.z;
	mEndPos.yaw = endPos.yaw;
	mEndPos.pitch = endPos.pitch;
	mEndPos.roll = endPos.roll;

	/*
	double startAngle[3], endAngle[3];
	startAngle[0] = startPos.yaw * PI / 180;
	startAngle[1] = startPos.pitch * PI / 180;
	startAngle[2] = startPos.roll * PI / 180;

	endAngle[0] = endPos.yaw * PI / 180;
	endAngle[1] = endPos.pitch * PI / 180;
	endAngle[2] = endPos.roll * PI / 180;

	mStartMatrixData[0] = cos(startAngle[0])*cos(startAngle[1])*cos(startAngle[2]) - sin(startAngle[0])*sin(startAngle[2]);
	mStartMatrixData[1] = -cos(startAngle[0])*cos(startAngle[1])*sin(startAngle[2]) - sin(startAngle[0])*cos(startAngle[2]);
	mStartMatrixData[2] = cos(startAngle[0])*sin(startAngle[1]);
	mStartMatrixData[3] = startPos.x / 1000;

	mStartMatrixData[4] = sin(startAngle[0])*cos(startAngle[1])*cos(startAngle[2]) + cos(startAngle[0])*sin(startAngle[2]);
	mStartMatrixData[5] = -sin(startAngle[0])*cos(startAngle[1])*sin(startAngle[2]) + cos(startAngle[0])*cos(startAngle[2]);
	mStartMatrixData[6] = sin(startAngle[0])*sin(startAngle[1]);
	mStartMatrixData[7] = startPos.y / 1000;

	mStartMatrixData[8] = -sin(startAngle[1])*cos(startAngle[2]);
	mStartMatrixData[9] = sin(startAngle[1])*sin(startAngle[2]);
	mStartMatrixData[10] = cos(startAngle[1]);
	mStartMatrixData[11] = startPos.z / 1000;

	mStartMatrixData[12] = 0;
	mStartMatrixData[13] = 0;
	mStartMatrixData[14] = 0;
	mStartMatrixData[15] = 1;

	mEndMatrixData[0] = cos(endAngle[0])*cos(endAngle[1])*cos(endAngle[2]) - sin(endAngle[0])*sin(endAngle[2]);
	mEndMatrixData[1] = -cos(endAngle[0])*cos(endAngle[1])*sin(endAngle[2]) - sin(endAngle[0])*cos(endAngle[2]);
	mEndMatrixData[2] = cos(endAngle[0])*sin(endAngle[1]);
	mEndMatrixData[3] = endPos.x / 1000;

	mEndMatrixData[4] = sin(endAngle[0])*cos(endAngle[1])*cos(endAngle[2]) + cos(endAngle[0])*sin(endAngle[2]);
	mEndMatrixData[5] = -sin(endAngle[0])*cos(endAngle[1])*sin(endAngle[2]) + cos(endAngle[0])*cos(endAngle[2]);
	mEndMatrixData[6] = sin(endAngle[0])*sin(endAngle[1]);
	mEndMatrixData[7] = endPos.y / 1000;

	mEndMatrixData[8] = -sin(endAngle[1])*cos(endAngle[2]);
	mEndMatrixData[9] = sin(endAngle[1])*sin(endAngle[2]);
	mEndMatrixData[10] = cos(endAngle[1]);
	mEndMatrixData[11] = endPos.z / 1000;

	mEndMatrixData[12] = 0;
	mEndMatrixData[13] = 0;
	mEndMatrixData[14] = 0;
	mEndMatrixData[15] = 1;*/

	double angle1, angle2, angle3, angle4, angle5, angle6;
	HLRobot::SetRobotEndPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll);
	HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);

	mJointAngleBegin[0] = angle1;
	mJointAngleBegin[1] = angle2;
	mJointAngleBegin[2] = angle3;
	mJointAngleBegin[3] = angle4;
	mJointAngleBegin[4] = angle5;
	mJointAngleBegin[5] = angle6;
	cout << "Begin Joints:" << mJointAngleBegin[0] << "\t" << mJointAngleBegin[1] << "\t" << mJointAngleBegin[2] << "\t" << mJointAngleBegin[3] << "\t" << mJointAngleBegin[4] << "\t" << mJointAngleBegin[5] << "\t" << endl;

	HLRobot::SetRobotEndPos(endPos.x, endPos.y, endPos.z, endPos.yaw, endPos.pitch, endPos.roll);
	HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);
	mJointAngleEnd[0] = angle1;
	mJointAngleEnd[1] = angle2;
	mJointAngleEnd[2] = angle3;
	mJointAngleEnd[3] = angle4;
	mJointAngleEnd[4] = angle5;
	mJointAngleEnd[5] = angle6;
	cout << "End Joints:" << mJointAngleEnd[0] << "\t" << mJointAngleEnd[1] << "\t" << mJointAngleEnd[2] << "\t" << mJointAngleEnd[3] << "\t" << mJointAngleEnd[4] << "\t" << mJointAngleEnd[5] << "\t" << endl;
}

/********************************************************************
ABSTRACT:	运动轨迹规划部分（以关节空间为例）

INPUTS:		pos						二维位置向量

OUTPUTS:	pos						二维位置向量（第一维：位置个数，第二维：每个轴的关节角度，单位弧度）

RETURN:		<none>
***********************************************************************/

/******
 * 参考步骤
 * 步骤1：创建文件并写入初始角度
 * 步骤2：计算每个轴旋转的角度
 * 步骤3：计算每个轴移动到终止点所需要时间
 * 步骤4：根据采样时间计算离散点位数
 * 步骤5：根据离散点位数得到每刻的关节角数值
 * 步骤6：将每时刻的关节角数值写入到文件
 */
void CHLMotionPlan::GetPlanPoints(const string &kFilePath)
{
	ofstream outfile; //创建文件
	outfile.open(kFilePath);
	outfile << mJointAngleBegin[0] << "  "
			<< mJointAngleBegin[1] << "  "
			<< mJointAngleBegin[2] << "  "
			<< mJointAngleBegin[3] << "  "
			<< mJointAngleBegin[4] << "  "
			<< mJointAngleBegin[5] << "  ";
	outfile << endl; //保存初始的时间、六个关节角度

	//完成代码
	double dtheta[6];
	int sign[6];
	for (int i = 0; i < 6; i++)
	{
		dtheta[i] = fabs(mJointAngleEnd[i] - mJointAngleBegin[i]);
		if (mJointAngleEnd[i] - mJointAngleBegin[i] > 0)
			sign[i] = 1;
		else
			sign[i] = -1;
	}
	//cout << "dtheta=" << dtheta[0] << "\t" << dtheta[1] << "\t" << dtheta[2] << "\t" << dtheta[3] << "\t" << dtheta[4] << "\t" << dtheta[5] << endl;
	//cout << "sign=" << sign[0] << "\t" << sign[1] << "\t" << sign[2] << "\t" << sign[3] << "\t" << sign[4] << "\t" << sign[5] << endl;
	double tAccEnd[6], tDecBegin[6], tEnd[6];
	double min_theta = pow(mVel, 2) / (2 * mAcc) + pow(mVel, 2) / (2 * mDec);
	for (int i = 0; i < 6; i++)
	{
		if (dtheta[i] < min_theta)
		{
			//cout << "theta" << i + 1 << " too small" << endl;
			tAccEnd[i] = tDecBegin[i] = sqrt(2 * dtheta[i] * mDec / (mAcc * mDec + pow(mAcc, 2)));
			tEnd[i] = tDecBegin[i] + (mAcc * tAccEnd[i]) / mDec;
		}
		else
		{
			tAccEnd[i] = mVel / mAcc;
			tDecBegin[i] = tAccEnd[i] + (dtheta[i] - min_theta) / mVel;
			tEnd[i] = tDecBegin[i] + mVel / mDec;
		}
	}
	//cout << "tAccEnd:" << tAccEnd[0] << '\t' << tAccEnd[1] << '\t' << tAccEnd[2] << '\t' << tAccEnd[3] << '\t' << tAccEnd[4] << '\t' << tAccEnd[5] << endl;
	//cout << "tDecBegin:" << tDecBegin[0] << '\t' << tDecBegin[1] << '\t' << tDecBegin[2] << '\t' << tDecBegin[3] << '\t' << tDecBegin[4] << '\t' << tDecBegin[5] << endl;

	double maxt = 0;
	for (int i = 0; i < 6; i++)
		if (tEnd[i] > maxt)
			maxt = tEnd[i];

	//cout << "maxt=" << maxt << endl;
	double angles[6] = {0};
	for (int i = 0; i < 6; i++)
	{
		angles[i] = mJointAngleBegin[i];
		//angles[i] += sign[i] * (mSampleTime * mAcc) * mSampleTime / 2;
	}
	outfile << angles[0] << "  " << angles[1] << "  " << angles[2] << "  " << angles[3] << "  " << angles[4] << "  " << angles[5] << endl;

	for (double t = mSampleTime; t < maxt; t += mSampleTime)
	{
		for (int i = 0; i < 6; i++)
		{
			if (t < tAccEnd[i])
			{
				angles[i] += sign[i] * mAcc * (2 * t - mSampleTime) * mSampleTime / 2;
			}
			else if (t > tAccEnd[i] && t < tDecBegin[i])
			{
				angles[i] += sign[i] * mVel * mSampleTime;
			}
			else if (t > tDecBegin[i] && t < tEnd[i])
			{
				double tmp = tEnd[i] - t;
				angles[i] += sign[i] * mDec * (2 * tmp + mSampleTime) * mSampleTime / 2;
			}
		}
		outfile << angles[0] << "  " << angles[1] << "  " << angles[2] << "  " << angles[3] << "  " << angles[4] << "  " << angles[5] << endl;
	}

	outfile.close();
}

void CHLMotionPlan::GetPlanPoints_line(const string &kFilePath)
{
	//完成代码
	ofstream outfile; //创建文件
	outfile.open(kFilePath, ios::app);
	outfile << mJointAngleBegin[0] << "  "
			<< mJointAngleBegin[1] << "  "
			<< mJointAngleBegin[2] << "  "
			<< mJointAngleBegin[3] << "  "
			<< mJointAngleBegin[4] << "  "
			<< mJointAngleBegin[5] << "  ";
	outfile << endl; //保存初始的时间、六个关节角度

	double dx = mEndPos.x - mStartPos.x;
	double dy = mEndPos.y - mStartPos.y;
	double dz = mEndPos.z - mStartPos.z;
	double dis = sqrt(dx * dx + dy * dy + dz * dz);
	double unit_vec[3] = {dx / dis, dy / dis, dz / dis};
	double min_dis = pow(mVel, 2) / (2 * mAcc) + pow(mVel, 2) / (2 * mDec);

	double tAccEnd, tDecBegin, tEnd;

	if (dis < min_dis)
	{
		tAccEnd = tDecBegin = sqrt(2 * dis * mDec / (mAcc * mDec + pow(mAcc, 2)));
		tEnd = tDecBegin + (mAcc * tAccEnd) / mDec;
	}
	else
	{
		tAccEnd = mVel / mAcc;
		tDecBegin = tAccEnd + (dis - min_dis) / mVel;
		tEnd = tDecBegin + mVel / mDec;
	}
	cout << "tEnd=" << tEnd << endl;

	double tmpx = mStartPos.x, tmpy = mStartPos.y, tmpz = mStartPos.z;
	double angles[6];
	for (double t = mSampleTime; t < tEnd; t += mSampleTime)
	{
		if (t < tAccEnd)
		{
			tmpx += unit_vec[0] * mAcc * (2 * t - mSampleTime) * mSampleTime / 2;
			tmpy += unit_vec[1] * mAcc * (2 * t - mSampleTime) * mSampleTime / 2;
			tmpz += unit_vec[2] * mAcc * (2 * t - mSampleTime) * mSampleTime / 2;
		}
		else if (t > tAccEnd && t < tDecBegin)
		{
			tmpx += unit_vec[0] * mVel * mSampleTime;
			tmpy += unit_vec[1] * mVel * mSampleTime;
			tmpz += unit_vec[2] * mVel * mSampleTime;
		}
		else
		{
			double tmp = tEnd - t;
			tmpx += unit_vec[0] * mDec * (2 * tmp + mSampleTime) * mSampleTime / 2;
			tmpy += unit_vec[1] * mDec * (2 * tmp + mSampleTime) * mSampleTime / 2;
			tmpz += unit_vec[2] * mDec * (2 * tmp + mSampleTime) * mSampleTime / 2;
		}
		HLRobot::SetRobotEndPos(tmpx, tmpy, tmpz, mStartPos.yaw, mStartPos.pitch, mStartPos.roll);
		HLRobot::GetJointAngles(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
		outfile << angles[0] << "  " << angles[1] << "  " << angles[2] << "  " << angles[3] << "  " << angles[4] << "  " << angles[5] << endl;
	}

	outfile.close();
}

/*
double dPos[6] = { mEndPos.x - mStartPos.x,mEndPos.y - mStartPos.y,mEndPos.z - mStartPos.z,mEndPos.yaw - mStartPos.yaw,mEndPos.pitch - mStartPos.pitch,mEndPos.roll - mStartPos.roll };
	int sign[6];
	for (int i = 0; i < 6; i++)
		if (dPos[i] > 0)
			sign[i] = 1;
		else
			sign[i] = -1;
	double tAccEnd[6], tDecBegin[6], tEnd[6];
	double min_dis = pow(mVel, 2) / (2 * mAcc) + pow(mVel, 2) / (2 * mDec);

	for (int i = 0; i < 6; i++)
	{
		if (fabs(dPos[i]) < min_dis)
		{
			tAccEnd[i] = tDecBegin[i] = sqrt(2 * fabs(dPos[i]) * mDec / (mAcc * mDec + pow(mAcc, 2)));
			tEnd[i] = tDecBegin[i] + (mAcc * tAccEnd[i]) / mDec;
		}
		else
		{
			tAccEnd[i] = mVel / mAcc;
			tDecBegin[i] = tAccEnd[i] + (fabs(dPos[i]) - min_dis) / mVel;
			tEnd[i] = tDecBegin[i] + mVel / mDec;
		}
	}

	double maxt = 0;
	for (int i = 0; i < 6; i++)
		if (tEnd[i] > maxt)
			maxt = tEnd[i];
	cout << "maxt=" << maxt << endl;
	double angles[6] = { 0 };
	double tmpPos[6];
	tmpPos[0] = mStartPos.x;
	tmpPos[1] = mStartPos.y;
	tmpPos[2] = mStartPos.z;
	tmpPos[3] = mStartPos.yaw;
	tmpPos[4] = mStartPos.pitch;
	tmpPos[5] = mStartPos.roll;



	for (double t = mSampleTime; t < maxt; t += mSampleTime)
	{
		for (int i = 0; i < 6; i++)
		{
			if (t < tAccEnd[i])
			{
				tmpPos[i] += sign[i] * mAcc * (2 * t - mSampleTime) * mSampleTime / 2;
			}
			else if (t > tAccEnd[i] && t < tDecBegin[i])
			{
				tmpPos[i] += sign[i] * mVel * mSampleTime;
			}
			else if (t > tDecBegin[i] && t < tEnd[i])
			{
				double tmp = tEnd[i] - t;
				tmpPos[i] += sign[i] * mDec * (2 * tmp + mSampleTime) * mSampleTime / 2;
			}
		}
		HLRobot::SetRobotEndPos(tmpPos[0], tmpPos[1], tmpPos[2], tmpPos[3], tmpPos[4], tmpPos[5]);
		HLRobot::GetJointAngles(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
		outfile << angles[0] << "  " << angles[1] << "  " << angles[2] << "  " << angles[3] << "  " << angles[4] << "  " << angles[5] << endl;
	}*/
