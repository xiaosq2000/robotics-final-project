
#include "HLrobotconfig.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using namespace Eigen;

namespace HLRobot
{
	//初始化TransMatrix
	double mTransMatrix[16] {0};
	double angles[6];

	//只使用一种姿态
	bool mConfig[3] = { 1, 1, 1 };

	const double L1 = 0.491, L2 = 0.45, L3 = 0.45, L4 = 0.084;
	const Vector3d w[6] = { Vector3d(0,0,1),Vector3d(0,1,0),Vector3d(0,1,0),Vector3d(0,0,1),Vector3d(0,1,0),Vector3d(0,0,1) };
	const Vector3d q_[6] = { Vector3d(0,0,0),Vector3d(0,0,L1),Vector3d(0,0,L1 + L2),Vector3d(0,0,0),Vector3d(0,0,L1 + L2 + L3),Vector3d(0,0,0) };

	Matrix4d get_g(Vector3d v, Vector3d w, double theta)
	{
		Matrix3d R, w_hat;
		Vector3d p;
		Matrix4d g=Matrix4d::Zero();
		w_hat << 0, -w(2), w(1),
			w(2), 0, -w(0),
			-w(1), w(0), 0;
		R = Matrix3d::Identity() + w_hat * sin(theta) + w_hat * w_hat * (1 - cos(theta));
		p = (Matrix3d::Identity() - R) * w.cross(v) + w * w.dot(v * theta);
		g.block<3, 3>(0, 0) = R;
		g.block<3, 1>(0, 3) = p;
		g(3, 3) = 1;
		return g;
	}

	void subpro1(Vector3d p, Vector3d q, Vector3d r, Vector3d w, double& theta)
	{
		Vector3d u = p - r, v = q - r;
		Vector3d up = u - w * w.dot(u);
		Vector3d vp = v - w * w.dot(v);
		if (fabs(w.dot(u) - w.dot(v)) > 1e-6 || fabs(up.norm() - vp.norm()) > 1e-6)
		{
			cout << "No solution for subproblem1! Because w'u /= w'v or ||u'||/=||v'||" << endl;
			cout << "w'u=" << w.dot(u) << "\t" << "w'v=" << w.dot(v) << endl;
			cout << "||u'||=" << up.norm() << "\t||v'||=" << vp.norm() << endl;
			theta = NULL;
			return;
		}
		if (up.norm() > 1e-6)
		{
			theta = atan2(w.dot(up.cross(vp)), up.dot(vp));
		}
		else
		{
			cout << "No solution for subproblem1! Because ||u'||=0" << endl;
			theta = NULL;
		}
	}

	void subpro2(Vector3d p, Vector3d q, Vector3d r, Vector3d w1, Vector3d w2, double& theta1, double& theta2)
	{
		Vector3d u = p - r, v = q - r;
		if (fabs(u.norm() - v.norm()) > 1e-6)
		{
			cout << "No solution for subproblem2! Because ||u||/=||v||" << endl;
			cout << "||u||=" << u.norm() << "\t||v||=" << v.norm() << endl;
			theta1 = theta2 - NULL;
			return;
		}
		double alpha = ((w1.dot(w2)) * w2.dot(u) - w1.dot(v)) / (pow(w1.dot(w2), 2) - 1);
		double beta = ((w1.dot(w2)) * w1.dot(v) - w2.dot(u)) / (pow(w1.dot(w2), 2) - 1);
		double gamma = -sqrt((pow(u.norm(), 2) - pow(alpha, 2) - pow(beta, 2) - 2 * alpha * beta * w1.dot(w2)) / pow(w1.cross(w2).norm(), 2));
		Vector3d z = alpha * w1 + beta * w2 + gamma * w1.cross(w2);
		Vector3d c = z + r;
		subpro1(p, c, r, w2, theta2);
		subpro1(q, c, r, -w1, theta1);
	}

	void subpro3(Vector3d p, Vector3d q, Vector3d r, Vector3d w, double d, double& theta)
	{
		Vector3d u = p - r, v = q - r;
		Vector3d up = u - w * w.dot(u);
		Vector3d vp = v - w * w.dot(v);
		if (fabs(w.dot(u) - w.dot(v)) > 1e-6 || fabs(up.norm() - vp.norm()) > 1e-6)
		{
			cout << "No solution for subproblem1! Because w' /= tw'v or ||u'||/=||v'||" << endl;
			cout << "w'u=" << w.dot(u) << "\t" << "w'v=" << w.dot(v) << endl;
			cout << "||u'||=" << up.norm() << "\t||v'||=" << vp.norm() << endl;
			theta = NULL;
			return;
		}
		double theta0 = atan2(w.dot(up.cross(vp)), up.dot(vp));
		double dp2 = pow(d, 2) - pow(w.dot(p - q),2);
		theta = theta0 - acos((pow(up.norm(), 2) + pow(vp.norm(), 2) - dp2) / (2 * up.norm() * vp.norm()));
	}

	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
		//单位转换，将xyz转换为m，ypr转换为rad
		x /= 1000.0;
		y /= 1000.0;
		z /= 1000.0;
		yaw /= 180/PI;
		pitch /= 180/PI;
		roll /= 180/PI;
		//根据xyz rpy求位姿矩阵，用于求逆运动学
		Matrix4d g0 = Matrix4d::Zero();
		g0(0, 0) = g0(1, 1) = g0(2, 2) = g0(3, 3) = 1;
		Matrix4d g1 = get_g(Vector3d(0, 0, 0), Vector3d(0, 0, 1), yaw);
		Matrix4d g2 = get_g(Vector3d(0, 0, 0), Vector3d(0, 1, 0), pitch);
		Matrix4d g3 = get_g(Vector3d(0, 0, 0), Vector3d(0, 0, 1), roll);
		Matrix4d g_r = g1 * g2 * g3 * g0;
		Matrix4d g_end = Matrix4d::Zero();
		g_end.block<3, 3>(0, 0) = g_r.block<3, 3>(0, 0);
		g_end.block<3, 1>(0, 3) = Vector3d(x, y, z);
		g_end(3, 3) = 1;
		for (int i = 0; i < 16; i++)
			mTransMatrix[i] = g_end(i / 4, i % 4);
	}

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, double &angle5, double &angle6)
	{
		//求逆运动学
		double theta[6];
		robotBackward(mTransMatrix, mConfig, theta);
		//单位转换，转化为角度
		angle1 = theta[0] * 180 / PI;
		angle2 = theta[1] * 180 / PI;
		angle3 = theta[2] * 180 / PI;
		angle4 = theta[3] * 180 / PI;
		angle5 = theta[4] * 180 / PI;
		angle6 = theta[5] * 180 / PI;
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6)
	{
		//单位转换，将angle1~6转换为rad
		//存储六个轴角度，用于求正运动学
		angles[0] = angle1 / (180 / PI);
		angles[1] = angle2 / (180 / PI);
		angles[2] = angle3 / (180 / PI);
		angles[3] = angle4 / (180 / PI);
		angles[4] = angle5 / (180 / PI);
		angles[5] = angle6 / (180 / PI);
	}

	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll)
	{
		//正运动学求得末态位姿，再转化为xyz rpy
		double Transmatrix[16];
		robotForward(angles, Transmatrix, mConfig);
		Matrix4d T;
		for (int i = 0; i < 16; i++)
			T(i / 4, i % 4) = Transmatrix[i];
		x = T(0, 3);
		y = T(1, 3);
		z = T(2, 3);
		Matrix4d T_r = T;
		T_r.block<3, 1>(0, 3) = Vector3d(0, 0, 0);
		Matrix4d g0 = Matrix4d::Zero();
		g0(0, 0) = g0(1, 1) = g0(2, 2) = g0(3, 3) = 1;
		Matrix4d g1 = T_r * g0.inverse();
		Vector4d p1 = Vector4d(0, 0, 1, 1);
		Vector4d p2 = g1 * p1;
		Vector3d r = Vector3d(0, 0, 0);
		subpro2(p1.block<3, 1>(0, 0), p2.block<3, 1>(0, 0), r, Vector3d(0, 0, 1), Vector3d(0, 1, 0), yaw, pitch);
		Vector4d p3 = Vector4d(0, 1, 0, 1);
		Vector4d p4 = get_g(Vector3d(0, 0, 0), Vector3d(0, 1, 0), -pitch) * get_g(Vector3d(0, 0, 0), Vector3d(0, 0, 1), -yaw) * g1 * p3;
		subpro1(p3.block<3, 1>(0, 0), p4.block<3, 1>(0, 0), r, Vector3d(0, 0, 1), roll);
		//单位转换，将xyz转换为mm，ypr转换为度
		x *= 1000; y *= 1000; z *= 1000;
		yaw *= 180/PI; pitch *= 180 / PI; roll *= 180 / PI;
	}

	

	/********************************************************************
	ABSTRACT:	机器人逆运动学

	INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

				config[3]：姿态，六轴机器人对应有8种姿态（即对应的逆运动学8个解），为了安全，
				实验室中我们只计算一种即可。config用来作为选解的标志数。

	OUTPUTS:    theta[6] 6个关节角, 单位为弧度

	RETURN:		<none>
	***********************************************************************/
	void robotBackward(const double* TransVector, bool* mconfig, double* theta)
	{
		Vector3d v[6];
		for (int i = 0; i < 6; i++)
			v[i] = -w[i].cross(q_[i]);
		Matrix4d gst;
		for (int i = 0; i < 16; i++)
			gst(i / 4, i % 4) = TransVector[i];
		Matrix4d gst0 = Matrix4d::Zero();
		gst0.block<3, 3>(0, 0) << -1, 0, 0,
								  0, -1, 0,
								  0, 0, 1;
		gst0.block<3, 1>(0, 3) = Vector3d(0, 0, L1 + L2 + L3 + L4);
		gst0(3, 3) = 1;
		Matrix4d g1 = gst * gst0.inverse();
		Vector4d p1(0, 0, L1 + L2 + L3, 1);
		Vector4d p2(0, 0, L1, 1);
		Vector3d r(0, 0, L1 + L2);
		subpro3(p1.block<3, 1>(0, 0), p2.block<3, 1>(0, 0), r, w[2], (g1 * p1 - p2).norm(), theta[2]);
		Vector4d p3 = get_g(v[2], w[2], theta[2]) * p1;
		Vector4d p4 = g1 * p1;
		r = Vector3d(0, 0, L1);
		subpro2(p3.block<3, 1>(0, 0), p4.block<3, 1>(0, 0), r, w[0], w[1], theta[0], theta[1]);
		Vector4d p5(0, 0, L1 + L2 + L3 + L4, 1);
		Vector4d p6 = get_g(v[2], w[2], -theta[2]) * get_g(v[1], w[1], -theta[1]) * get_g(v[0], w[0], -theta[0]) * g1 * p5;
		r = Vector3d(0, 0, L1 + L2 + L3);
		subpro2(p5.block<3, 1>(0, 0), p6.block<3, 1>(0, 0), r, w[3], w[4], theta[3], theta[4]);
		Vector4d p7(0, 1, 0, 0);
		r = Vector3d(0, 0, 0);
		Vector4d p8 = get_g(v[4], w[4], -theta[4]) * get_g(v[3], w[3], -theta[3]) * get_g(v[2], w[2], -theta[2]) * get_g(v[1], w[1], -theta[1]) * get_g(v[0], w[0], -theta[0]) * g1 * p7;
		subpro1(p7.block<3, 1>(0, 0), p8.block<3, 1>(0, 0), r, w[5], theta[5]);
	}

	/********************************************************************
	ABSTRACT:	机器人正运动学
	
	INPUTS:		q[6]: 6个关节角, 单位为弧度
	
	OUTPUTS:	config[3]：姿态，六轴机器人对应有8种姿态，为了安全，
				实验室中我们只计算一种即可。config用来作为选解的标志数。

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米
	
	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* q, double* TransVector, bool* mconfig)
	{
		
		Vector3d v[6];
		for (int i = 0; i < 6; i++)
			v[i] = -w[i].cross(q_[i]);
		Matrix4d gst0=Matrix4d::Zero();
		gst0.block<3, 3>(0, 0) << -1, 0, 0,
								  0, -1, 0,
								  0, 0, 1;
		gst0.block<3, 1>(0, 3) = Vector3d(0, 0, L1 + L2 + L3 + L4);
		gst0(3, 3) = 1;
		Matrix4d g[6] = { Matrix4d::Zero() };
		for (int i = 0; i < 6; i++)
		{
			g[i]=get_g(v[i], w[i], q[i]);
		}
		Matrix4d gst=g[0];
		for (int i = 1; i < 6; i++)
			gst *= g[i];
		gst *= gst0;
		for (int i = 0; i < 16; i++)
			TransVector[i] = gst(i / 4, i % 4);
	}
}
