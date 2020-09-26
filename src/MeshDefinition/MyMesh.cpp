#include "MyMesh.h"

void MyMesh::ComputeLocalAveragingRegion(int kind)
{//类型:重心、Voronoi、混合
	V_LocalAverageRegionArea.clear();
	V_LocalAverageRegionArea.resize(NFaces());
	//遍历每一个面，获取面的"中心"
	for (int i = 0; i < NFaces(); i++)
	{
		Eigen::Vector3d center1,center2,center3;
		int v1, v2, v3;
		getFaceVertices(i, v1, v2, v3);
		Point p1 = getPoint(v1), p2 = getPoint(v2), p3 = getPoint(v3);
		Point m12 = (p1 + p2) / 2, m13 = (p1 + p3) / 2, m23 = (p2 + p3) / 2;
		if (kind ==0)//重心坐标
		{
			center1 = ((p1 + p2 + p3) / 3).toEigen3d();
			center2 = center1;
			center3 = center1;
		}
		else if (kind == 1)//Voronoi
		{
			Eigen::Vector3d f_normal = getFaceNormal(i);
			//Eigen::vec3d
			center1 = ComputeTriangleCenter(f_normal,p1.toEigen3d(),p2.toEigen3d(),p3.toEigen3d());
			center2= ComputeTriangleCenter(f_normal, p2.toEigen3d(), p3.toEigen3d(), p1.toEigen3d());
			center3= ComputeTriangleCenter(f_normal, p3.toEigen3d(), p1.toEigen3d(), p2.toEigen3d());
			double test1=(center1 - m13.toEigen3d()).dot(center1 - m12.toEigen3d());
			double test2=(center2 - m23.toEigen3d()).dot(center2 - m12.toEigen3d());
			double test3=(center3 - m13.toEigen3d()).dot(center3 - m23.toEigen3d());
		}
		else if (kind == 2) //Mixed
		{
			Eigen::Vector3d f_normal = getFaceNormal(i);
			double angle1, angle2, angle3;
			getFaceAngles(i, angle1, angle2, angle3);
			if (angle1 <= 0.5 * M_PI)
				center1 = ComputeTriangleCenter(f_normal, p1.toEigen3d(), p2.toEigen3d(), p3.toEigen3d());
			else
				center1 = (p2 + p3).toEigen3d() / 2;
			if (angle2 <= 0.5 * M_PI)
				center2 = ComputeTriangleCenter(f_normal, p2.toEigen3d(), p3.toEigen3d(), p1.toEigen3d());
			else
				center2 = (p1 + p3).toEigen3d() / 2;
			if (angle3 <= 0.5 * M_PI)
				center3 = ComputeTriangleCenter(f_normal, p3.toEigen3d(), p1.toEigen3d(), p2.toEigen3d());
			else
				center3 = (p2 + p1).toEigen3d() / 2;
		}
		V_LocalAverageRegionArea[v1] += ComputeArea(p1.toEigen3d(), m12.toEigen3d(), center1) + ComputeArea(p1.toEigen3d(), m13.toEigen3d(), center1);
		V_LocalAverageRegionArea[v2] += ComputeArea(p2.toEigen3d(), m12.toEigen3d(), center2) + ComputeArea(p2.toEigen3d(), m23.toEigen3d(), center2);
		V_LocalAverageRegionArea[v3] += ComputeArea(p3.toEigen3d(), m13.toEigen3d(), center3) + ComputeArea(p3.toEigen3d(), m23.toEigen3d(), center3);
	}
}

double MyMesh::ComputeArea(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3)
{
	Eigen::Vector3d e12 = (p2 - p1);
	Eigen::Vector3d e13 = (p3 - p1);
	double area = 0.5 * (e12.cross(e13)).norm();
	return area;
}

Eigen::Vector3d MyMesh::ComputeTriangleCenter(Eigen::Vector3d f_normal, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3)
{
	Eigen::Vector3d e12 = p2- p1;
	Eigen::Vector3d e13 = p3 - p1;
	Eigen::Vector3d e23 = p3 - p2;
	Eigen::Vector3d e12n = f_normal.cross(e12);
	double t1up = (e12.x() - e13.x()) * (p3[0] - p1[0]) + (e12.y() - e13.y()) * (p3[1] - p1[1]) + (e12.z() - e13.z()) * (p3[2] - p1[2]);
	double t1down = e12n.x() * (p3[0] - p1[0]) + e12n.y() * (p3[1] - p1[1]) + e12n.z() * (p3[2] - p1[2]);
	double t1 = -t1up / t1down;
	Eigen::Vector3d center1 = (p1+p2)/2 + t1 * e12n;
	return center1;
}

void MyMesh::UpdateMeanCurvature()
{
	VCurvature.clear();
	VCurvature.resize(NVertices());
	std::vector<double> x_curvature(NVertices(), 0);
	std::vector<double> y_curvature(NVertices(), 0);
	std::vector<double> z_curvature(NVertices(), 0);
	for (int i = 0; i < NFaces(); i++)
	{
		int v1, v2, v3;
		double angle1, angle2, angle3;
		getFaceVertices(i, v1, v2, v3);
		getFaceAngles(i, angle1, angle2, angle3);
		Point p1 = getPoint(v1), p2 = getPoint(v2), p3 = getPoint(v3);
		//v1v2边
		x_curvature[v1] += (atan(angle3)) / V_LocalAverageRegionArea[v1]*(p2-p1).v[0];
		y_curvature[v1]+= (atan(angle3)) / V_LocalAverageRegionArea[v1] * (p2 - p1).v[1];
		z_curvature[v1]+= (atan(angle3)) / V_LocalAverageRegionArea[v1] * (p2 - p1).v[2];
		//v2v1边
		x_curvature[v2] += (atan(angle3)) / V_LocalAverageRegionArea[v2] * (p1 - p2).v[0];
		y_curvature[v2] += (atan(angle3)) / V_LocalAverageRegionArea[v2] * (p1 - p2).v[1];
		z_curvature[v2] += (atan(angle3)) / V_LocalAverageRegionArea[v2] * (p1 - p2).v[2];
		//v1v3边
		x_curvature[v1] += (atan(angle2)) / V_LocalAverageRegionArea[v1] * (p3 - p1).v[0];
		y_curvature[v1] += (atan(angle2)) / V_LocalAverageRegionArea[v1] * (p3 - p1).v[1];
		z_curvature[v1] += (atan(angle2)) / V_LocalAverageRegionArea[v1] * (p3 - p1).v[2];
		//v3v1边		
		x_curvature[v3] += (atan(angle2)) / V_LocalAverageRegionArea[v3] * (p1 - p3).v[0];
		y_curvature[v3] += (atan(angle2)) / V_LocalAverageRegionArea[v3] * (p1 - p3).v[1];
		z_curvature[v3] += (atan(angle2)) / V_LocalAverageRegionArea[v3] * (p1 - p3).v[2];
		//v2v3边
		x_curvature[v2] += (atan(angle1)) / V_LocalAverageRegionArea[v2] * (p3 - p2).v[0];
		y_curvature[v2] += (atan(angle1)) / V_LocalAverageRegionArea[v2] * (p3 - p2).v[1];
		z_curvature[v2] += (atan(angle1)) / V_LocalAverageRegionArea[v2] * (p3 - p2).v[2];
		//v3v2边
		x_curvature[v3] += (atan(angle1)) / V_LocalAverageRegionArea[v3] * (p2 - p3).v[0];
		y_curvature[v3] += (atan(angle1)) / V_LocalAverageRegionArea[v3] * (p2 - p3).v[1];
		z_curvature[v3] += (atan(angle1)) / V_LocalAverageRegionArea[v3] * (p2 - p3).v[2];
	}
	for (int i = 0; i < NVertices(); i++)
	{
		VCurvature[i] =0.5* sqrt(x_curvature[i] * x_curvature[i] + y_curvature[i] * y_curvature[i] + z_curvature[i] * z_curvature[i]);
	}
}

void MyMesh::UpdateGaussianCurvature()
{
	VCurvature.clear();
	VCurvature.resize(NVertices());

	std::vector<double> v_angledefect(NVertices(),2*M_PI);
	for (int i = 0; i < NFaces(); i++)
	{
		int v1, v2, v3;
		double angle1, angle2, angle3;
		getFaceVertices(i, v1, v2, v3);
		getFaceAngles(i, angle1, angle2, angle3);
		v_angledefect[v1] -= angle1;
		v_angledefect[v2] -= angle2;
		v_angledefect[v3] -= angle3;
	}
	for (int i = 0; i < NVertices(); i++)
	{
		VCurvature[i]=	v_angledefect[i] / V_LocalAverageRegionArea[i];
	}
}
