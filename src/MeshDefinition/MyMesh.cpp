#include "MyMesh.h"
#include <random>
#include <fstream>

void MyMesh::ComputeLAR(int kind)
{//类型:重心、Voronoi、混合
	LAR.setZero();
	LAR.resize(NFaces());
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
		LAR[v1] += ComputeArea(p1.toEigen3d(), m12.toEigen3d(), center1) + ComputeArea(p1.toEigen3d(), m13.toEigen3d(), center1);
		LAR[v2] += ComputeArea(p2.toEigen3d(), m12.toEigen3d(), center2) + ComputeArea(p2.toEigen3d(), m23.toEigen3d(), center2);
		LAR[v3] += ComputeArea(p3.toEigen3d(), m13.toEigen3d(), center3) + ComputeArea(p3.toEigen3d(), m23.toEigen3d(), center3);
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
	ComputeLaplacian(0);//算cotangent的拉普拉斯；
	int nv = NVertices();
	Vertices.resize(nv, 3);
	Vertices.setZero();
	for (int i=0;i<nv;i++)
	{
		Point p = getPoint(i);
		Vertices(i, 0) = p[0];
		Vertices(i, 1) = p[1];
		Vertices(i, 2) = p[2];
	}
	Eigen::MatrixXd left	= Laplacian * Vertices;

	UpdateNormals();
	VertexCurvature.resize(NVertices());
	VertexCurvature.setZero();

	for (int i = 0; i < NVertices(); i++)
	{
		Eigen::Vector3d n = getVertexNormal(i);
		Eigen::Vector3d v_row(left(i, 0), left(i, 1), left(i, 2));
		if (n.dot(v_row) > 0)
			VertexCurvature[i] = 0.5*v_row.norm();
		else
			VertexCurvature(i) = -0.5* v_row.norm();
	}
	std::ofstream test("test.txt");
	test << VertexCurvature;
	test.close();
}

void MyMesh::UpdateGaussianCurvature()
{
	VertexCurvature.resize(NVertices());

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
		VertexCurvature[i]=	v_angledefect[i] / LAR[i];
	}
}

void MyMesh::MakeNoise()
{
	double e_length = 0;
	for (int i = 0; i < NEdges(); i++)
	{
		e_length += getEdgeLength(i);
	}
	e_length /= NEdges();
	//平均边长
	std::default_random_engine generator;
	std::normal_distribution<double> d(e_length);
	for (int i = 0; i < NVertices(); i++)
	{
//		double r = *d(generator);
		double rx = d(generator);
		double ry = d(generator);
		double rz = d(generator);
		Eigen::Vector3d n(rx, ry, rz);
//		SetVerticeNewCoord(i,(getVertexCoord(i) + r * getVertexNormal(i)));
		SetVerticeNewCoord(i, (getVertexCoord(i) +  0.5*n/n.norm() * e_length));
	}
}

void MyMesh::Fairing(int power)
{
	//算拉普拉斯矩阵
	ComputeLaplacian(0);
	//寻找边界上的点
	Eigen::MatrixXd L = Laplacian;
	Eigen::MatrixXd b(NVertices(), 3);
	b.setZero();
	std::ofstream res3("result.txt");
	res3 << L << std::endl;
	res3.close();

	std::ofstream res2("result1.txt");
	res2 << L.inverse() << std::endl;
	res2.close();
	for (int i = 0; i < NVertices(); i++)
	{
		if (isBoundaryVertex(i))
		{
			for (int j = 0; j < NVertices(); j++)
			{
				Laplacian(i, j) = 0;
			}
			Laplacian(i, i) = 1;
			b(i, 0) = Vertices(i, 0);
			b(i, 1) = Vertices(i, 1);
			b(i, 2) = Vertices(i, 2);
		}
	}
	Eigen::MatrixXd res = L.inverse() * b;
	for (int i = 0; i < NVertices(); i++)
	{
		Eigen::Vector3d value(res(i, 0), res(i, 1), res(i, 2));
		SetVerticeNewCoord(i, value);
	}
	std::cout << 123 << std::endl;
}

void MyMesh::getVCurvature(std::vector<double>& c)
{
	c.resize(NVertices());
	for (int i = 0; i < VertexCurvature.size(); i++)
	{
		c[i] = VertexCurvature[i];
	}
}
