#include "MyMesh.h"
#include <random>
#include <fstream>
//#ifdef LINSYSSOLVER_USE_EIGEN
//#include "../LinSysSolver/EigenLinSolver.h"
//#endif // LINSYSSOLVER_USE_EIGEN

void MyMesh::ComputeLAR()//计算顶点的局部平均区域
{
	if (LAR_latest)
		return;

	LAR.resize(NVertices());	
	LAR.setZero();
	for (int i = 0; i < NFaces(); i++)
	{
		Eigen::Vector3d center;
		int v1, v2, v3;
		getFaceVertices(i, v1, v2, v3);
		Eigen::Vector3d p1 = getPoint(v1), p2 = getPoint(v2), p3 = getPoint(v3);
		Eigen::Vector3d m12 = (p1 + p2) / 2, m13 = (p1 + p3) / 2, m23 = (p2 + p3) / 2;
		if (LAR_kind == 0)//重心坐标
		{
			center = ((p1 + p2 + p3) / 3);
		}
		else if (LAR_kind == 1)//Voronoi
		{
			Eigen::Vector3d f_normal = getFaceNormal(i);
			center = ComputeTriangleCirumcenter(p1, p2, p3);
		}
		else if (LAR_kind == 2) //Mixed
		{//
			Eigen::Vector3d f_normal = getFaceNormal(i);
			double angle1, angle2, angle3;
			getFaceAngles(i, angle1, angle2, angle3);
			{
				if (angle1 > 0.5 * M_PI)
					center = m23;
				else if (angle2 > 0.5 * M_PI)
					center = m13;
				else if (angle3 > 0.5 * M_PI)
					center = m12;
				else
					center = ComputeTriangleCirumcenter(p1, p2, p3);
			}
		}
		LAR[v1] += (ComputeTriangleArea(p1, m12, center) + ComputeTriangleArea(p1, m13, center));
		LAR[v2] += (ComputeTriangleArea(p2, m12, center) + ComputeTriangleArea(p2, m23, center));
		LAR[v3] += (ComputeTriangleArea(p3, m13, center) + ComputeTriangleArea(p3, m23, center));
	}
	LAR_latest = true;
}

Eigen::Vector3d MyMesh::ComputeTriangleCirumcenter(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3)
{//处理三点共线的情况
	Eigen::Vector3d e12 = p2 - p1;
	Eigen::Vector3d e13 = p3 - p1;
	if (e12.x() * e13.y() - e13.x() * e12.y() == 0)
	{
		std::cerr << "三点共线发生" << std::endl;
		return (p1 + p2 + p3) / 3;//如果共线则返回三点的重心
	}
	Eigen::Vector3d normal =e12.cross(e13);
 	Eigen::Vector3d dir = normal.cross(e12);
	dir = dir / dir.norm();
	Eigen::Vector3d m12 = (p1 + p2) / 2;
	Eigen::Vector3d m23 = (p2 + p3) / 2;
	Eigen::Vector3d m13 = (p1 + p3) / 2;
	double kup1 = (m12.x() - p1.x()) * (m12.x() - p1.x()) + (m12.y() - p1.y()) * (m12.y() - p1.y()) + (m12.z() - p1.z()) * (m12.z() - p1.z());
	double kup2= (m12.x() - p3.x()) * (m12.x() - p3.x()) + (m12.y() - p3.y()) * (m12.y() - p3.y()) + (m12.z() - p3.z()) * (m12.z() - p3.z());
	double kdown1 = (m12.x() - p1.x()) * dir.x() + (m12.y() - p1.y()) * dir.y() + (m12.z() - p1.z()) * dir.z();
	double kdown2 = (m12.x() - p3.x()) * dir.x() + (m12.y() - p3.y()) * dir.y() + (m12.z() - p3.z()) * dir.z();

	double k = (kup1 - kup2) / (2 * (kdown2 - kdown1));
	Eigen::Vector3d center = m12 + k * dir;
	return center;
}

double MyMesh::ComputeTriangleArea(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3)
{
	Eigen::Vector3d e12 = (p2 - p1);
	Eigen::Vector3d e13 = (p3 - p1);
	double area = 0.5 * (e12.cross(e13)).norm();
	return area;
}

void MyMesh::ComputeCurvature()
{
	if (Curvature_latest)//如果曲率没有变动则直接返回
		return;
	CheckProperty(PROPERTY::P_CURVATURE);
	Curvature.resize(NVertices());
	switch (Curvature_kind)
	{
	case MEAN://平均曲率
	{
		ComputeLaplacian(1);//必须是Cotangent的laplace
		LoadVertex();
		LoadVerticeNormal();
		Eigen::MatrixXd left = Laplacian.toDense() * Vertices;		
		for (int i = 0; i < NVertices(); i++)
		{
			Eigen::Vector3d n = Eigen::Vector3d(VertexNormal(i, 0), VertexNormal(i, 1), VertexNormal(i, 2));
			Eigen::Vector3d v_row = Eigen::Vector3d(left(i, 0), left(i, 1), left(i, 2));
			if (n.dot(v_row) > 0)
				Curvature[i] = 0.5 * v_row.norm();
			else
				Curvature[i] = -0.5 * v_row.norm();
		}
		break;
	}
	case 1:
	{
		ComputeLaplacian(1);//必须是Cotangent的laplace
		LoadVertex();
		Eigen::MatrixXd left = Laplacian.toDense() * Vertices;
		for (int i = 0; i < NVertices(); i++)
		{
			Eigen::Vector3d v_row = Eigen::Vector3d(left(i, 0), left(i, 1), left(i, 2));
				Curvature[i] = abs(0.5 * v_row.norm());
		}
		break;
	}
	case 2:
	{
		ComputeLAR();
		std::vector<double> v_angledefect(NVertices(), 2 * M_PI);
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
			Curvature[i] = v_angledefect[i] / LAR[i];
		break;
	}
	default:
		break;
	}
	Curvature_latest = true;//仅有曲率被修改
	eigen_output();
}

void MyMesh::VertexModified()
{
	LAR_latest = false;
	Vertices_latest = false;
	VertexNormal_latest = false;
	FaceNormal_latest = false;
	Curvature_latest = false;
	if (Laplacian_kind == 1)//如果Laplace是cotangent的，那么也要更新
		Laplacian_latest = false;
}


void MyMesh::MakeNoise()
{
	LoadVertex();
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
		double rx = d(generator);
		double ry = d(generator);
		double rz = d(generator);
		Eigen::Vector3d n(rx, ry, rz);
		Vertices(i, 0) += 0.5 * n.x() / n.norm() * e_length;
		Vertices(i, 1) += 0.5 * n.y() / n.norm() * e_length;
		Vertices(i, 2) += 0.5 * n.z() / n.norm() * e_length;
	}
	VertexModified();
}

void MyMesh::Fairing(int power)
{
	//算拉普拉斯矩阵
	ComputeLaplacian();
	LoadVertex();
	//寻找边界上的点
	Eigen::MatrixXd L =Laplacian.toDense();
	Eigen::MatrixXd b(NVertices(), 3);
	b.setZero();

	for (int i = 0; i < NVertices(); i++)
	{
		if (isBoundaryVertex(i))
		{
			for (int j = 0; j < NVertices(); j++)
			{
				L(i, j) = 0;
			}
			L(i, i) = 1;
			b(i, 0) = Vertices(i, 0);
			b(i, 1) = Vertices(i, 1);
			b(i, 2) = Vertices(i, 2);
		}
	}
	Eigen::SparseMatrix<double> LL = L.sparseView();
	//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.compute(LL);
	Eigen::MatrixXd res = solver.solve(b);

	for (int i = 0; i < NVertices(); i++)
	{
		Eigen::Vector3d value(res(i, 0), res(i, 1), res(i, 2));
		SetVertexNewCoord(i, value);
	}
	VertexModified();
}

void MyMesh::Smoothing(int integrationkind)
{
	ComputeLaplacian();//更新拉普拉斯
	LoadVertex();
	double steplength = 0.5;
	Eigen::MatrixXd delta_Vertex = Laplacian.toDense() * Vertices;
	for (int i = 0; i < NVertices(); i++)
	{
		Vertices(i, 0) = Vertices(i, 0) + steplength * delta_Vertex(i, 0);
		Vertices(i, 1) = Vertices(i, 1) + steplength * delta_Vertex(i, 1);
		Vertices(i, 2) = Vertices(i, 2) + steplength * delta_Vertex(i, 2);
	}
	SetVerticesNewCoord();
	VertexModified();
}

void MyMesh::getVCurvature(std::vector<double>& c)
{
	c.resize(NVertices());

	for (int i = 0; i < Curvature.size(); i++)
	{
		c[i] = Curvature[i];
	}
}

void MyMesh::CheckProperty(PROPERTY p)
{
	switch (p)
	{
	case MyMesh::P_LAR:
		break;
	case MyMesh::P_VERTICES:
		break;
	case MyMesh::P_V_NORMAL:
		break;
	case MyMesh::P_F_NORMAL:
		break;
	case MyMesh::P_CURVATURE:
	{
		switch (Curvature_kind)
		{
		case MyMesh::MEAN:
		{//在求平均曲率的时候，如果laplacian不是contangent的，那么就需要重新求laplacian
			if (Laplacian_kind != LAPLACIAN_KIND::CONTANGENT)
			{
				Laplacian_latest = false;
				Laplacian_kind = LAPLACIAN_KIND::CONTANGENT;
				ComputeLaplacian();
			}
		}
			break;
		case MyMesh::ABSOLUTEMEAN:
			break;
		case MyMesh::GAUSSIAN:
			break;
		case MyMesh::CURVATURE_ND:
			break;
		default:
			break;
		}
	}
		break;
	case MyMesh::P_LAPLACIAN:
	{//检测laplacian
		switch (Laplacian_kind)
		{
		case MyMesh::UNIFORM:
			break;
		case MyMesh::CONTANGENT:
		{//在算cotangent拉普拉斯的时候，需要检测

		}
			break;
		case MyMesh::LAPLACIAN_ND:
			break;
		default:
			break;
		}
	}
		break;
	case MyMesh::P_M_VOLUME:
		break;
	default:
		break;
	}
}

void MyMesh::eigen_output()
{
	std::ofstream laplacian_f("output/laplacian.txt");
	laplacian_f << Laplacian;
	laplacian_f.close();

	std::ofstream LAR_f("output/LAR.txt");
	LAR_f << LAR;
	LAR_f.close();

	std::ofstream Curvature_f("output/Curvature.txt");
	Curvature_f << Curvature;
	Curvature_f.close();

}

MyMesh::MyMesh()
{
//	solver = new EigenLinSolver();
}

