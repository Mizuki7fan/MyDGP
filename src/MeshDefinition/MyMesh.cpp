#include "MyMesh.h"
#include <random>
#include <fstream>
//#ifdef LINSYSSOLVER_USE_EIGEN
//#include "../LinSysSolver/EigenLinSolver.h"
//#endif // LINSYSSOLVER_USE_EIGEN

void MyMesh::ComputeLAR()//计算顶点的局部平均区域
{	
	if (!Vertices_latest)
		LoadVertex();//如果顶点位置不是最新的，那么要更新顶点
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
	Curvature_latest = false;
	Laplacian_latest = false;
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
	switch (Curvature_kind)
	{
	case MEAN://平均曲率
	{
		if (Laplacian_kind != LAPLACIAN_KIND::CONTANGENT || !Laplacian_latest)
		{
			Laplacian_kind = LAPLACIAN_KIND::CONTANGENT;
			Laplacian_latest = false;
			ComputeLaplacian();
		}
		if (!Vertices_latest)
			LoadVertex();
		if (!VertexNormal_latest)
			LoadVerticeNormal();//需要知道点法向以确定曲率值的正负
		if (Curvature_latest)
			return;
		Curvature.resize(NVertices());
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
		Curvature_kind = CURVATURE_KIND::MEAN;
		Curvature_latest = true;
		break;
	}
	case ABSOLUTEMEAN://绝对平均曲率
	{
		if (Laplacian_kind != LAPLACIAN_KIND::CONTANGENT || !Laplacian_latest)
		{
			Laplacian_kind = LAPLACIAN_KIND::CONTANGENT;
			Laplacian_latest = false;
			ComputeLaplacian();
		}
		if (!Vertices_latest)
			LoadVertex();
		if (Curvature_latest)
			return;
		Curvature.resize(NVertices());
		Eigen::MatrixXd left = Laplacian.toDense() * Vertices;
		for (int i = 0; i < NVertices(); i++)
		{
			Eigen::Vector3d v_row = Eigen::Vector3d(left(i, 0), left(i, 1), left(i, 2));
				Curvature[i] = abs(0.5 * v_row.norm());
		}
		Curvature_kind = CURVATURE_KIND::ABSOLUTEMEAN;
		Curvature_latest = true;
		break;
	}
	case CURVATURE_KIND::GAUSSIAN:
	{
		if (!LAR_latest)
			ComputeLAR();
		if (Curvature_latest)
			return;
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
	SetVerticesNewCoord();
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
	Eigen::SparseMatrix<double> LL =L.sparseView();
	//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.compute(LL);
	Eigen::MatrixXd res = solver.solve(b);

	for (int i = 0; i < NVertices(); i++)
	{
		Eigen::Vector3d value(res(i, 0), res(i, 1), res(i, 2));
		SetVertexNewCoord(i, value);
	}
}

void MyMesh::Smoothing()
{	
	if (!Vertices_latest)
		LoadVertex();
	if (!Laplacian_latest)
		ComputeLaplacian();//更新拉普拉斯
	double ori_vol = ComputeMeshVolume();
	switch (Euler_integration_kind)
	{
	case MyMesh::EXPLICIT://显式
	{
		double steplength = 0.5;
		Eigen::MatrixXd delta_Vertex = Laplacian.toDense() * Vertices;
		for (int i = 0; i < NVertices(); i++)
		{
			Vertices(i, 0) = Vertices(i, 0) + steplength * delta_Vertex(i, 0);
			Vertices(i, 1) = Vertices(i, 1) + steplength * delta_Vertex(i, 1);
			Vertices(i, 2) = Vertices(i, 2) + steplength * delta_Vertex(i, 2);
		}
	}
		break;
	case MyMesh::IMPLICIT://隐式
	{
		double steplength = 0.5;
		int nv = NVertices();
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nv,nv);//单位矩阵
		Eigen::MatrixXd A=steplength* Laplacian.toDense();
		Eigen::SparseMatrix<double> LL = (I-A).sparseView();
		Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
		solver.compute(LL);
		Eigen::MatrixXd res = solver.solve(Vertices);
		for (int i = 0; i < NVertices(); i++)
		{
			Vertices(i, 0) = Vertices(i, 0) + res(i, 0);
			Vertices(i, 1) = Vertices(i, 1) + res(i, 1);
			Vertices(i, 2) = Vertices(i, 2) + res(i, 2);
		}
		SetVerticesNewCoord();
		double new_vol = ComputeMeshVolume();
		double scale = pow(new_vol / ori_vol, 1.0 / 3);
		for (int i = 0; i < NVertices(); i++)
		{
			Vertices(i, 0) = Vertices(i, 0)/scale;
			Vertices(i, 1) = Vertices(i, 1) /scale;
			Vertices(i, 2) = Vertices(i, 2) /scale;
		}
	}
		break;
	case MyMesh::EULER_INTEGRATION_ND:
		break;
	default:
		break;
	}
	SetVerticesNewCoord();
}

void MyMesh::BilateralDenoising(double stdevs,double stdevr)
{
	if (!Vertices_latest)
		LoadVertex();
	if (!VertexNormal_latest)
		LoadVerticeNormal();
	Eigen::VectorXd D;
	D.resize(NVertices());//顶点要沿着其法向进行偏移的距离
	for (int i = 0; i < NVertices(); i++)
	{//遍历所有顶点，算这个顶点的法向平面
		Eigen::Vector3d N = getVertexNormal(i);
		Eigen::Vector3d V = getVertexCoord(i);
		//找这个点的邻域点
		std::vector<int> n_nei;
		getVerticesNeighbour(i, n_nei);
		double d_total = 0;
		double Kv = 0;
		for (int j = 0; j < n_nei.size(); j++)
		{
			Eigen::Vector3d Q = getVertexCoord(n_nei[j]);
			double t = (Q - V).norm();
			double d = (N.dot(Q - V)) / N.norm();
			double Ws = exp(-t * t / (2 * stdevs));
			double Wr = exp(-d * d / (2 * stdevr));
			d_total += Ws * Wr * d;
			Kv += Ws * Wr;
		}
		D[i] = d_total / Kv;
	}
	for (int i = 0; i < NVertices(); i++)
	{
		Vertices(i) = Vertices(i) + VertexNormal(i) * D[i];
	}
	SetVerticesNewCoord();
}

void MyMesh::BilateralNormalFiltering(double stdevs, double stdevr)
{
	if (!Vertices_latest)
		LoadVertex();
	if (!FaceNormal_latest)
		LoadFaceNormal();
	Eigen::MatrixXd NewNormal(NFaces(),3);
//	NewNormal.setZero();
	for (int i = 0; i < NFaces(); i++)
	{//对所有边进行遍历
		double newNormalX = 0, newNormalY = 0, newNormalZ = 0;//新法向的三个值
		std::vector<int> f_nei;
		//算面的中心点
		int vi1, vi2, vi3;
		getFaceVertices(i, vi1, vi2, vi3);
		Eigen::Vector3d fi_center = (getPoint(vi1) + getPoint(vi2) + getPoint(vi3)) / 3;
		Eigen::Vector3d i_normal = getFaceNormal(i);
		getFaceNeighbour(i, f_nei);
		//std::cout << f_nei.size() << std::endl;
		double Kp = 0;
		for (int j = 0; j < f_nei.size(); j++)
		{
			int vj1, vj2, vj3;
			getFaceVertices(f_nei[j], vj1, vj2, vj3);//获取面的三个点
			Eigen::Vector3d p1 = getPoint(vj1), p2 = getPoint(vj2), p3 = getPoint(vj3);
			Eigen::Vector3d fj_center = (p1 + p2 + p3) / 3;//获取面的中心
			double delta_center = (fj_center - fi_center).norm();//中心之间的距离
			Eigen::Vector3d j_normal = getFaceNormal(f_nei[j]);//获取面的法向
			double delta_normal = (j_normal - i_normal).norm();//获取法向的差异
			double Aj = ComputeTriangleArea(p1, p2, p3);
			Kp += Aj;
			double Ws = exp(-delta_center * delta_center / (2 * stdevs));
			double Wr = exp(-delta_normal * delta_normal / (2 * stdevr));
			newNormalX += Aj * Ws * Wr * j_normal.x();
			newNormalY += Aj * Ws * Wr * j_normal.y();
			newNormalZ += Aj * Ws * Wr * j_normal.z();
		}
		newNormalX /= Kp;
		newNormalY /= Kp;
		newNormalZ /= Kp;
		double norm = sqrt(newNormalX * newNormalX + newNormalY * newNormalY + newNormalZ * newNormalZ);
		NewNormal(i, 0) = newNormalX/norm;
		NewNormal(i, 1) = newNormalY/norm;
		NewNormal(i, 2) = newNormalZ/norm;
	}
	std::ofstream ori_normal("output//ori_normal.txt");
	ori_normal << FaceNormal;
	ori_normal.close();
	std::ofstream new_normal("output//new_normal.txt");
	new_normal << NewNormal;
	new_normal.close();
	FaceNormal = NewNormal;
	SetFacesNewNormalCoord();

}

void MyMesh::getVCurvature(std::vector<double>& c)
{
	c.resize(NVertices());

	for (int i = 0; i < Curvature.size(); i++)
	{
		c[i] = Curvature[i];
	}
}

void MyMesh::SetLaplacianKind(LAPLACIAN_KIND k)
{
	if (k != Laplacian_kind)
	{
		Laplacian_kind = k;
		Laplacian_latest = false;
	}
}

void MyMesh::SetLARKind(LAR_KIND k)
{
	if (k != LAR_kind)
	{
		LAR_kind = k;
		LAR_latest = false;
		Laplacian_latest = false;//因为修改后可能会影响laplace的计算
	}
}

void MyMesh::SetCurvatureKind(CURVATURE_KIND k)
{
	if (k != Curvature_kind)
	{
		Curvature_kind = k;
		Curvature_latest = false;
	}
}

void MyMesh::SetEulerIntegrationKind(EULER_INTEGRATION_KIND k)
{
	if (k != Euler_integration_kind)
	{
		Euler_integration_kind = k;
		Euler_integration_latest = false;
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

