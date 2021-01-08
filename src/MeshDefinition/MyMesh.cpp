#include "MyMesh.h"
#include <random>
#include <fstream>

void MyMesh::ComputeLaplacian()
{
	int nv = mesh.n_vertices();
	Laplacian.resize(nv, nv);
	std::vector<Eigen::Triplet<double>> L;
	switch (Laplacian_kind)
	{
	case MyMesh::UNIFORM:
	{
		for (T::VertexHandle vh : mesh.vertices())
		{
			int idx = vh.idx();
			int n_range = 0;
			for (auto vv : mesh.vv_range(vh))
				n_range++;
			L.push_back(Eigen::Triplet<double>(idx, idx, -1));
			for (auto vv : mesh.vv_range(vh))
			{
				L.push_back(Eigen::Triplet<double>(idx, vv.idx(), 1.0 / n_range));
			}
		}
		break;
	}
	case MyMesh::CONTANGENT:
	{//算cotangent的拉普拉斯，需要知道最新的LAR
		ComputeLAR();
		for (T::VertexHandle vh : mesh.vertices())
			L.push_back(Eigen::Triplet<double>(vh.idx(), vh.idx(), -1 / (2 * LAR(vh.idx()))));
		for (T::FaceHandle fh : mesh.faces())
		{
			int idx = fh.idx();
			std::vector<int> fv;
			for (auto fvi : mesh.fv_range(fh))
			{
				fv.push_back(fvi.idx());
			}
			OpenMesh::Vec3d e12 = mesh.point(mesh.vertex_handle(fv[1])) - mesh.point(mesh.vertex_handle(fv[0]));
			OpenMesh::Vec3d e13 = mesh.point(mesh.vertex_handle(fv[2])) - mesh.point(mesh.vertex_handle(fv[0]));
			OpenMesh::Vec3d e23 = mesh.point(mesh.vertex_handle(fv[2])) - mesh.point(mesh.vertex_handle(fv[1]));
			e12 = e12.normalize();
			e13 = e13.normalize();
			e23 = e23.normalize();
			double angle1 = acos(e12 | e13);
			double angle2 = acos(-e12 | e23);
			double angle3 = acos(e13 | e23);

			L.push_back(Eigen::Triplet<double>(fv[0], fv[1], 1 / tan(angle3) / (2 * LAR[fv[0]])));
			L.push_back(Eigen::Triplet<double>(fv[0], fv[0], -1 / tan(angle3) / (2 * LAR[fv[0]])));
			L.push_back(Eigen::Triplet<double>(fv[1], fv[0], 1 / tan(angle3) / (2 * LAR[fv[1]])));
			L.push_back(Eigen::Triplet<double>(fv[1], fv[1], -1 / tan(angle3) / (2 * LAR[fv[1]])));

			L.push_back(Eigen::Triplet<double>(fv[0], fv[2], 1 / tan(angle2) / (2 * LAR[fv[0]])));
			L.push_back(Eigen::Triplet<double>(fv[0], fv[0], -1 / tan(angle2) / (2 * LAR[fv[0]])));
			L.push_back(Eigen::Triplet<double>(fv[2], fv[0], 1 / tan(angle2) / (2 * LAR[fv[2]])));
			L.push_back(Eigen::Triplet<double>(fv[2], fv[2], -1 / tan(angle2) / (2 * LAR[fv[2]])));


			L.push_back(Eigen::Triplet<double>(fv[1], fv[2], 1 / tan(angle1) / (2 * LAR[fv[1]])));
			L.push_back(Eigen::Triplet<double>(fv[1], fv[1], -1 / tan(angle1) / (2 * LAR[fv[1]])));
			L.push_back(Eigen::Triplet<double>(fv[2], fv[1], 1 / tan(angle1) / (2 * LAR[fv[2]])));
			L.push_back(Eigen::Triplet<double>(fv[2], fv[2], -1 / tan(angle1) / (2 * LAR[fv[2]])));
		}
		break;
	}
	case MyMesh::LAPLACIAN_ND:
		break;
	default:
		break;
	}
	Laplacian.setFromTriplets(L.begin(), L.end());
	Laplacian.makeCompressed();
}

void MyMesh::ComputeLAR()//计算顶点的局部平均区域
{
	std::cout << "计算了LAR" << std::endl;
	LAR.resize(mesh.n_vertices());
	LAR.setZero();
	for (T::FaceHandle fh : mesh.faces())
	{
		T::Point center;
		std::vector<T::VertexHandle> f_v;
		for (T::VertexHandle fv : mesh.fv_range(fh))
			f_v.push_back(fv);//取面的3个点
		T::Point p0 = mesh.point(f_v[0]), p1 = mesh.point(f_v[1]), p2 = mesh.point(f_v[2]);
		OpenMesh::Vec3d m01 = (p0 + p1) / 2, m02 = (p0 + p2) / 2, m12 = (p1 + p2) / 2;
		switch (LAR_kind)
		{
		case MyMesh::BARYCENTRIC:
			center = ((p0 + p1 + p2) / 3);
			break;
		case MyMesh::VORONOI:
			center = CalcTriangleCirumcenter(p0, p1, p2);
			break;
		case MyMesh::MIXED:
			double angle1, angle2, angle3;
			CalcTriangleAngles(p0, p1, p2, angle1, angle2, angle3);
			{
				if (angle1 > 0.5 * M_PI)
					center = m12;
				else if (angle2 > 0.5 * M_PI)
					center = m02;
				else if (angle3 > 0.5 * M_PI)
					center = m01;
				else
					center = CalcTriangleCirumcenter(p0, p1, p2);
			}
			break;
		case MyMesh::LAR_ND:
			std::cerr << "LAR类型未设定！" << std::endl;
			break;
		default:
			break;
		}
		LAR[f_v[0].idx()] += (CalcTriangleArea(p0, m01, center) + CalcTriangleArea(p0, m02, center));
		LAR[f_v[1].idx()] += (CalcTriangleArea(p1, m01, center) + CalcTriangleArea(p1, m12, center));
		LAR[f_v[2].idx()] += (CalcTriangleArea(p2, m02, center) + CalcTriangleArea(p2, m12, center));
	}
}

void MyMesh::CalcCurvature()
{
	Curvature.resize(mesh.n_vertices());
	switch (Curvature_kind)
	{
	case MEAN://平均曲率，需要用到Laplacian
	{
		LoadVertex();
		Laplacian_kind = LAPLACIAN_KIND::CONTANGENT;
		ComputeLaplacian();
		Eigen::MatrixXd left = Laplacian.toDense() * Vertices;
		for (T::VertexHandle vh : mesh.vertices())
		{
			int idx = vh.idx();
			T::Normal n = mesh.normal(vh);

			OpenMesh::Vec3d v_row(left(idx, 0), left(idx, 1), left(idx, 2));
			if ((n | v_row) > 0)
				Curvature[idx] = 0.5 * v_row.norm();
			else
				Curvature[idx] = -0.5 * v_row.norm();
		}
		break;
	}
	case ABSOLUTEMEAN://绝对平均曲率
	{
		Laplacian_kind = LAPLACIAN_KIND::CONTANGENT;
		ComputeLaplacian();
		Eigen::MatrixXd left = Laplacian.toDense() * Vertices;
		for (T::VertexHandle vh : mesh.vertices())
		{
			int idx = vh.idx();
			Eigen::Vector3d v_row = Eigen::Vector3d(left(idx, 0), left(idx, 1), left(idx, 2));
			Curvature[idx] = abs(0.5 * v_row.norm());
		}
		break;
	}
	case CURVATURE_KIND::GAUSSIAN:
	{
		ComputeLAR();
		std::vector<double> v_angledefect(mesh.n_vertices(), 2 * M_PI);
		for (T::FaceHandle fh : mesh.faces())
		{
			std::vector<T::VertexHandle> f_v;
			for (T::VertexHandle fv : mesh.fv_range(fh))
				f_v.push_back(fv);//取面的3个点
			double angle0 = 0, angle1 = 0, angle2 = 0;
			T::Point p0 = mesh.point(f_v[0]), p1 = mesh.point(f_v[1]), p2 = mesh.point(f_v[2]);
			CalcTriangleAngles(p0, p1, p2, angle0, angle1, angle2);
			v_angledefect[f_v[0].idx()] -= angle0;
			v_angledefect[f_v[1].idx()] -= angle1;
			v_angledefect[f_v[2].idx()] -= angle2;
		}
		for (T::VertexHandle vh : mesh.vertices())
			Curvature[vh.idx()] = v_angledefect[vh.idx()] / LAR[vh.idx()];
		break;
	}
	default:
		break;
	}
}

void MyMesh::MakeNoise()
{
	double e_length = 0;
	for (T::EdgeHandle eh : mesh.edges())
		e_length += mesh.calc_edge_length(eh);
	e_length /= mesh.n_edges();
	std::default_random_engine generator;
	std::normal_distribution<double> d(e_length);
	for (T::VertexHandle vh : mesh.vertices())
	{
		int idx = vh.idx();
		double rx = d(generator);
		double ry = d(generator);
		double rz = d(generator);
		OpenMesh::Vec3d n(rx, ry, rz);
		n = n / n.norm() * e_length;
		T::Point p = mesh.point(vh);
		T::Point newp = p + 0.5 * n;
		mesh.set_point(vh, newp);
	}
}

void MyMesh::Fairing(int power)
{
	//算拉普拉斯矩阵
	ComputeLaplacian();
	//寻找边界上的点
	Eigen::MatrixXd L = Laplacian.toDense();
	Eigen::MatrixXd b(mesh.n_vertices(), 3);
	b.setZero();
	for (T::VertexHandle vh : mesh.vertices())
	{
		int idx = vh.idx();
		if (mesh.is_boundary(vh))
		{
			for (int j = 0; j < mesh.n_vertices(); j++)
			{
				L(idx, j) = 0;
			}
			L(idx, idx) = 1;
			T::Point p = mesh.point(vh);
			b(idx, 0) = p.data()[0];
			b(idx, 1) = p.data()[1];
			b(idx, 2) = p.data()[2];
		}
	}
	Eigen::SparseMatrix<double> LL = L.sparseView();
	//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.compute(LL);
	Eigen::MatrixXd res = solver.solve(b);

	for (T::VertexHandle vh : mesh.vertices())
	{
		int idx = vh.idx();
		T::Point p(res(idx, 0), res(idx, 1), res(idx, 2));
		mesh.set_point(vh, p);
	}
}

void MyMesh::Smoothing()
{
	LoadVertex();
	ComputeLaplacian();//更新拉普拉斯
	double ori_vol = ComputeMeshVolume();
	switch (Euler_integration_kind)
	{
	case MyMesh::EXPLICIT://显式
	{
		double steplength = 0.5;
		Eigen::MatrixXd delta_Vertex = Laplacian.toDense() * Vertices;

		for (int i = 0; i < mesh.n_vertices(); i++)
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
		int nv = mesh.n_vertices();
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nv, nv);//单位矩阵
		Eigen::MatrixXd A = steplength * Laplacian.toDense();
		Eigen::SparseMatrix<double> LL = (I - A).sparseView();
		Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
		solver.compute(LL);
		Eigen::MatrixXd res = solver.solve(Vertices);
		for (int i = 0; i < mesh.n_vertices(); i++)
		{
			Vertices(i, 0) = Vertices(i, 0) + res(i, 0);
			Vertices(i, 1) = Vertices(i, 1) + res(i, 1);
			Vertices(i, 2) = Vertices(i, 2) + res(i, 2);
		}
		SaveVertex();
		double new_vol = ComputeMeshVolume();
		double scale = pow(new_vol / ori_vol, 1.0 / 3);
		for (int i = 0; i < mesh.n_vertices(); i++)
		{
			Vertices(i, 0) = Vertices(i, 0) / scale;
			Vertices(i, 1) = Vertices(i, 1) / scale;
			Vertices(i, 2) = Vertices(i, 2) / scale;
		}
	}
	break;
	case MyMesh::EULER_INTEGRATION_ND:
		break;
	default:
		break;
	}
	SaveVertex();
}

bool MyMesh::Load(std::string s)
{
	return OpenMesh::IO::read_mesh(mesh, s);
}

bool MyMesh::Write(std::string s)
{
	return OpenMesh::IO::write_mesh(mesh, s);
}

void MyMesh::LoadVertex()
{
	Vertices.resize(mesh.n_vertices(), 3);
	for (auto a : mesh.vertices())
	{
		T::Point p = mesh.point(a);
		Vertices(a.idx(), 0) = p.data()[0];
		Vertices(a.idx(), 1) = p.data()[1];
		Vertices(a.idx(), 2) = p.data()[2];
	}
}

void MyMesh::SaveVertex()
{
	for (auto vh : mesh.vertices())
	{
		int idx = vh.idx();
		T::Point p(Vertices(vh.idx(), 0), Vertices(vh.idx(), 1), Vertices(vh.idx(), 2));
		mesh.set_point(vh, p);
	}
}

void MyMesh::Clear()
{
	mesh.clean();
}

void MyMesh::UpdateNormals()
{
	mesh.update_normals();
	mesh.request_face_normals();
	mesh.request_vertex_normals();
}

int MyMesh::NVertices() const
{
	return mesh.n_vertices();
}

int MyMesh::NEdges() const
{
	return mesh.n_edges();
}

int MyMesh::NFaces() const
{
	return mesh.n_faces();
}

double MyMesh::CalcEdgeLength(int i)
{
	return mesh.calc_edge_length(mesh.edge_handle(i));
}

T::Normal MyMesh::getVertexNormal(int i) const
{
	return mesh.normal(mesh.vertex_handle(i));
}

T::Normal MyMesh::getFaceNormal(int i) const
{
	return mesh.normal(mesh.face_handle(i));
}

T::Point MyMesh::getPoint(int i) const
{
	return mesh.point(mesh.vertex_handle(i));
}

void MyMesh::getFaceVertices(int f, int& v1, int& v2, int& v3) const
{
	T::FaceHandle fh = mesh.face_handle(f);
	std::vector<int> fv;
	for (auto fvi : mesh.fv_range(fh))
	{
		fv.push_back(fvi.idx());
	}
	v1 = fv[0]; v2 = fv[1]; v3 = fv[2];
}

void MyMesh::getEdgeVertices(int e, int& v1, int& v2) const
{
	T::HalfedgeHandle he = mesh.halfedge_handle(mesh.edge_handle(e), 0);
	T::VertexHandle vi = mesh.to_vertex_handle(he);
	T::VertexHandle vj = mesh.from_vertex_handle(he);
	v1 = vi.idx();
	v2 = vj.idx();
}

double MyMesh::getEdgeLength(int i) const
{
	return mesh.calc_edge_length(mesh.edge_handle(i));
}

bool MyMesh::EdgeIsBoundary(int i) const
{
	return mesh.is_boundary(mesh.edge_handle(i));
}

void MyMesh::BilateralDenoising(double stdevs, double stdevr)
{
	Eigen::VectorXd D;
	D.resize(mesh.n_vertices());
	mesh.update_normals();
	for (T::VertexHandle vh : mesh.vertices())
	{
		T::Normal N = mesh.normal(vh);
		T::Point V = mesh.point(vh);
		double d_total = 0;
		double Kv = 0;
		for (T::VertexHandle vj : mesh.vv_range(vh))
		{
			T::Point Q = mesh.point(vj);
			double t = (Q - V).norm();
			double d = (N | (Q - V)) / N.norm();
			double Ws = exp(-t * t / (2 * stdevs));
			double Wr = exp(-d * d / (2 * stdevr));
			d_total += Ws * Wr * d;
			Kv += Ws * Wr;
		}
		D[vh.idx()] = d_total / Kv;
	}
	for (T::VertexHandle vh : mesh.vertices())
	{
		T::Normal N = mesh.normal(vh);
		T::Point V = mesh.point(vh);
		T::Point newp = V + N * D[vh.idx()];
		mesh.set_point(vh, newp);
	}
}

void MyMesh::BilateralNormalFiltering(double stdevs, double stdevr)
{
	mesh.update_normals();
	std::vector<T::Normal> NewNormal(mesh.n_faces());//记录新的normal
	std::vector<double> FaceArea(mesh.n_faces());
	std::vector<T::Point> FaceCenter(mesh.n_faces());
	double ta = 0;

	for (T::FaceHandle fh : mesh.faces())
	{
		std::vector<T::Point> P;
		for (T::VertexHandle vh : mesh.fv_range(fh))
		{
			P.push_back(mesh.point(vh));
		}
		auto e12 = (P[1] - P[0]);
		auto e13 = (P[2] - P[0]);
		double area = 0.5 * (e12 % e13).norm();
		ta += area;
		FaceArea[fh.idx()] = area;
		NewNormal[fh.idx()] = mesh.normal(fh);
		FaceCenter[fh.idx()] = mesh.calc_face_centroid(fh);
	}
	
	stdevs = 0;
	for (T::FaceHandle fh : mesh.faces())
	{
		//auto centroid = mesh.calc_face_centroid(fh);
		for (T::FaceHandle ffh:mesh.ff_range(fh))
		{
			stdevs += (FaceCenter[fh.idx()] - FaceCenter[ffh.idx()]).norm();
		}
	}
	stdevs /= mesh.n_faces() / 3;

	for (int i = 0; i < 8; i++)
	{
		for (T::FaceHandle fh : mesh.faces())
		{
			double Kp = 0;
			T::Normal NewN(0, 0, 0);
			for (T::FaceHandle nei_fh : mesh.ff_range(fh))
			{
				double delta_center = (FaceCenter[fh.idx()] - FaceCenter[nei_fh.idx()]).norm();
				double delta_normal = (NewNormal[fh.idx()] - NewNormal[nei_fh.idx()]).norm();
				double Aj = FaceArea[nei_fh.idx()];
				double Ws = exp(-delta_center * delta_center / (2 * stdevs * stdevs));
				double Wr = exp(-delta_normal * delta_normal / (2 * stdevr * stdevr));
				NewN += Aj * Ws * Wr * NewNormal[nei_fh.idx()];
				Kp += Aj * Ws * Wr;
			}
			NewNormal[fh.idx()] = NewN/Kp;
			NewNormal[fh.idx()].norm();
		}
	}

	for (int i = 0; i < 8; i++)
	{
		for (T::VertexHandle vh : mesh.vertices())
		{
			T::Point x_i = mesh.point(vh);
			T::Point delta_xi(0, 0, 0);
			int Nei_count = 0;
			for (T::FaceHandle fh : mesh.vf_range(vh))
			{
				Nei_count++;
				T::Point cj = mesh.calc_face_centroid(fh);
				T::Normal nj = NewNormal[fh.idx()];
				delta_xi += nj * (nj.data()[0] * (cj - x_i).data()[0] + nj.data()[1] * (cj - x_i).data()[1] + nj.data()[2] * (cj - x_i).data()[2]);
			}
			x_i =x_i+ delta_xi / Nei_count;
			mesh.set_point(vh, x_i);
		}
	}
}

void MyMesh::CalcTutte()
{
	//先把网格边界点按顺序投影到一个圆上
	std::vector<int> Bnd_v;
	for (T::VertexHandle vh : mesh.vertices())
	{
		if (mesh.is_boundary(vh))
		{
			Bnd_v.push_back(vh.idx());
		}
	}
	if (Bnd_v.size() == 0)
	{
		std::cerr << "网格是封闭的" << std::endl;
		return;
	}
	std::vector<int> Bnd_v_inorder;//有序的网格边界点
	T::VertexHandle ima_bnd_v = mesh.vertex_handle(Bnd_v[0]);
	std::vector<T::VertexHandle> vv_nei;
	for (T::VertexHandle vvh : mesh.vv_range(ima_bnd_v))
		if (mesh.is_boundary(vvh))
			vv_nei.push_back(vvh);
	T::VertexHandle pre_bnd_v = vv_nei[0];
	T::VertexHandle next_bnd_v = vv_nei[1];

	while (ima_bnd_v != pre_bnd_v)
	{
		Bnd_v_inorder.push_back(ima_bnd_v.idx());
		T::VertexHandle nextnext_bnd_v;
		for (T::VertexHandle vvh : mesh.vv_range(next_bnd_v))
		{
			if (mesh.is_boundary(vvh) && vvh != ima_bnd_v)
			{
				nextnext_bnd_v = vvh;
				break;
			}
		}
		ima_bnd_v = next_bnd_v;
		next_bnd_v = nextnext_bnd_v;
	}
	Bnd_v_inorder.push_back(pre_bnd_v.idx());
	if (Bnd_v_inorder.size() != Bnd_v.size())
	{
		std::cerr << "网格并非圆盘拓扑" << std::endl;
	}
	double r = 5.0;
	int bnd_size = Bnd_v.size();
	Eigen::MatrixXd A, b;
	A.resize(mesh.n_vertices(), mesh.n_vertices());
	A.setZero();
	b.resize(mesh.n_vertices(), 3);
	b.setZero();
	for (double idx = 0; idx < bnd_size; idx++)
	{
		int vid = Bnd_v_inorder[idx];
		A(vid, vid) = 1;
		double theta = 2 * M_PI / bnd_size * idx;
		b(vid, 0) = r * cos(theta);
		b(vid, 1) = r * sin(theta);
	}
	for (T::VertexHandle vh : mesh.vertices())
	{
		if (mesh.is_boundary(vh))
			continue;
		//否则求vh的邻域
		int nei_count = 0;
		for (T::VertexHandle vnei : mesh.vv_range(vh))
			nei_count++;
		for (T::VertexHandle vnei : mesh.vv_range(vh))
		{
			A(vh.idx(), vnei.idx()) = 1.0;
		}
		A(vh.idx(), vh.idx()) = -nei_count;//这么写可以获得一个对称矩阵
	}
	Eigen::SparseMatrix<double> LL = A.sparseView();
	//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;要求矩阵正定
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.compute(LL);
	Eigen::MatrixXd res = solver.solve(b);//参数化结果的点
	//Eigen::MatrixXd res = A.inverse() * b;
	for (T::VertexHandle vh : mesh.vertices())
	{
		int idx = vh.idx();
		T::Point p(res(idx, 0), res(idx, 1), res(idx, 2));
		mesh.set_point(vh, p);
	}
}

void MyMesh::CalcLSCM(int method_id)
{
	if (method_id == 1)
	{
		if (mesh.n_vertices() == 0)
			return;
		//有2种方法，先用赵征宇教的，方法1
		Eigen::MatrixXd a(2, 2);
		int nv = mesh.n_vertices();
		a(0, 0) = 0; a(0, 1) = 1; a(1, 0) = -1; a(1, 1) = 0;
		Eigen::SparseMatrix<double> A(2 * mesh.n_faces(), 2 * mesh.n_vertices());
		std::vector<Eigen::Triplet<double>> tripletsL;
		for (T::FaceHandle fh : mesh.faces())
		{
			int idx = fh.idx();
			std::vector<T::VertexHandle> f_v;
			for (T::VertexHandle fv : mesh.fv_range(fh))
			{
				f_v.push_back(fv);
			}
			double x0, y0, x1, y1, x2, y2;
			T::Point p0 = mesh.point(f_v[0]), p1 = mesh.point(f_v[1]), p2 = mesh.point(f_v[2]);
			double Area = CalcTriangleArea(p0, p1, p2);
			x0 = 0; y0 = 0;
			OpenMesh::Vec3d e01 = p1 - p0;
			x1 = e01.length(), y1 = 0;
			OpenMesh::Vec3d e02 = p2 - p0;
			y2 = 2 * Area / e01.length();
			x2 = sqrt(e02.length() * e02.length() - y2 * y2);
			Eigen::MatrixXd Mt(2, 3);
			Mt(0, 0) = y1 - y2; Mt(0, 1) = y2 - y0; Mt(0, 2) = y0 - y1;
			Mt(1, 0) = x2 - x1; Mt(1, 1) = x0 - x2; Mt(1, 2) = x1 - x0;
			Mt = Mt / Area;
			Eigen::MatrixXd Mt_u = a * Mt;
			//u在前，v在后
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx, f_v[0].idx(), Mt_u(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx, f_v[1].idx(), Mt_u(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx, f_v[2].idx(), Mt_u(0, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx, f_v[0].idx() + nv, Mt(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx, f_v[1].idx() + nv, Mt(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx, f_v[2].idx() + nv, Mt(0, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx + 1, f_v[0].idx(), Mt_u(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx + 1, f_v[1].idx(), Mt_u(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx + 1, f_v[2].idx(), Mt_u(1, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx + 1, f_v[0].idx() + nv, Mt(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx + 1, f_v[1].idx() + nv, Mt(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(2 * idx + 1, f_v[2].idx() + nv, Mt(1, 2)));
		}
		A.setFromTriplets(tripletsL.begin(), tripletsL.end());
		//选择2个点进行固定
		Eigen::SparseMatrix<double> M = A.transpose() * A;
		//http://www.suoniao.com/article/22026
		//稀疏转三元组
		tripletsL.clear();
		for (int k = 0; k < M.outerSize(); ++k)
			for (Eigen::SparseMatrix<double>::InnerIterator it(M, k); it; ++it)
			{
				tripletsL.push_back(Eigen::Triplet<double>(it.row(), it.col(), it.value()));
			}
		//固定4个值
		Eigen::SparseMatrix<double> MM(2 * mesh.n_vertices() + 4, 2 * mesh.n_vertices() + 4);

		T::VertexHandle vh1 = mesh.vertex_handle(1);
		int fix1 = vh1.idx();
		double dis = DBL_MIN;
		int max_id = -1;
		for (T::VertexHandle vh : mesh.vertices())
		{
			double d = (mesh.point(vh) - mesh.point(vh1)).length();
			if (d > dis)
			{
				dis = d;
				max_id = vh.idx();
			}
		}
		T::VertexHandle vh2 = mesh.vertex_handle(max_id);
		int fix2 = vh2.idx();
		tripletsL.push_back(Eigen::Triplet<double>(2 * nv, fix1, 1));
		tripletsL.push_back(Eigen::Triplet<double>(fix1, 2 * nv, 1));
		tripletsL.push_back(Eigen::Triplet<double>(2 * nv + 1, fix1 + nv, 1));
		tripletsL.push_back(Eigen::Triplet<double>(fix1 + nv, 2 * nv + 1, 1));
		tripletsL.push_back(Eigen::Triplet<double>(2 * nv + 2, fix2, 1));
		tripletsL.push_back(Eigen::Triplet<double>(fix2, 2 * nv + 2, 1));
		tripletsL.push_back(Eigen::Triplet<double>(2 * nv + 3, fix2 + nv, 1));
		tripletsL.push_back(Eigen::Triplet<double>(fix2 + nv, 2 * nv + 3, 1));
		MM.setFromTriplets(tripletsL.begin(), tripletsL.end());//设置好MM
		Eigen::VectorXd b(2 * nv + 4);
		b(2 * nv + 2) = dis;
		//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;要求矩阵正定
		Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
		solver.compute(MM);
		Eigen::VectorXd res = solver.solve(b);//参数化结果的点
		for (T::VertexHandle vh : mesh.vertices())
		{
			int idx = vh.idx();
			T::Point p(res(idx), res(idx + nv), 0);
			mesh.set_point(vh, p);
		}
		std::cout << "LSCM use method 1" << std::endl;
	}
	else
	{
		int nv = mesh.n_vertices();
		Eigen::SparseMatrix<double> A(2 * nv + 4, 2 * nv + 4);//这个方程存的是能量E关于坐标每个值的偏导数，有2nv个参数且每个参数都有偏导数，所以是2nv*2nv的
		std::vector<Eigen::Triplet<double>> tripletsL;
		A.setZero();
		for (T::FaceHandle fh : mesh.faces())
		{
			int idx = fh.idx();
			std::vector<T::VertexHandle> f_v;
			for (T::VertexHandle fv : mesh.fv_range(fh))
			{
				f_v.push_back(fv);
			}
			double x0, y0, x1, y1, x2, y2;
			T::Point p0 = mesh.point(f_v[0]), p1 = mesh.point(f_v[1]), p2 = mesh.point(f_v[2]);
			double Area = CalcTriangleArea(p0, p1, p2);
			x0 = 0; y0 = 0;
			OpenMesh::Vec3d e01 = p1 - p0;
			x1 = e01.length(), y1 = 0;
			OpenMesh::Vec3d e02 = p2 - p0;
			y2 = 2 * Area / e01.length();
			x2 = sqrt(e02.length() * e02.length() - y2 * y2);
			Eigen::MatrixXd S(2, 3);
			S(0, 0) = y1 - y2; S(0, 1) = y2 - y0; S(0, 2) = y0 - y1;
			S(1, 0) = x2 - x1; S(1, 1) = x0 - x2; S(1, 2) = x1 - x0;
			S = S / (2 * Area);

			// 能量的前一半
			// /partial{E}_{/partial{u_i}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[0].idx(), 2 * S(0, 0) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[1].idx(), 2 * S(0, 0) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[2].idx(), 2 * S(0, 0) * S(0, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[0].idx() + nv, -2 * S(0, 0) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[1].idx() + nv, -2 * S(0, 0) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[2].idx() + nv, -2 * S(0, 0) * S(1, 2)));
			// /partial{E}_{/partial{u_j}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[0].idx(), 2 * S(0, 1) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[1].idx(), 2 * S(0, 1) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[2].idx(), 2 * S(0, 1) * S(0, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[0].idx() + nv, -2 * S(0, 1) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[1].idx() + nv, -2 * S(0, 1) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[2].idx() + nv, -2 * S(0, 1) * S(1, 2)));
			// /partial{E}_{/partial{u_k}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[0].idx(), 2 * S(0, 2) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[1].idx(), 2 * S(0, 2) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[2].idx(), 2 * S(0, 2) * S(0, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[0].idx() + nv, -2 * S(0, 2) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[1].idx() + nv, -2 * S(0, 2) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[2].idx() + nv, -2 * S(0, 2) * S(1, 2)));

			// /partial{E}_{/partial{v_i}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[0].idx(), -2 * S(1, 0) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[1].idx(), -2 * S(1, 0) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[2].idx(), -2 * S(1, 0) * S(0, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[0].idx() + nv, 2 * S(1, 0) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[1].idx() + nv, 2 * S(1, 0) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[2].idx() + nv, 2 * S(1, 0) * S(1, 2)));
			// /partial{E}_{/partial{v_j}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[0].idx(), -2 * S(1, 1) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[1].idx(), -2 * S(1, 1) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[2].idx(), -2 * S(1, 1) * S(0, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[0].idx() + nv, 2 * S(1, 1) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[1].idx() + nv, 2 * S(1, 1) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[2].idx() + nv, 2 * S(1, 1) * S(1, 2)));
			// /partial{E}_{/partial{u_k}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[0].idx(), -2 * S(1, 2) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[1].idx(), -2 * S(1, 2) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[2].idx(), -2 * S(1, 2) * S(0, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[0].idx() + nv, 2 * S(1, 2) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[1].idx() + nv, 2 * S(1, 2) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[2].idx() + nv, 2 * S(1, 2) * S(1, 2)));

			//能量的后一半
			// /partial{E}_{/partial{u_i}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[0].idx(), 2 * S(1, 0) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[1].idx(), 2 * S(1, 0) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[2].idx(), 2 * S(1, 0) * S(1, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[0].idx() + nv, 2 * S(1, 0) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[1].idx() + nv, 2 * S(1, 0) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx(), f_v[2].idx() + nv, 2 * S(1, 0) * S(0, 2)));
			// /partial{E}_{/partial{u_j}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[0].idx(), 2 * S(1, 1) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[1].idx(), 2 * S(1, 1) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[2].idx(), 2 * S(1, 1) * S(1, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[0].idx() + nv, 2 * S(1, 1) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[1].idx() + nv, 2 * S(1, 1) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx(), f_v[2].idx() + nv, 2 * S(1, 1) * S(0, 2)));
			// /partial{E}_{/partial{u_k}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[0].idx(), 2 * S(1, 2) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[1].idx(), 2 * S(1, 2) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[2].idx(), 2 * S(1, 2) * S(1, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[0].idx() + nv, 2 * S(1, 2) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[1].idx() + nv, 2 * S(1, 2) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx(), f_v[2].idx() + nv, 2 * S(1, 2) * S(0, 2)));

			// /partial{E}_{/partial{v_i}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[0].idx(), 2 * S(0, 0) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[1].idx(), 2 * S(0, 0) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[2].idx(), 2 * S(0, 0) * S(1, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[0].idx() + nv, 2 * S(0, 0) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[1].idx() + nv, 2 * S(0, 0) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[0].idx() + nv, f_v[2].idx() + nv, 2 * S(0, 0) * S(0, 2)));
			// /partial{E}_{/partial{v_j}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[0].idx(), 2 * S(0, 1) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[1].idx(), 2 * S(0, 1) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[2].idx(), 2 * S(0, 1) * S(1, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[0].idx() + nv, 2 * S(0, 1) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[1].idx() + nv, 2 * S(0, 1) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[1].idx() + nv, f_v[2].idx() + nv, 2 * S(0, 1) * S(0, 2)));
			// /partial{E}_{/partial{u_k}}
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[0].idx(), 2 * S(0, 2) * S(1, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[1].idx(), 2 * S(0, 2) * S(1, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[2].idx(), 2 * S(0, 2) * S(1, 2)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[0].idx() + nv, 2 * S(0, 2) * S(0, 0)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[1].idx() + nv, 2 * S(0, 2) * S(0, 1)));
			tripletsL.push_back(Eigen::Triplet<double>(f_v[2].idx() + nv, f_v[2].idx() + nv, 2 * S(0, 2) * S(0, 2)));
		}
		//设置边界条件
		T::VertexHandle vh1 = mesh.vertex_handle(1);
		int fix1 = vh1.idx();
		double dis = DBL_MIN;
		int max_id = -1;
		for (T::VertexHandle vh : mesh.vertices())
		{
			double d = (mesh.point(vh) - mesh.point(vh1)).length();
			if (d > dis)
			{
				dis = d;
				max_id = vh.idx();
			}
		}
		T::VertexHandle vh2 = mesh.vertex_handle(max_id);
		int fix2 = vh2.idx();
		tripletsL.push_back(Eigen::Triplet<double>(2 * nv, fix1, 1));
		tripletsL.push_back(Eigen::Triplet<double>(fix1, 2 * nv, 1));
		tripletsL.push_back(Eigen::Triplet<double>(2 * nv + 1, fix1 + nv, 1));
		tripletsL.push_back(Eigen::Triplet<double>(fix1 + nv, 2 * nv + 1, 1));
		tripletsL.push_back(Eigen::Triplet<double>(2 * nv + 2, fix2, 1));
		tripletsL.push_back(Eigen::Triplet<double>(fix2, 2 * nv + 2, 1));
		tripletsL.push_back(Eigen::Triplet<double>(2 * nv + 3, fix2 + nv, 1));
		tripletsL.push_back(Eigen::Triplet<double>(fix2 + nv, 2 * nv + 3, 1));
		A.setFromTriplets(tripletsL.begin(), tripletsL.end());//设置好A
		Eigen::VectorXd b(2 * nv + 4);
		b.setZero();
		b(2 * nv + 2) = dis;
		//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;要求矩阵正定
		Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
		solver.compute(A);
		Eigen::VectorXd res = solver.solve(b);//参数化结果的点
		for (T::VertexHandle vh : mesh.vertices())
		{
			int idx = vh.idx();
			T::Point p(res(idx), res(idx + nv), 0);
			mesh.set_point(vh, p);
		}
		std::cout << "LSCM use method 2" << std::endl;
	}
}

void MyMesh::CalcABF()
{
	std::ofstream R("res.txt");
	//R.close();
	std::cout << "准备计算ABF" << std::endl;
	int nv = mesh.n_vertices();
	int nf = mesh.n_faces();
	//先算内部点的数量
	int n_bnd_v = 0;
	for (T::VertexHandle vh : mesh.vertices())
		if (!mesh.is_boundary(vh))
			n_bnd_v++;
	Eigen::VectorXd b(3 * nf + nf + n_bnd_v + n_bnd_v);
	b.setZero();
	Eigen::SparseMatrix<double> A(3 * nf + nf + n_bnd_v + n_bnd_v, 3 * nf + nf + n_bnd_v + n_bnd_v);
	std::vector<Eigen::Triplet<double>> tripletsL;
	std::vector < std::vector<std::pair<int, double>>> FV;//存放面的三个顶点信息
	FV.resize(nf);
	std::vector<double> Angledefect(nv, 0);
	for (T::FaceHandle fh : mesh.faces())
	{
		int idx = fh.idx();
		std::vector<T::VertexHandle> f_v;
		for (T::VertexHandle fv : mesh.fv_range(fh))
		{
			f_v.push_back(fv);
		}
		T::Point p0 = mesh.point(f_v[0]), p1 = mesh.point(f_v[1]), p2 = mesh.point(f_v[2]);
		double angle0, angle1, angle2;
		CalcTriangleAngles(p0, p1, p2, angle0, angle1, angle2);
		Angledefect[f_v[0].idx()] += angle0;
		Angledefect[f_v[1].idx()] += angle1;
		Angledefect[f_v[2].idx()] += angle2;
		FV[idx].resize(3);
		//存面的三个点的信息
		FV[idx][0].first = f_v[0].idx(); FV[idx][0].second = angle0;
		FV[idx][1].first = f_v[1].idx(); FV[idx][1].second = angle1;
		FV[idx][2].first = f_v[2].idx(); FV[idx][2].second = angle2;
	}
	for (int fid = 0; fid < nf; fid++)
	{
		std::vector<double> beta(3);
		std::vector<int> vid(3);
		beta[0] = FV[fid][0].second; vid[0] = FV[fid][0].first;
		beta[1] = FV[fid][1].second; vid[1] = FV[fid][1].first;
		beta[2] = FV[fid][2].second; vid[2] = FV[fid][2].first;
		for (int i = 0; i < 3; i++)
		{
			if (!mesh.is_boundary(mesh.vertex_handle(vid[i])))
			{
				beta[i] = beta[i] * 2 * M_PI / Angledefect[vid[i]];
				FV[fid][i].second = beta[i];//修正角度值
			}
			//对于每个角度的导数值等于0的能量约束
			tripletsL.push_back(Eigen::Triplet<double>(3 * fid + i, 3 * fid + i, 2 / (beta[i] * beta[i])));
			b[3 * fid + i] = 0;
		}
		//添加面的内角和为PI的约束
		tripletsL.push_back(Eigen::Triplet<double>(3 * nf + fid, 3 * fid, 1));
		tripletsL.push_back(Eigen::Triplet<double>(3 * nf + fid, 3 * fid + 1, 1));
		tripletsL.push_back(Eigen::Triplet<double>(3 * nf + fid, 3 * fid + 2, 1));

		tripletsL.push_back(Eigen::Triplet<double>(3 * fid, 3 * nf + fid, 1));
		tripletsL.push_back(Eigen::Triplet<double>(3 * fid + 1, 3 * nf + fid, 1));
		tripletsL.push_back(Eigen::Triplet<double>(3 * fid + 2, 3 * nf + fid, 1));

		b[3 * nf + fid] = M_PI - beta[0] - beta[1] - beta[2];
	}
	int n_bnd_idx = 0;//表示当前内部点的序号，首先周围一圈的角度之和得为2PI，还有就是重建约束，需要确定righthand和lefthand

	for (T::VertexHandle vh : mesh.vertices())
	{
		if (mesh.is_boundary(vh))
			continue;
		else
		{//设置内角和等于2pi的约束
			std::map<int, int> v_flag;//用于记录邻接点的访问情况,未访问则记录为0
			for (T::VertexHandle vh2 : mesh.vv_range(vh))
				v_flag[vh2.idx()] = 0;
			double sigma = 0;
			std::map<int, std::map<int, int>> FV_flag;
			for (T::FaceHandle vfh : mesh.vf_range(vh))
			{
				std::vector<int> fv_flaginfo;
				int fid = vfh.idx();
				for (int fv_id = 0; fv_id < 3; fv_id++)
				{
					if (FV[fid][fv_id].first != vh.idx())
					{
						tripletsL.push_back(Eigen::Triplet<double>(4 * nf + n_bnd_idx, 3 * fid + fv_id, 1));
						tripletsL.push_back(Eigen::Triplet<double>(3 * fid + fv_id, 4 * nf + n_bnd_idx, 1));
						sigma += FV[fid][fv_id].second;
						FV_flag[fid][FV[fid][fv_id].first] = 0;
					}
					else
						FV_flag[fid][FV[fid][fv_id].first] = -2;//表示这个顶点是面的中心点
				}
			}
			//FV_flag记录了一个面的三个顶点的正负值
			b[4 * nf + n_bnd_idx] = 2 * M_PI - sigma;
			std::vector<int> last_v, now_v;//记录上一个访问面的三个顶点和当前访问面的三个顶点
			int common_v;
			int first = 1;
			for (T::FaceHandle vfh : mesh.vf_range(vh))
			{
				if (FV_flag.count(vfh.idx()) == 0)
					std::cerr << "未收录的面" << std::endl;
				//std::cout << vfh.idx() << std::endl;
				last_v = now_v;
				now_v.clear();
				for (auto& a : FV_flag[vfh.idx()])//遍历面的所有顶点
				{
					if (a.second != -2)
					{
						now_v.push_back(a.first);
						if (first)
						{
							common_v = a.first;
						}
					}
				}
				if (!first)
				{
					if (last_v[0] == now_v[0] || last_v[0] == now_v[1])
						common_v = last_v[0];
					if (last_v[1] == now_v[0] || last_v[1] == now_v[1])
						common_v = last_v[1];
				}
				else
					first = false;
				for (auto& a : FV_flag[vfh.idx()])
				{
					if (a.second != -2)
					{
						if (a.first == common_v)
							a.second = -1;
						else a.second = 1;
						if (first)
						{
							a.second = 1;
						}
					}
				}
			}
			double right_value = 0;
			int row = 4 * nf + n_bnd_v + n_bnd_idx;//矩阵行号
			for (auto a : FV_flag)//遍历每个相邻的面
			{
				int fid = a.first;
				for (int idx = 0; idx < 3; idx++)
				{
					if (a.second[FV[fid][idx].first] == -2)
						continue;
					int col = 3 * fid + idx;
					double value = a.second[FV[fid][idx].first] /tan(FV[fid][idx].second);//根据求出来的符号值进行赋值
					right_value += -a.second[FV[fid][idx].first] * log(sin(FV[fid][idx].second));
					tripletsL.push_back(Eigen::Triplet<double>(row, col, value));
					tripletsL.push_back(Eigen::Triplet<double>(col, row, value));
				}
			}
			b[row] = right_value;
			n_bnd_idx++;
		}
	}
	A.setFromTriplets(tripletsL.begin(), tripletsL.end());//设置好A
	//R << "A:" << std::endl << A << std::endl << "b:" << std::endl << b << std::endl;
	//R << "denseA:" << std::endl << A.toDense().inverse() << std::endl;

	//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;要求矩阵正定
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
	solver.compute(A);
	//R << "b:" << b << std::endl;
	//R.close();
	Eigen::VectorXd res = solver.solve(b);//参数化结果的点,解出角度值

	R << "A:" << std::endl << A.toDense() << std::endl << "b:" << std::endl << b << std::endl << "res:" << std::endl << res << std::endl;
	Eigen::SparseMatrix<double> A2(2 * nv + 4, 2 * nv + 4);
	Eigen::VectorXd b2(2 * nv + 4);
	b2.setZero();
	std::vector<Eigen::Triplet<double>> tripletsL2;
	R << "ALPHA:" << std::endl;
	for (T::FaceHandle fh : mesh.faces())
	{
		int fid = fh.idx();
		int vid0 = FV[fid][0].first, vid1 = FV[fid][1].first, vid2 = FV[fid][2].first;
		Eigen::MatrixXd Mt(2, 2);
		double alpha0 = res(3 * fid) + FV[fid][0].second, alpha1 = res(3 * fid + 1) + FV[fid][1].second, alpha2 = res(3 * fid + 2) + FV[fid][2].second;//求出的角度值
		//double alpha0 = res(3 * fid) - FV[fid][0].second, alpha1 = res(3 * fid + 1) - FV[fid][1].second, alpha2 = res(3 * fid + 2) - FV[fid][2].second;//求出的角度值

		R<< alpha0 << "    " << alpha1 << "    " << alpha2 << std::endl;
		double q = sin(alpha2) / sin(alpha0);
		Mt(0, 0) = q * cos(alpha1); Mt(0, 1) = q * (-sin(alpha1));
		Mt(1, 0) = q * sin(alpha1); Mt(1, 1) = q * cos(alpha1);

		tripletsL2.push_back(Eigen::Triplet<double>(vid0, vid0, 2));
		tripletsL2.push_back(Eigen::Triplet<double>(vid0, vid1, -2 * (1 - Mt(0, 0))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid0, vid2, -2 * Mt(0, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid0, nv + vid1, 2 * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid0, nv + vid2, -2 * Mt(0, 1)));

		tripletsL2.push_back(Eigen::Triplet<double>(vid1, vid0, -2 * (1 - Mt(0, 0))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1, vid1, 2 * (1 - Mt(0, 0)) * (1 - Mt(0, 0))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1, vid2, 2 * Mt(0, 0) * (1 - Mt(0, 0))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1, vid1, -2 * (1 - Mt(0, 0)) * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1, vid2, 2 * (1 - Mt(0, 0)) * Mt(0, 1)));

		tripletsL2.push_back(Eigen::Triplet<double>(vid2, vid0, -2 * Mt(0, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2, vid1, 2 * (1 - Mt(0, 0)) * Mt(0, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2, vid2, 2 * Mt(0, 0) * Mt(0, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2, vid1, -2 * Mt(0, 0) * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2, vid2, 2 * Mt(0, 0) * Mt(0, 1)));

		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid0, 2 * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid1, -2 * (1 - Mt(0, 0)) * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid2, -2 * Mt(0, 0) * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid1, 2 * Mt(0, 1) * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid2, -2 * Mt(0, 1) * Mt(0, 1)));

		tripletsL2.push_back(Eigen::Triplet<double>(vid2 + nv, vid0, -2 * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2 + nv, vid1, 2 * (1 - Mt(0, 0)) * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2 + nv, vid2, 2 * Mt(0, 0) * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2 + nv, vid1, -2 * Mt(0, 1) * Mt(0, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2 + nv, vid2, 2 * Mt(0, 1) * Mt(0, 1)));

		tripletsL2.push_back(Eigen::Triplet<double>(vid1, vid1, 2 * Mt(1, 0) * Mt(1, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1, vid2, -2 * Mt(1, 0) * Mt(1, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1, vid0 + nv, 2 * Mt(1, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1, vid1 + nv, -2 * Mt(1, 0) * (1 - Mt(1, 1))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1, vid2 + nv, -2 * Mt(1, 0) * Mt(1, 1)));

		tripletsL2.push_back(Eigen::Triplet<double>(vid2, vid1, -2 * Mt(1, 0) * Mt(1, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2, vid2, 2 * Mt(1, 0) * Mt(1, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2, vid0 + nv, -2 * Mt(1, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2, vid1 + nv, 2 * Mt(1, 0) * (1 - Mt(1, 1))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid2, vid2 + nv, 2 * Mt(1, 0) * Mt(1, 1)));

		tripletsL2.push_back(Eigen::Triplet<double>(vid0 + nv, vid1, 2 * Mt(1, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid0 + nv, vid2, -2 * Mt(1, 0)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid0 + nv, vid0 + nv, 2));
		tripletsL2.push_back(Eigen::Triplet<double>(vid0 + nv, vid1 + nv, -2 * (1 - Mt(1, 1))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid0 + nv, vid2 + nv, -2 * Mt(1, 1)));

		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid1, -2 * Mt(1, 0) * (1 - Mt(1, 1))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid2, 2 * Mt(1, 0) * (1 - Mt(1, 1))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid0 + nv, -2 * (1 - Mt(1, 1))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid1 + nv, 2 * (1 - Mt(1, 1)) * (1 - Mt(1, 1))));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid2 + nv, 2 * Mt(1, 1) * (1 - Mt(1, 1))));

		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid1, -2 * Mt(1, 0) * Mt(1, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid2, 2 * Mt(1, 0) * Mt(1, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid0 + nv, -2 * Mt(1, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid1 + nv, 2 * (1 - Mt(1, 1)) * Mt(1, 1)));
		tripletsL2.push_back(Eigen::Triplet<double>(vid1 + nv, vid2 + nv, 2 * Mt(1, 1) * Mt(1, 1)));
	}
	T::EdgeHandle eh = mesh.edge_handle(2);
	T::VertexHandle v1 = mesh.to_vertex_handle(mesh.halfedge_handle(eh, 0));
	T::VertexHandle v2 = mesh.from_vertex_handle(mesh.halfedge_handle(eh, 0));
	double length = mesh.calc_edge_length(eh);
	tripletsL2.push_back(Eigen::Triplet<double>(2 * nv, v1.idx(), 1));
	tripletsL2.push_back(Eigen::Triplet<double>(v1.idx(), 2 * nv, 1));
	tripletsL2.push_back(Eigen::Triplet<double>(2 * nv + 1, v1.idx() + nv, 1));
	tripletsL2.push_back(Eigen::Triplet<double>(v1.idx() + nv, 2 * nv + 1, 1));
	tripletsL2.push_back(Eigen::Triplet<double>(2 * nv + 2, v2.idx(), 1));
	tripletsL2.push_back(Eigen::Triplet<double>(v2.idx(), 2 * nv + 2, 1));
	tripletsL2.push_back(Eigen::Triplet<double>(2 * nv + 3, v2.idx() + nv, 1));
	tripletsL2.push_back(Eigen::Triplet<double>(v2.idx() + nv, 2 * nv + 3, 1));
	b2[2 * nv + 2] = length;

	A2.setFromTriplets(tripletsL2.begin(), tripletsL2.end());//设置好A
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver2;
	solver2.compute(A2);
	Eigen::VectorXd res2 = solver2.solve(b2);//参数化结果的点,解出角度值
	for (T::VertexHandle vh : mesh.vertices())
	{
		int idx = vh.idx();
		T::Point p(res2(idx), res2(idx + nv), 0);
		mesh.set_point(vh, p);
	}
	//R.close();
}

void MyMesh::getVCurvature(std::vector<double>& c)
{
	c.resize(mesh.n_vertices());

	for (int i = 0; i < Curvature.size(); i++)
	{
		c[i] = Curvature[i];
	}
}

MyMesh::MyMesh()
{
	//	solver = new EigenLinSolver();
}

MyMesh::MyMesh(const MyMesh& copy)
{
	this->mesh = copy.mesh;
}

//输入三角形的三个顶点获取对应的三个角
void MyMesh::CalcTriangleAngles(T::Point& p1, T::Point& p2, T::Point& p3, double& angle1, double& angle2, double& angle3)
{
	OpenMesh::Vec3d e12 = p2 - p1;
	OpenMesh::Vec3d e13 = p3 - p1;
	OpenMesh::Vec3d e23 = p3 - p2;
	e12 = e12.normalize();
	e13 = e13.normalize();
	e23 = e23.normalize();
	angle1 = acos(e12 | e13);
	angle2 = acos(-e12 | e23);
	angle3 = acos(e13 | e23);
}

//根据三角形的三个顶点位置求取三角形的外心
T::Point MyMesh::CalcTriangleCirumcenter(T::Point& p1, T::Point& p2, T::Point& p3)
{//处理三点共线的情况
	OpenMesh::Vec3d e12 = p2 - p1;
	OpenMesh::Vec3d e13 = p3 - p1;
	if (e12.data()[0] * e13.data()[1] - e13.data()[0] * e12.data()[1] == 0)
	{
		std::cerr << "三点共线发生" << std::endl;
		return (p1 + p2 + p3) / 3;//如果共线则返回三点的重心
	}
	T::Normal normal = e12 % e13;
	T::Normal dir = normal % e12;
	dir = dir / dir.norm();
	T::Point m12 = (p1 + p2) / 2;
	T::Point m23 = (p2 + p3) / 2;
	T::Point m13 = (p1 + p3) / 2;
	double kup1 = (m12.data()[0] - p1.data()[0]) * (m12.data()[0] - p1.data()[0]) + (m12.data()[1] - p1.data()[1]) * (m12.data()[1] - p1.data()[1]) + (m12.data()[2] - p1.data()[2]) * (m12.data()[2] - p1.data()[2]);
	double kup2 = (m12.data()[0] - p3.data()[0]) * (m12.data()[0] - p3.data()[0]) + (m12.data()[1] - p3.data()[1]) * (m12.data()[1] - p3.data()[1]) + (m12.data()[2] - p3.data()[2]) * (m12.data()[2] - p3.data()[2]);
	double kdown1 = (m12.data()[0] - p1.data()[0]) * dir.data()[0] + (m12.data()[1] - p1.data()[1]) * dir.data()[1] + (m12.data()[2] - p1.data()[2]) * dir.data()[2];
	double kdown2 = (m12.data()[0] - p3.data()[0]) * dir.data()[0] + (m12.data()[1] - p3.data()[1]) * dir.data()[1] + (m12.data()[2] - p3.data()[2]) * dir.data()[2];

	double k = (kup1 - kup2) / (2 * (kdown2 - kdown1));
	T::Point center = m12 + k * dir;
	return center;
}

double MyMesh::CalcTriangleArea(T::Point& p1, T::Point& p2, T::Point& p3)
{
	OpenMesh::Vec3d e12 = (p2 - p1);
	OpenMesh::Vec3d e13 = (p3 - p1);
	double area = 0.5 * (e12 % e13).norm();
	return area;
}

double MyMesh::ComputeMeshVolume()
{
	mesh.update_face_normals();
	mesh.request_face_normals();
	double vol = 0;
	for (auto fh : mesh.faces())
	{
		std::vector<T::Point> p;
		for (auto fi : mesh.fv_range(fh))
			p.push_back(mesh.point(fi));
		OpenMesh::Vec3d e01 = p[1] - p[0];
		OpenMesh::Vec3d e02 = p[2] - p[0];
		double area = 0.5 * (e01 % e02).norm();
		OpenMesh::Vec3d n = mesh.normal(fh);
		OpenMesh::Vec3d eh = p[0];
		double height = (n | eh);
		vol += area * height / 3;
	}
	this->MeshVolume = vol;
	std::cout << vol << std::endl;
	return vol;
}

