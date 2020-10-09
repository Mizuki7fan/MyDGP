#include "MyMesh_Openmesh.h"


Mesh::Mesh()
{
}

bool Mesh::Load(std::string s)
{//加载网格
	return OpenMesh::IO::read_mesh(mesh, s);
}

void Mesh::LoadVertex()
{
	if (Vertices_latest)
		return;
	Vertices.resize(mesh.n_vertices(),3);
	for (auto a : mesh.vertices())
	{
		T::Point p = mesh.point(a);
		Vertices(a.idx(),0) = p.data()[0];
		Vertices(a.idx(), 1) = p.data()[1];
		Vertices(a.idx(), 2) = p.data()[2];
	}
	Vertices_latest = true;
	LAR_latest = false;
	Laplacian_latest = false;
	Curvature_latest = false;
	MeshVolume_latest = false;
	VertexNormal_latest = false;
}

bool Mesh::Write(std::string s)
{
	return OpenMesh::IO::write_mesh(mesh, s);
}

void Mesh::Clear()
{
	mesh.clear();
}

void Mesh::UpdateNormals()
{
	mesh.update_normals();
	mesh.request_face_normals();
}

void Mesh::LoadVerticeNormal(void)
{
	if (VertexNormal_latest)
		return;
	mesh.update_vertex_normals();
	VertexNormal.resize(mesh.n_vertices(),3);
	for (auto v : mesh.vertices())
	{
		OpenMesh::Vec3d n = mesh.normal(v);
		VertexNormal(v.idx(), 0) = n.data()[0];
		VertexNormal(v.idx(), 1) = n.data()[1];
		VertexNormal(v.idx(), 2) = n.data()[2];
	}
	VertexNormal_latest = true;
	Curvature_latest = false;
}

void Mesh::LoadFaceNormal(void)
{
	if (FaceNormal_latest)
		return;
	else if (!Vertices_latest)
	{
		SetVerticesNewCoord();
		mesh.update_vertex_normals();
		mesh.update_face_normals();
		Vertices_latest = true;
	}
	FaceNormal.resize(mesh.n_faces(), 3);
	mesh.update_face_normals();

	for (auto f : mesh.faces())
	{
		OpenMesh::Vec3d n = mesh.normal(f);
		FaceNormal(f.idx(), 0) = n.data()[0];
		FaceNormal(f.idx(), 1) = n.data()[1];
		FaceNormal(f.idx(), 2) = n.data()[2];
	}
	FaceNormal_latest = true;
}

bool Mesh::VerticesEmpty()
{
	return mesh.vertices_empty();
}

int Mesh::NVertices() const
{
	return mesh.n_vertices();
}

int Mesh::NEdges() const
{
	return mesh.n_edges();
}

int Mesh::NFaces() const
{
	return mesh.n_faces();
}

double Mesh::CalcEdgeLength(int i)
{
	return mesh.calc_edge_length(mesh.edge_handle(i));
}

double Mesh::CalcFaceArea()
{
	double area = 0;
	for (auto fi : mesh.faces())
	{
		std::vector<T::VertexHandle> v;
		for (auto fvi : mesh.fv_range(fi))
		{
			v.push_back(fvi);
		}
		OpenMesh::Vec3d e01 = mesh.point(v[1]) - mesh.point(v[0]);
		OpenMesh::Vec3d e02 = mesh.point(v[2]) - mesh.point(v[0]);
		area += 0.5 * (e01 % e02).norm();
	}

	return area;
}

double Mesh::getEdgeLength(int i) const
{
	return mesh.calc_edge_length(mesh.edge_handle(i));
}

Eigen::Vector3d Mesh::getPoint(int i) const
{
	T::Point tp = mesh.point(mesh.vertex_handle(i));
	return Eigen::Vector3d(tp[0],tp[1],tp[2]);
}

Eigen::Vector3d Mesh::getVertexNormal(int i) const
{
	OpenMesh::Vec3d n = mesh.normal(mesh.vertex_handle(i));
	return Eigen::Vector3d(n.data()[0], n.data()[1], n.data()[2]);
}

Eigen::Vector3d Mesh::getFaceNormal(int i) const
{
	OpenMesh::Vec3d n = mesh.normal(mesh.face_handle(i));
	return Eigen::Vector3d(n.data()[0], n.data()[1], n.data()[2]);
}
void Mesh::getVerticesNeighbour(int i, std::vector<int>& nei) const
{
	nei.clear();
	T::VertexHandle vh = mesh.vertex_handle(i);
	for (auto vi:mesh.vv_range(vh))
	{
		nei.push_back(vi.idx());
	}
}
void Mesh::getFaceNeighbour(int i, std::vector<int>& nei) const
{
	nei.clear();
	T::FaceHandle fh = mesh.face_handle(i);
	for (auto fi : mesh.fv_range(fh))
	{
		nei.push_back(fi.idx());
	}
}
void Mesh::getEdgeVertices(int e, int& v1, int& v2) const
{
	T::HalfedgeHandle he = mesh.halfedge_handle(mesh.edge_handle(e),0);
	T::VertexHandle vi = mesh.to_vertex_handle(he);
	T::VertexHandle vj = mesh.from_vertex_handle(he);
	v1 = vi.idx();
	v2 = vj.idx();
}

void Mesh::getFaceVertices(int f, int& v1, int& v2, int& v3) const
{
	T::FaceHandle fh = mesh.face_handle(f);
	std::vector<int> fv;
	for (auto fvi : mesh.fv_range(fh))
	{
		fv.push_back(fvi.idx());
	}
	v1 = fv[0]; v2 = fv[1]; v3 = fv[2];
}

void Mesh::getFaceAngles(int f, double& angle1, double& angle2, double& angle3) const
{
	T::FaceHandle fh = mesh.face_handle(f);
	std::vector<T::Point> fv;
	for (auto fvi : mesh.fv_range(fh))
	{
		fv.push_back(mesh.point(fvi));
	}
	OpenMesh::Vec3d e12= fv[1] - fv[0];
	OpenMesh::Vec3d e13 = fv[2] - fv[0];
	OpenMesh::Vec3d e23 = fv[2] - fv[1];
	e12 = e12.normalize();
	e13 = e13.normalize();
	e23 = e23.normalize();
	angle1 = acos(e12 | e13);
	angle2 = acos(-e12 | e23);
	angle3 = acos(e13 | e23);
}

bool Mesh::isBoundary(int i) const
{
	return mesh.is_boundary(mesh.edge_handle(i));
}

bool Mesh::isBoundaryVertex(int i) const
{
	return mesh.is_boundary(mesh.vertex_handle(i));
}

void Mesh::SetVerticesNewCoord()
{
	for (auto vh : mesh.vertices())
	{
		int idx = vh.idx();
		T::Point p(Vertices(vh.idx(), 0), Vertices(vh.idx(), 1), Vertices(vh.idx(), 2));
		mesh.set_point(vh, p);
	}
	Vertices_latest = false;
}

void Mesh::SetVertexNewCoord(int i, Eigen::Vector3d p)
{
	T::VertexHandle vh = mesh.vertex_handle(i);
	T::Point p1(p.x(),p.y(),p.z());
	mesh.set_point(vh,p1);
	Vertices_latest = false;}

void Mesh::SetFacesNewNormalCoord()
{
	for (int i = 0; i < mesh.n_faces(); i++)
	{
		T::FaceHandle fh = mesh.face_handle(i);
		T::Normal n(FaceNormal(i, 0), FaceNormal(i, 1), FaceNormal(i, 2));
		mesh.set_normal(fh, n);
//		mesh.set_normal()
	}
	FaceNormal_latest = false;
}

Eigen::Vector3d Mesh::getVertexCoord(int i)
{
	T::VertexHandle vh= mesh.vertex_handle(i);
	T::Point p = mesh.point(vh);
	return Eigen::Vector3d(p.data()[0],p.data()[1],p.data()[2]);
}

double Mesh::ComputeMeshVolume()
{
	//先做定向，然后算原点到面的投影的模长
	mesh.update_face_normals();
	mesh.request_face_normals();
	double vol = 0;
	for (auto fh : mesh.faces())
	{
		std::vector<T::Point> p;
		//求底面面积
		for (auto fi : mesh.fv_range(fh))
			p.push_back(mesh.point(fi));
		OpenMesh::Vec3d e01 = p[1] - p[0];
		OpenMesh::Vec3d e02 = p[2] - p[0];
		double area = 0.5 * (e01 % e02).norm();
		//求高
		OpenMesh::Vec3d n = mesh.normal(fh);
		OpenMesh::Vec3d eh = p[0];
		double height =(n | eh);
		vol += area * height / 3;
	}
	this->MeshVolume = vol;
	std::cout << "网格体积为: "<<vol << std::endl;
	return vol;
}

void Mesh::ComputeLaplacian()
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
		if (!LAR_latest)
			ComputeLAR();
		if (Laplacian_latest)
			return;
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
	Laplacian_latest = true;
	Curvature_latest = false;
	std::cout << "更新了Laplacian" << std::endl;
}

void Mesh::BilateralDenoising(double stdevs, double stdevr)
{
	Eigen::VectorXd D;
	D.resize(mesh.n_vertices());//顶点要沿着其法向进行偏移的距离
	mesh.update_normals();
	for (T::VertexHandle vh : mesh.vertices())
	{
		T::Normal N = mesh.normal(vh);
		if (vh.idx()==130)
			std::cout << N << std::endl;
		T::Point V = mesh.point(vh);
		double d_total = 0;
		double Kv = 0;
		for (T::VertexHandle vj : mesh.vv_range(vh))
		{
			T::Point Q = mesh.point(vj);
			double t = (Q - V).norm();
			double d = (N | (Q - V)) / N.norm();//点乘
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

void Mesh::BilateralNormalFiltering(double stdevs, double stdevr)
{
	mesh.update_normals();
	Eigen::MatrixXd NewNormal(NFaces(), 3);
	Eigen::VectorXd FaceArea(mesh.n_faces());//三角形的面积
	for (T::FaceHandle fh : mesh.faces())//更新三角形的面积和中心点
	{
		std::vector<T::Point> P;
		for (T::VertexHandle vh : mesh.fv_range(fh))
		{
			P.push_back(mesh.point(vh));
		}
		auto e12 = (P[1] - P[0]);
		auto e13 = (P[2] - P[0]);
		double area = 0.5 * (e12 % e13).norm();
		FaceArea[fh.idx()] = area;
	}

	for (T::FaceHandle fh : mesh.faces())
	{
		T::Normal N = mesh.normal(fh);
		double newNormalX = 0, newNormalY = 0, newNormalZ = 0;//新法向的三个值
		T::Point center(0, 0, 0);
		for (T::VertexHandle fvi : mesh.fv_range(fh))//求面的中心
		{
			center += mesh.point(fvi);
		}
		center = center / 3;
		double Kp = 0;
		for (T::FaceHandle nei_fh : mesh.ff_range(fh))//遍历邻域的面
		{
			T::Point nei_center(0, 0, 0);
			for (T::VertexHandle nei_fvi : mesh.fv_range(nei_fh))
			{
				nei_center += mesh.point(nei_fvi);
			}
			nei_center = nei_center / 3;//相邻面的中心
			T::Normal nei_N = mesh.normal(nei_fh);
			double delta_center = (nei_center - center).norm();
			double delta_normal = (nei_N - N).norm();
			double Aj = FaceArea[nei_fh.idx()];
			Kp += Aj;
			double Ws = exp(-delta_center * delta_center / (2 * stdevs));
			double Wr = exp(-delta_normal * delta_normal / (2 * stdevr));
			newNormalX += Aj * Ws * Wr * nei_N.data()[0];
			newNormalY += Aj * Ws * Wr * nei_N.data()[1];
			newNormalZ += Aj * Ws * Wr * nei_N.data()[2];
		}
		newNormalX /= Kp;
		newNormalY /= Kp;
		newNormalZ /= Kp;
		double norm = sqrt(newNormalX * newNormalX + newNormalY * newNormalY + newNormalZ * newNormalZ);
		//double norm = 1;
		//std::cout << norm << std::endl;
		NewNormal(fh.idx(), 0) = newNormalX / norm;
		NewNormal(fh.idx(), 1) = newNormalY / norm;
		NewNormal(fh.idx(), 2) = newNormalZ / norm;
	}
	//std::ofstream ori_normal("output//ori_normal.txt");
	//Eigen::MatrixXd oriNormal(mesh.n_faces(),3);
	//for (auto fh : mesh.faces())
	//{
	//	T::Normal N = mesh.normal(fh);
	//	oriNormal(fh.idx(), 0) = N.data()[0];
	//	oriNormal(fh.idx(), 1) = N.data()[1];
	//	oriNormal(fh.idx(), 2) = N.data()[2];
	//}
	//ori_normal << oriNormal;
	//ori_normal.close();
//	for (T::FaceHandle fh : mesh.faces())
//	{
//		T::Normal n(NewNormal(fh.idx(), 0), NewNormal(fh.idx(),1), NewNormal(fh.idx(), 2));
//		mesh.set_normal(fh, n);
//	}//暂时更新法向，根据法向求新顶点
	//std::ofstream new_normal("output//new_normal.txt");
	//Eigen::MatrixXd newNormal(mesh.n_faces(),3);
	//for (auto fh : mesh.faces())
	//{
	//	T::Normal N = mesh.normal(fh);
	//	newNormal(fh.idx(), 0) = N.data()[0];
	//	newNormal(fh.idx(), 1) = N.data()[1];
	//	newNormal(fh.idx(), 2) = N.data()[2];
	//}
	//new_normal << newNormal;
	//new_normal.close();
	
	//用Gauss-Seidel迭代,迭代20此
	for (int i = 0; i < 200; i++)
	{
		for (T::VertexHandle vh : mesh.vertices())
		{
			T::Point x_i = mesh.point(vh);
			T::Point delta_xi(0, 0, 0);
			int Nei_count = 0;
			for (T::FaceHandle fh : mesh.vf_range(vh))
			{//新的面法向
				Nei_count++;
				T::Normal nj(NewNormal(fh.idx(), 0), NewNormal(fh.idx(), 1), NewNormal(fh.idx(), 2));
				//由于每次的顶点是在不断修改的，所以不能预存中心点
				std::vector<T::Point> P;
				for (T::VertexHandle vh : mesh.fv_range(fh))
				{
					P.push_back(mesh.point(vh));
				}
				T::Point cj = (P[0] + P[1] + P[2]) / 3;
				delta_xi += nj * (nj.data()[0] * (cj - x_i).data()[0] + nj.data()[1] * (cj - x_i).data()[1] + nj.data()[2] * (cj - x_i).data()[2]);
			}
			x_i += delta_xi / Nei_count;
			mesh.set_point(vh, x_i);
		}
	}
}
void Mesh::CalcTutte()
{
	std::cout << 213 << std::endl;
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
			//A(vh.idx(), vnei.idx()) = 1.0/nei_count;
			A(vh.idx(), vnei.idx()) = 1.0;
		}
		//A(vh.idx(), vh.idx()) = -1;
		A(vh.idx(), vh.idx()) = -nei_count;//这么写可以获得一个对称矩阵
	}
	Eigen::SparseMatrix<double> LL = A.sparseView();
	//Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;要求矩阵正定
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
//批量注释快捷键:Ctrl + K + C批量取消注释快捷键: Ctrl + K + U