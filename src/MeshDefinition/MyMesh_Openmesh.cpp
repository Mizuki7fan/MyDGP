#include "MyMesh_Openmesh.h"


Mesh::Mesh()
{
	mesh.update_vertex_normals();
	mesh.request_vertex_normals();
	mesh.update_face_normals();
	mesh.request_face_normals();
}

bool Mesh::Load(std::string s)
{
	return OpenMesh::IO::read_mesh(mesh, s);
}

void Mesh::LoadVertex()
{
	Vertices.resize(mesh.n_vertices(),3);
	Vertices.setZero();
	for (auto a : mesh.vertices())
	{
//		Vertices[a.idx()] = mesh.point(a);
		T::Point p = mesh.point(a);
		Vertices(a.idx(),0) = p.data()[0];
		Vertices(a.idx(), 1) = p.data()[1];
		Vertices(a.idx(), 2) = p.data()[2];
	}
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

void Mesh::RequestVerticeNormal(void)
{
	VertexNormal.resize(mesh.n_vertices(),3);
	for (auto v : mesh.vertices())
	{
		OpenMesh::Vec3d n = mesh.normal(v);
		VertexNormal(v.idx(), 0) = n.data()[0];
		VertexNormal(v.idx(), 1) = n.data()[1];
		VertexNormal(v.idx(), 2) = n.data()[2];
	}
}

void Mesh::RequestFaceNormal(void)
{
	FaceNormal.resize(mesh.n_faces(), 3);
	for (auto f : mesh.faces())
	{
		OpenMesh::Vec3d n = mesh.normal(f);
		FaceNormal(f.idx(), 0) = n.data()[0];
		FaceNormal(f.idx(), 1) = n.data()[1];
		FaceNormal(f.idx(), 2) = n.data()[2];
	}
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

MyMesh::Point Mesh::getPoint(int i) const
{
	T::Point tp = mesh.point(mesh.vertex_handle(i));
	return MyMesh::Point(tp[0],tp[1],tp[2]);
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

void Mesh::SetVerticeNewCoord(int i, Eigen::Vector3d p)
{
	T::VertexHandle vh = mesh.vertex_handle(i);
	T::Point p1(p.x(),p.y(),p.z());
	mesh.set_point(vh,p1);
}

Eigen::Vector3d Mesh::getVertexCoord(int i)
{
	T::VertexHandle vh= mesh.vertex_handle(i);
	T::Point p = mesh.point(vh);
	return Eigen::Vector3d(p.data()[0],p.data()[1],p.data()[2]);
}

void Mesh::ComputeLAR(int kind)
{
	int nv = mesh.n_vertices();
	LAR.resize(nv);
	for (T::FaceHandle fh : mesh.faces())
	{
		T::Point center1, center2, center3;
		T::Point p1, p2, p3;
		std::vector<int> fv;
		for (auto fvi : mesh.fv_range(fh))
		{
			fv.push_back(fvi.idx());
		}
		p1 = mesh.point(mesh.vertex_handle(fv[0]));
		p2 = mesh.point(mesh.vertex_handle(fv[1]));
		p3 = mesh.point(mesh.vertex_handle(fv[2]));
		T::Point m12 = (p1 + p2) / 2, m13 = (p1 + p3) / 2, m23 = (p2 + p3) / 2;
		switch (kind)
		{
		case 0://重心坐标
			center1 = (p1 + p2 + p3) / 3;
			center2 = center1;
			center3 = center1;
			break;
		case 1:
			center1 = toPoint(ComputeTriangleCenter(Eigen::Vector3d(mesh.normal(fh).data()), Eigen::Vector3d(p1.data()), Eigen::Vector3d(p2.data()), Eigen::Vector3d(p3.data())));
			center2 = toPoint(ComputeTriangleCenter(Eigen::Vector3d(mesh.normal(fh).data()), Eigen::Vector3d(p2.data()), Eigen::Vector3d(p3.data()), Eigen::Vector3d(p1.data())));
			center3 = toPoint(ComputeTriangleCenter(Eigen::Vector3d(mesh.normal(fh).data()), Eigen::Vector3d(p3.data()), Eigen::Vector3d(p1.data()), Eigen::Vector3d(p2.data())));
			break;
		case 2:
		{
			OpenMesh::Vec3d e12 = mesh.point(mesh.vertex_handle(fv[1])) - mesh.point(mesh.vertex_handle(fv[0]));
			OpenMesh::Vec3d e13 = mesh.point(mesh.vertex_handle(fv[2])) - mesh.point(mesh.vertex_handle(fv[0]));
			OpenMesh::Vec3d e23 = mesh.point(mesh.vertex_handle(fv[2])) - mesh.point(mesh.vertex_handle(fv[1]));
			e12 = e12.normalize();
			e13 = e13.normalize();
			e23 = e23.normalize();
			double angle1 = acos(e12 | e13);
			double angle2 = acos(-e12 | e23);
			double angle3 = acos(e13 | e23);
			if (angle1 <= 0.5 * M_PI)
				center1 = toPoint(ComputeTriangleCenter(Eigen::Vector3d(mesh.normal(fh).data()), Eigen::Vector3d(p1.data()), Eigen::Vector3d(p2.data()), Eigen::Vector3d(p3.data())));
			else
				center1 = (p2 + p3)/ 2;
			if (angle2 <= 0.5 * M_PI)
				center2 = toPoint(ComputeTriangleCenter(Eigen::Vector3d(mesh.normal(fh).data()), Eigen::Vector3d(p2.data()), Eigen::Vector3d(p3.data()), Eigen::Vector3d(p1.data())));
			else
				center2 = (p1 + p3) / 2;
			if (angle3 <= 0.5 * M_PI)
				center3 = toPoint(ComputeTriangleCenter(Eigen::Vector3d(mesh.normal(fh).data()), Eigen::Vector3d(p3.data()), Eigen::Vector3d(p1.data()), Eigen::Vector3d(p2.data())));
			else
				center3 = (p2 + p1) / 2;
			break;
		}
		default:
			break;
		}
		LAR[fv[0]] += ComputeArea(Eigen::Vector3d(p1.data()), Eigen::Vector3d(m12.data()), Eigen::Vector3d(center1.data())) + ComputeArea(Eigen::Vector3d(p1.data()), Eigen::Vector3d(m13.data()), Eigen::Vector3d(center1.data()));
		LAR[fv[1]] += ComputeArea(Eigen::Vector3d(p2.data()), Eigen::Vector3d(m12.data()), Eigen::Vector3d(center2.data())) + ComputeArea(Eigen::Vector3d(p2.data()), Eigen::Vector3d(m23.data()), Eigen::Vector3d(center2.data()));
		LAR[fv[2]] += ComputeArea(Eigen::Vector3d(p3.data()), Eigen::Vector3d(m13.data()), Eigen::Vector3d(center3.data())) + ComputeArea(Eigen::Vector3d(p3.data()), Eigen::Vector3d(m23.data()), Eigen::Vector3d(center3.data()));
	}
}

void Mesh::ComputeLaplacian(int kind)
{
	//使用Openmesh的函数功能来填装Laplacian
	int nv = mesh.n_vertices();
	Laplacian.resize(nv, nv);
	Laplacian.setZero();//这里应当改成稀疏矩阵
	if (kind == 0)
	{
		//如何快速构建uniform的Laplacian
		for (T::VertexHandle vh : mesh.vertices())
		{
			int idx = vh.idx();
			Laplacian(idx, idx) = -1;
			int n_range = 0;
			for (auto vv : mesh.vv_range(vh))
				n_range++;
			for (auto vv : mesh.vv_range(vh))
			{
				Laplacian(idx, vv.idx()) = 1.0 / n_range;
			}
		}
	}
	else if (kind == 1)
	{
		if (LAR.rows() == 0)
		{
			std::cerr << "尚未求取顶点平均区域" << std::endl;
			
			return;
		}
		for (T::VertexHandle vh : mesh.vertices())
			Laplacian(vh.idx(), vh.idx()) = -1 / (2 * LAR(vh.idx()));
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
			Laplacian(fv[0], fv[1]) +=1/tan (angle3) / (2 * LAR[fv[0]]);
			Laplacian(fv[0], fv[0]) -= 1/tan(angle3) / (2 * LAR[fv[0]]);
			Laplacian(fv[1], fv[0]) += 1/tan(angle3) / (2 * LAR[fv[1]]);
			Laplacian(fv[1], fv[1]) -= 1/tan(angle3) / (2 * LAR[fv[1]]);
			
			Laplacian(fv[0], fv[2]) += 1/tan(angle2) / (2 * LAR[fv[0]]);
			Laplacian(fv[0], fv[0]) -= 1/tan(angle2) / (2 * LAR[fv[0]]);
			Laplacian(fv[2], fv[0]) += 1/tan(angle2) / (2 * LAR[fv[2]]);
			Laplacian(fv[2], fv[2]) -= 1/tan(angle2) / (2 * LAR[fv[2]]);

			Laplacian(fv[1], fv[2]) += 1/tan(angle1) / (2 * LAR[fv[1]]);
			Laplacian(fv[1], fv[1]) -= 1/tan(angle1) / (2 * LAR[fv[1]]);
			Laplacian(fv[2], fv[1]) += 1/tan(angle1) / (2 * LAR[fv[2]]);
			Laplacian(fv[2], fv[2]) -= 1/tan(angle1) / (2 * LAR[fv[2]]);
		}
	}
}
