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
}

void Mesh::SetVertexNewCoord(int i, Eigen::Vector3d p)
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
	if (Laplacian_latest)
		return;
	CheckProperty(PROPERTY::P_LAPLACIAN);
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
	{
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
	Laplacian_latest = true;
}

void Mesh::ComputeLaplacian(int kind)
{
//	Laplacian_latest = false;
//	Laplacian_kind = kind;
	ComputeLAR();
	ComputeLaplacian();
}
