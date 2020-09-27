#include "MyMesh_Openmesh.h"


Mesh::Mesh()
{
	mesh.update_vertex_normals();
	mesh.request_vertex_normals();
}

bool Mesh::Load(std::string s)
{
	return OpenMesh::IO::read_mesh(mesh, s);
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

void Mesh::ComputeLaplacian(int kind)
{
	//使用Openmesh的函数功能来填装Laplacian
	int nv = mesh.n_vertices();
	Laplacian.resize(nv, nv);
	if (kind == 1)
	{
		//如何快速构建uniform的Laplacian
		for (T::VertexHandle vh : mesh.vertices())
		{
			int idx = vh.idx();
			Laplacian(idx, idx) = 1;
			int n_range = 0;
			for (auto vv : mesh.vv_range(vh))
				n_range++;
			for (auto vv : mesh.vv_range(vh))
			{
				Laplacian(idx, vv.idx()) = -1.0 / n_range;
			}
		}
	}
	else if (kind == 2)
	{

	}

}
