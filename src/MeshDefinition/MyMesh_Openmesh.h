#pragma once
#include "MyMesh.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#ifdef _DEBUG
#pragma comment(lib, "OpenMeshCored.lib")
#pragma comment(lib, "OpenMeshToolsd.lib")
#else
#pragma comment(lib, "OpenMeshCore.lib")
#pragma comment(lib, "OpenMeshTools.lib")
#endif
//使用Openmesh来实现各种mesh的效果
struct MeshTraits : public OpenMesh::DefaultTraits
{
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;
	VertexAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::Normal);
	FaceAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::Normal);
	EdgeAttributes(OpenMesh::Attributes::Status);
	HalfedgeAttributes(OpenMesh::Attributes::Status);
};
typedef OpenMesh::TriMesh_ArrayKernelT<MeshTraits> T;

class Mesh:public MyMesh
{
public:
	Mesh();
    bool Load(std::string);
	void LoadVertex();
	bool Write(std::string);
	void Clear();
	void UpdateNormals();
	void LoadVerticeNormal(void);
	void LoadFaceNormal(void);
	bool VerticesEmpty();
	int NVertices() const;
	int NEdges() const;
	int NFaces() const;
	double CalcEdgeLength(int i);
	double CalcFaceArea();
	double getEdgeLength(int) const;
	Eigen::Vector3d getPoint(int i) const;
	Eigen::Vector3d getVertexNormal(int i) const;
	Eigen::Vector3d getFaceNormal(int i) const;
	void getEdgeVertices(int e, int& v1, int& v2) const;
	void getFaceVertices(int f, int& v1, int& v2, int& v3) const;
	void getFaceAngles(int f, double& angle1, double& angle2, double& angle3) const;
	bool isBoundary(int) const;
	bool isBoundaryVertex(int) const;
	void SetVerticesNewCoord();
	void SetVertexNewCoord(int, Eigen::Vector3d);
	Eigen::Vector3d getVertexCoord(int);
	double ComputeMeshVolume();
	void ComputeLaplacian();
	void ComputeLaplacian(int);

	T mesh;
private:
	T::Point toPoint(Eigen::Vector3d v) { return T::Point(v.x(), v.y(), v.z()); }
};