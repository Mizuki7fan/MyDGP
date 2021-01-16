#include "MeshUtilities.h"

double MeshUtility::CalcTriangleArea(T::Point& p1, T::Point& p2, T::Point& p3)
{
	OpenMesh::Vec3d e12 = (p2 - p1);
	OpenMesh::Vec3d e13 = (p3 - p1);
	double area = 0.5 * (e12 % e13).norm();
	return area;
}

void MeshUtility::GetFaceVertices(T& m, T::FaceHandle& f, T::VertexHandle& v0, T::VertexHandle& v1, T::VertexHandle& v2)
{
	T::VertexHandle V[3];
	int id = 0;
	for (auto vh : m.fv_range(f))
	{
		V[id] = vh;
		id++;
	}
	v0 = V[0];
	v1 = V[1];
	v2 = V[2];
}

void MeshUtility::CalcVertexAngle(T::Point& p1, T::Point& p2, T::Point& p3, double& angle1, double& angle2, double& angle3)
{
	//基于余弦定理
	double e12 = (p2 - p1).length();
	double e13 = (p3 - p1).length();
	double e23 = (p3 - p2).length();
	angle1 = acos((e12 * e12 + e13 * e13 - e23 * e23) / (2 * e12 * e13));
	angle2 = acos((e12 * e12 + e23 * e23 - e13 * e13) / (2 * e12 * e23));
	angle3 = acos((e13 * e13 + e23 * e23 - e12 * e12) / (2 * e13 * e23));
	return;
}

