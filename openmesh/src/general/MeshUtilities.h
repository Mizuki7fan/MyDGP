#pragma once
#include "MeshDefinition.h"

namespace MeshUtility{
	//根据三顶点，求取三角形面积
	double CalcTriangleArea(T::Point& p1, T::Point& p2, T::Point& p3);

	//根据网格的面，获取其三个顶点
	void GetFaceVertices(T& m, T::FaceHandle& f, T::VertexHandle& v0, T::VertexHandle& v1, T::VertexHandle& v2);

	//根据三个顶点，基于余弦定理，求取对应的角度值
	void CalcVertexAngle(T::Point& p1, T::Point& p2, T::Point& p3,double&angle1,double&angle2,double&angle3);
}
