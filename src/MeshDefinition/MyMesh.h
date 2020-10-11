#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>

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

class MyMesh
{
public:
	MyMesh();
	MyMesh(const MyMesh& copy);
	enum LAR_KIND { BARYCENTRIC, VORONOI, MIXED, LAR_ND };//定义、记录LAR的类型
	enum V_NORMAL_KIND { V_NORMAL_ND };
	enum CURVATURE_KIND { MEAN, ABSOLUTEMEAN, GAUSSIAN, CURVATURE_ND };
	enum LAPLACIAN_KIND { UNIFORM, CONTANGENT, LAPLACIAN_ND };
	enum EULER_INTEGRATION_KIND { EXPLICIT, IMPLICIT, EULER_INTEGRATION_ND };//欧拉积分的类型，显式和隐式
private:
	T mesh;
	//基础功能
	//计算三角形的三个角度
	void CalcTriangleAngles(T::Point& p1,T::Point& p2,T::Point& p3,double& angle1,double& angle2,double& angle3);
	//计算三角形的外心
	T::Point CalcTriangleCirumcenter(T::Point& p1, T::Point& p2, T::Point& p3);
	//计算三角形的面积
	double CalcTriangleArea(T::Point& p1, T::Point& p2, T::Point& p3);
public:
	bool Load(std::string s);
	bool Write(std::string s);
	void LoadVertex();
	void SaveVertex();
	void Clear();
	void UpdateNormals();
	int NVertices() const;
	int NEdges() const;
	int NFaces() const ;
	double CalcEdgeLength(int i);
	T::Normal getVertexNormal(int i) const;
	T::Normal getFaceNormal(int i) const;
	T::Point getPoint(int i) const;
	void getFaceVertices(int f, int& v1, int& v2, int& v3) const;
	void getEdgeVertices(int e, int& v1, int& v2) const;
	double getEdgeLength(int i) const;
	bool EdgeIsBoundary(int i) const;





	void BilateralDenoising(double, double);
	void BilateralNormalFiltering(double, double);

	void CalcTutte();
	void CalcLSCM();

protected://辅助子类的功能
	void ComputeLaplacian();//构建Laplacian矩阵
	void ComputeLAR();

public://主要在父类cpp中实现的内容
	   //计算网格的体积
	double ComputeMeshVolume();
	void MakeNoise();
	void Fairing(int);
	void Smoothing();

	void CalcCurvature();

	void getVCurvature(std::vector<double>& c);


	void SetLaplacianKind(LAPLACIAN_KIND k) { Laplacian_kind = k; };
	void SetLARKind(LAR_KIND k) { LAR_kind = k; };
	void SetCurvatureKind(CURVATURE_KIND k) { Curvature_kind = k; };
	void SetEulerIntegrationKind(EULER_INTEGRATION_KIND k) { Euler_integration_kind = k; };

protected:

	Eigen::VectorXd LAR; //局部平均区域的面积
	LAR_KIND LAR_kind = LAR_KIND::LAR_ND;

	Eigen::MatrixXd Vertices;//网格顶点Eigen::Matrix3d FaceNormal;//面法向，唯一

	Eigen::MatrixXd VertexNormal;//顶点法向，有三种加权方式，不过这个暂时不管(因为Openmesh提供了默认的)

	Eigen::MatrixXd FaceNormal;

	Eigen::VectorXd Curvature;//顶点的曲率
	CURVATURE_KIND Curvature_kind = CURVATURE_KIND::CURVATURE_ND;

	Eigen::SparseMatrix<double> Laplacian;
	LAPLACIAN_KIND Laplacian_kind = LAPLACIAN_KIND::LAPLACIAN_ND;

	EULER_INTEGRATION_KIND Euler_integration_kind = EULER_INTEGRATION_KIND::EULER_INTEGRATION_ND;

	double MeshVolume;
};