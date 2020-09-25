#include "DGPAlgorithm.h"
/*
void DGPAlgorithm::ComputeCurvature(const MyMesh& mesh, std::vector<double>& curvature)
{
	int N = mesh.NVertices();
	curvature.resize(N);
	for (int i = 0; i < mesh.NVertices(); i++)
	{
		curvature[i] = 1-i / double(N);
	}
}
*/
void DGPAlgorithm::ComputeMeanCurvature( MyMesh& mesh, std::vector<double>& curvature)
{//算平均曲率
	//对于每个点，算其平均区域的信息
	mesh.UpdateMeanCurvature();
	mesh.getVCurvature(curvature);

}

void DGPAlgorithm::ComputeGaussianCurvature( MyMesh& mesh, std::vector<double>& curvature)
{
	//double area = mesh.CalcFaceArea();
	mesh.UpdateGaussianCurvature();
	mesh.getVCurvature(curvature);
}
