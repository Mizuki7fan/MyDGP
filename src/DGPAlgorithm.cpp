#include "DGPAlgorithm.h"

void DGPAlgorithm::ComputeCurvature(int localaverageregionKind, int curvatureKind, MyMesh& mesh, std::vector<double>& curvature)
{
	mesh.ComputeLocalAveragingRegion(localaverageregionKind);
	switch (curvatureKind)
	{
	case 0:
		ComputeMeanCurvature(mesh, curvature);
		break;
	case 1:
		ComputeMeanCurvature(mesh, curvature);
		for (int i = 0; i < curvature.size(); i++)
			if (curvature[i] < 0) curvature[i] = -curvature[i];
		break;
	case 2:
		ComputeGaussianCurvature(mesh, curvature);
		break;
	default:
		break;
	}
}
void DGPAlgorithm::ComputeMeanCurvature( MyMesh& mesh, std::vector<double>& curvature)
{//算平均曲率
	//对于每个点，算其平均区域的信息
	mesh.UpdateMeanCurvature();
	mesh.getVCurvature(curvature);

}

void DGPAlgorithm::ComputeGaussianCurvature( MyMesh& mesh, std::vector<double>& curvature)
{
	mesh.UpdateGaussianCurvature();
	mesh.getVCurvature(curvature);
}

void DGPAlgorithm::MakeNoise(MyMesh& mesh)
{
	mesh.UpdateNormals();
	mesh.MakeNoise();
	//先算平均边长，然后算点的法向，在令点沿着法向随机平移
	
}
