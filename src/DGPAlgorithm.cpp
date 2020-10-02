#include "DGPAlgorithm.h"
#include <fstream>

void DGPAlgorithm::ComputeCurvature(int localaverageregionKind, int curvatureKind, MyMesh& mesh, std::vector<double>& curvature)
{
	mesh.SetLARKind(localaverageregionKind);
	mesh.SetCurvatureKind(curvatureKind);
	mesh.ComputeCurvature();
	mesh.getVCurvature(curvature);
}

void DGPAlgorithm::MakeNoise(MyMesh& mesh)
{
	mesh.MakeNoise();
}

void DGPAlgorithm::DoFairing(MyMesh& mesh, int power)
{
	mesh.Fairing(power);
}

void DGPAlgorithm::DoSmoothing(MyMesh& mesh, int laplacekind,int integrationkind)
{
	mesh.SetLaplacianKind(laplacekind);
	mesh.Smoothing(integrationkind);
}
