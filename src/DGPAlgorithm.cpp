#include "DGPAlgorithm.h"
#include <fstream>

void DGPAlgorithm::ComputeCurvature(int LARKind, int CurvatureKind, MyMesh& mesh, std::vector<double>& curvature)
{
	mesh.SetLARKind(MyMesh::LAR_KIND(LARKind));
	mesh.SetCurvatureKind(MyMesh::CURVATURE_KIND(CurvatureKind));
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
//	mesh.SetLaplacianKind(laplacekind);
//	mesh.Smoothing(integrationkind);
}
