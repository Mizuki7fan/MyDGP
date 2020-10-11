#include "DGPAlgorithm.h"
#include <fstream>

void DGPAlgorithm::ComputeCurvature(int LARKind, int CurvatureKind, MyMesh& mesh, std::vector<double>& curvature)
{
	mesh.SetLARKind(MyMesh::LAR_KIND(LARKind));
	mesh.SetCurvatureKind(MyMesh::CURVATURE_KIND(CurvatureKind));
	mesh.CalcCurvature();
	mesh.getVCurvature(curvature);
}

void DGPAlgorithm::MakeNoise(MyMesh& mesh)
{
	mesh.MakeNoise();
}

void DGPAlgorithm::DoFairing(MyMesh& mesh, int power,int lap_kind)
{
	mesh.SetLaplacianKind(MyMesh::LAPLACIAN_KIND(lap_kind));
	mesh.Fairing(power);
}

void DGPAlgorithm::DoSmoothing(MyMesh& mesh, int laplacekind,int integrationkind)
{
	mesh.SetLaplacianKind(MyMesh::LAPLACIAN_KIND(laplacekind));
	mesh.SetEulerIntegrationKind(MyMesh::EULER_INTEGRATION_KIND(integrationkind));
	mesh.Smoothing();
}

void DGPAlgorithm::DoBilateralDenoising(MyMesh& mesh, double stdevs,double stdevr)
{
	mesh.BilateralDenoising(stdevs,stdevr);
}

void DGPAlgorithm::DoBilateralNormalFiltering(MyMesh& mesh, double stdevs, double stdevr)
{
	mesh.BilateralNormalFiltering(stdevs, stdevr);
}

void DGPAlgorithm::CalcTutte(MyMesh& mesh)
{
	mesh.CalcTutte();
}

void DGPAlgorithm::CalcLSCM(MyMesh& mesh)
{
}
